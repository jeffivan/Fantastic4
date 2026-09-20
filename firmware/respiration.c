/**
 * @file    respiration.c
 * @brief   Respiration rate from respiratory sinus arrhythmia (RSA).
 *
 * WHY THIS WORKS
 *   Inspiration inhibits vagal tone, so the heart briefly speeds up;
 *   expiration restores it and the heart slows. The result is that the R-R
 *   interval series is amplitude-modulated at the breathing frequency. Plot
 *   R-R against time and you are looking at a (noisy, non-uniformly sampled)
 *   recording of the subject's breathing. The whole module is the machinery
 *   needed to turn that into a number.
 *
 * WHY EACH STEP EXISTS
 *   1. RESAMPLE to a uniform 4 Hz grid. The tachogram is sampled at the
 *      heartbeats, which are exactly the thing being modulated - an inherently
 *      non-uniform grid. An FFT assumes uniform sampling, so feeding it raw
 *      beat indices would smear the spectrum and bias the answer toward the
 *      heart rate. Linear interpolation onto a fixed grid is the standard,
 *      cheap fix (Berger's method is better but needs more code than a
 *      hackathon justifies).
 *   2. 128 samples at 4 Hz = 32 s. Long enough to hold ~4.5 cycles of the
 *      slowest breath we accept (9 brpm), short enough that the rate has not
 *      meaningfully changed across the window.
 *   3. DETREND. A 32 s window almost always contains a slow heart-rate drift.
 *      Left in, that ramp leaks broadband energy across the low bins and can
 *      out-shout the respiratory peak. Removing the mean kills the DC bin;
 *      the Hann window handles the rest of the leakage.
 *   4. HANN WINDOW. The breath is not periodic in the window, so the record
 *      ends with a discontinuity. Hann's -31 dB first sidelobe keeps that
 *      discontinuity from painting energy all over the 8-bin respiratory band.
 *      It costs resolution (the main lobe is 4 bins wide) - which is precisely
 *      why step 6 exists.
 *   5. arm_rfft_fast_f32, 128-point real FFT, on the FPU.
 *   6. SEARCH ONLY 0.15-0.40 Hz. Below that sits the Mayer wave (~0.1 Hz,
 *      baroreflex) which is often the largest peak in an HRV spectrum and has
 *      nothing to do with breathing. Above it there is nothing but noise.
 *   7. PARABOLIC INTERPOLATION. Bin spacing is 4/128 = 0.03125 Hz = 1.875
 *      breaths/min, so the raw bin index quantises the answer far too coarsely
 *      to be clinically interesting. Fitting a parabola through the peak bin
 *      and its two neighbours recovers the true peak location to a fraction of
 *      a bin, taking the quantisation error to roughly +/-0.2 brpm.
 *   8. CONFIDENCE from peak prominence. A real breathing rhythm concentrates
 *      energy into one or two bins; motion artefact and detector errors give a
 *      flat band. peak/mean is a direct measure of that, and the module says
 *      "I don't know" rather than emitting a number from a flat spectrum.
 */

#include "respiration.h"
#include "config.h"
#include "ecg.h"
#include <string.h>
#include <math.h>

#define RESP_HALF           (RESP_FFT_LEN / 2U)
#define RESP_BIN_HZ         (RESP_RESAMPLE_HZ / (float32_t)RESP_FFT_LEN)
#define RESP_GRID_MS        (1000.0f / RESP_RESAMPLE_HZ)
#define PI_F                3.14159265358979f

/* ===================================================================== */
/* State                                                                 */
/* ===================================================================== */

static ecg_beat_t s_beats[RESP_MAX_BEATS];
static float32_t  s_bt[RESP_MAX_BEATS];     /* beat times, ms   (compacted)   */
static float32_t  s_brr[RESP_MAX_BEATS];    /* beat intervals, ms (compacted) */

static float32_t  s_tacho[RESP_FFT_LEN];    /* uniform 4 Hz tachogram         */
static float32_t  s_fft_in[RESP_FFT_LEN];   /* windowed input (DESTROYED)     */
static float32_t  s_fft_out[RESP_FFT_LEN];  /* packed real-FFT output         */
static float32_t  s_mag[RESP_HALF + 1U];    /* magnitude spectrum             */
static float32_t  s_hann[RESP_FFT_LEN];

static arm_rfft_fast_instance_f32 s_rfft;
static uint32_t s_bin_lo, s_bin_hi;
static respiration_result_t s_result;

/* ===================================================================== */
/* Init                                                                  */
/* ===================================================================== */

void respiration_init(void)
{
    uint32_t i;

    memset(&s_result, 0, sizeof(s_result));
    s_result.rate_brpm = 0.0f;
    s_result.valid     = 0U;

    /* Newer CMSIS-DSP packs also offer arm_rfft_fast_init_128_f32(); this call
     * remains available and keeps the length in one place. */
    (void)arm_rfft_fast_init_f32(&s_rfft, (uint16_t)RESP_FFT_LEN);

    for (i = 0U; i < RESP_FFT_LEN; i++) {
        s_hann[i] = 0.5f * (1.0f - cosf(2.0f * PI_F * (float32_t)i
                                        / (float32_t)(RESP_FFT_LEN - 1U)));
    }

    /* Bin range for the respiratory band, inclusive. ceil/floor so we never
     * search outside the band; clamped to leave room for the parabolic
     * interpolation's neighbours.
     *
     * With the shipped constants this is bins 5..12 (0.15625..0.375 Hz). After
     * refinement the reportable range is roughly 9.4..23.4 brpm rather than a
     * clean 9..24: the band edges do not fall on bin centres. Widen
     * RESP_BAND_HIGH_HZ to 0.42 if you need the top of the range, at the cost
     * of letting one more noise bin into the prominence calculation. */
    s_bin_lo = (uint32_t)ceilf (RESP_BAND_LOW_HZ  / RESP_BIN_HZ);
    s_bin_hi = (uint32_t)floorf(RESP_BAND_HIGH_HZ / RESP_BIN_HZ);
    if (s_bin_lo < 1U)              { s_bin_lo = 1U; }
    if (s_bin_hi > (RESP_HALF - 1U)){ s_bin_hi = RESP_HALF - 1U; }
}

/* ===================================================================== */
/* Step 1-2: build the uniform tachogram                                 */
/* ===================================================================== */

/**
 * @return number of valid beats used, or 0 if the window cannot be filled.
 */
static uint32_t build_tachogram(uint32_t now_ms)
{
    uint32_t n, m = 0U, i, k, j;
    float32_t t_start, t_end;

    if (now_ms < RESP_WINDOW_MS) { return 0U; }     /* not enough uptime yet   */

    t_end   = (float32_t)now_ms;
    t_start = (float32_t)(now_ms - RESP_WINDOW_MS);

    n = ecg_copy_beats(s_beats, RESP_MAX_BEATS);

    /* Compact to the valid beats that bracket the window. One extra beat on
     * each side is kept so the first and last grid points can be interpolated
     * rather than clamped. */
    for (i = 0U; i < n; i++) {
        const float32_t t = (float32_t)s_beats[i].t_ms;
        if (!s_beats[i].valid) { continue; }
        if (t < (t_start - (float32_t)RESP_EDGE_TOLERANCE_MS)) { continue; }
        if (t > (t_end   + (float32_t)RESP_EDGE_TOLERANCE_MS)) { continue; }
        s_bt[m]  = t;
        s_brr[m] = s_beats[i].rr_ms;
        m++;
    }

    if (m < RESP_MIN_BEATS) { return 0U; }

    /* Coverage: a window with a 10 s hole in it will happily produce a
     * beautiful spurious peak. Require real beats near both edges. */
    if (s_bt[0]       > (t_start + (float32_t)RESP_EDGE_TOLERANCE_MS)) { return 0U; }
    if (s_bt[m - 1U]  < (t_end   - (float32_t)RESP_EDGE_TOLERANCE_MS)) { return 0U; }

    j = 0U;
    for (k = 0U; k < RESP_FFT_LEN; k++) {
        const float32_t t_k = t_start + (float32_t)k * RESP_GRID_MS;

        /* Single forward walk: the grid is monotonic, so j never rewinds and
         * the whole resampling is O(beats + grid), not O(beats * grid). */
        while ((j + 1U) < m && s_bt[j + 1U] <= t_k) { j++; }

        if (t_k <= s_bt[0]) {
            s_tacho[k] = s_brr[0];                  /* clamp at the left edge  */
        } else if (t_k >= s_bt[m - 1U]) {
            s_tacho[k] = s_brr[m - 1U];             /* clamp at the right edge */
        } else {
            const float32_t t0 = s_bt[j],  t1 = s_bt[j + 1U];
            const float32_t v0 = s_brr[j], v1 = s_brr[j + 1U];
            const float32_t dt = t1 - t0;
            s_tacho[k] = (dt > 0.0f) ? (v0 + (v1 - v0) * ((t_k - t0) / dt)) : v0;
        }
    }

    return m;
}

/* ===================================================================== */
/* Step 3-8: spectrum                                                    */
/* ===================================================================== */

uint8_t respiration_update(uint32_t now_ms)
{
    float32_t mean, peak, band_sum, ratio, conf;
    float32_t m0, m1, m2, den, delta, f_hz;
    uint32_t  beats, k, k_peak;

    beats = build_tachogram(now_ms);
    if (beats == 0U) {
        s_result.valid      = 0U;
        s_result.confidence = 0.0f;
        s_result.beats_used = 0U;
        return 0U;
    }

    /* 3. Detrend. arm_mean_f32 + arm_offset_f32 rather than a hand loop: both
     *    are FPU-vectorised and the intent is obvious. */
    arm_mean_f32(s_tacho, RESP_FFT_LEN, &mean);
    arm_offset_f32(s_tacho, -mean, s_fft_in, RESP_FFT_LEN);

    /* 4. Hann window, in place. */
    arm_mult_f32(s_fft_in, s_hann, s_fft_in, RESP_FFT_LEN);

    /* 5. Real FFT. NOTE: arm_rfft_fast_f32 runs an in-place complex FFT over
     *    its input buffer, so s_fft_in is destroyed here. That is why the
     *    windowed data lives in a scratch buffer and s_tacho is kept intact
     *    (it is useful to dump over UART when debugging). */
    arm_rfft_fast_f32(&s_rfft, s_fft_in, s_fft_out, 0U);

    /* Unpack: out[0] = Re(X[0]), out[1] = Re(X[N/2]), then interleaved
     * Re/Im for bins 1..N/2-1. */
    s_mag[0]         = fabsf(s_fft_out[0]);
    arm_cmplx_mag_f32(&s_fft_out[2], &s_mag[1], RESP_HALF - 1U);
    s_mag[RESP_HALF] = fabsf(s_fft_out[1]);

    /* 6. Peak search restricted to the respiratory band. */
    peak     = -1.0f;
    k_peak   = s_bin_lo;
    band_sum = 0.0f;
    for (k = s_bin_lo; k <= s_bin_hi; k++) {
        band_sum += s_mag[k];
        if (s_mag[k] > peak) { peak = s_mag[k]; k_peak = k; }
    }

    /* 8a. Prominence. The mean includes the peak itself, so the ratio is
     *     bounded by the bin count (8) - documented, not a bug. */
    {
        const float32_t nbins = (float32_t)(s_bin_hi - s_bin_lo + 1U);
        const float32_t band_mean = band_sum / nbins;
        ratio = (band_mean > 1e-9f) ? (peak / band_mean) : 0.0f;
    }

    /* 7. Parabolic (quadratic) interpolation over the three bins around the
     *    maximum. den < 0 at a genuine maximum; anything else means the peak
     *    sits on the band edge and we keep the bin centre. */
    m0  = s_mag[k_peak - 1U];
    m1  = s_mag[k_peak];
    m2  = s_mag[k_peak + 1U];
    den = m0 - 2.0f * m1 + m2;

    if (den < -1e-12f) {
        delta = 0.5f * (m0 - m2) / den;
        if (delta >  0.5f) { delta =  0.5f; }
        if (delta < -0.5f) { delta = -0.5f; }
    } else {
        delta = 0.0f;
    }

    f_hz = ((float32_t)k_peak + delta) * RESP_BIN_HZ;

    /* 8b. Publish. Everything that could make this wrong is a reason to say
     *     "invalid" rather than to emit a plausible-looking number. */
    conf = (ratio - RESP_CONF_RATIO_MIN)
         / (RESP_CONF_RATIO_FULL - RESP_CONF_RATIO_MIN);
    if (conf < 0.0f) { conf = 0.0f; }
    if (conf > 1.0f) { conf = 1.0f; }

    s_result.peak_hz    = f_hz;
    s_result.rate_brpm  = f_hz * 60.0f;
    s_result.peak_ratio = ratio;
    s_result.confidence = conf;
    s_result.beats_used = (uint16_t)beats;

    /* The refinement can move an edge bin by up to half a bin, which may land
     * just outside the search band. That is the resolution limit talking, not
     * an out-of-band peak, so allow it rather than clamping (clamping would
     * pile up spurious readings exactly on 9.0 and 24.0 brpm). */
    s_result.valid      = (ratio >= RESP_CONF_RATIO_MIN &&
                           beats >= RESP_MIN_BEATS &&
                           f_hz  >= (RESP_BAND_LOW_HZ  - 0.5f * RESP_BIN_HZ) &&
                           f_hz  <= (RESP_BAND_HIGH_HZ + 0.5f * RESP_BIN_HZ)) ? 1U : 0U;

    return s_result.valid;
}

const respiration_result_t *respiration_get(void)
{
    return &s_result;
}
