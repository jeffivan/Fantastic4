/**
 * @file    ecg.c
 * @brief   250 Hz ECG acquisition and Pan-Tompkins QRS detection.
 *
 * SIGNAL CHAIN
 *   ADC (12-bit counts)
 *     -> band-pass 5-15 Hz      : isolate the QRS energy band. Everything the
 *                                 detector is fooled by lives outside it -
 *                                 baseline wander below 5 Hz, T-waves around
 *                                 1-7 Hz (mostly attenuated), EMG and mains
 *                                 above 15 Hz.
 *     -> 5-point derivative     : QRS is defined by its SLOPE, not its height.
 *                                 Differentiating turns the steep R upstroke
 *                                 into the largest thing in the signal and
 *                                 suppresses the slow P and T waves further.
 *     -> square                 : makes everything positive (so up- and
 *                                 down-strokes reinforce) and non-linearly
 *                                 amplifies the large QRS slope over noise.
 *     -> 150 ms moving window   : integrates the squared slope over roughly
 *                                 one QRS duration, collapsing the biphasic
 *                                 complex into a single smooth hump whose
 *                                 width carries QRS duration information.
 *     -> adaptive threshold     : the amplitude of an ECG depends on electrode
 *                                 placement, skin prep and the subject. A
 *                                 fixed threshold is useless, so we track
 *                                 running signal (SPKI) and noise (NPKI) peak
 *                                 estimates and put the threshold between them.
 *     -> fiducial refinement    : the MWI hump peaks ~85 ms AFTER the R wave
 *                                 (2-sample derivative delay + half the 150 ms
 *                                 window). For heart rate that offset cancels,
 *                                 but respiratory sinus arrhythmia is a few
 *                                 tens of milliseconds of R-R modulation, so
 *                                 we go back into the band-passed signal and
 *                                 take the true |peak| as the beat time.
 *
 * TIMEBASE
 *   Beat timestamps are derived from the ADC sample counter, not HAL_GetTick().
 *   The R-R series feeds an FFT; using a counter that is phase-locked to the
 *   same clock that produced the samples removes any relative jitter between
 *   "when the sample was taken" and "what time we think it is".
 */

#include "ecg.h"
#include "config.h"
#include <string.h>
#include <math.h>

/* 1000 ms / 250 Hz must be an exact integer for the timestamp macro below. */
#if (1000U % ECG_SAMPLE_RATE_HZ) != 0U
#error "ECG_SAMPLE_RATE_HZ must divide 1000 exactly"
#endif
#define MS_PER_SAMPLE           (1000U / ECG_SAMPLE_RATE_HZ)
#define SAMPLES_TO_MS(n)        ((uint32_t)(n) * MS_PER_SAMPLE)
#define MS_TO_SAMPLES(ms)       ((uint32_t)(ms) / MS_PER_SAMPLE)

#define MWI_LEN                 ((ECG_SAMPLE_RATE_HZ * ECG_MWI_WINDOW_MS) / 1000U)
#define REFRACTORY_SAMPLES      MS_TO_SAMPLES(ECG_REFRACTORY_MS)
#define LEARN_SAMPLES           MS_TO_SAMPLES(ECG_LEARN_MS)
#define FIDUCIAL_SEARCH         (MWI_LEN + ECG_FIDUCIAL_SEARCH_EXTRA)

/* Band-passed history. It has to cover more than the fiducial search window:
 * a searchback can accept a beat up to ECG_THR_SEARCHBACK_FACTOR * RR_max in
 * the past (1.66 * 2000 ms = 3.3 s = 830 samples), and we still want to refine
 * that beat's fiducial point. 1024 samples = 4.1 s costs 4 KB of the 64 KB.
 * Power of two so the circular index is a mask, not a modulo. */
#define BP_HIST_LEN             1024U
#define BP_HIST_MASK            (BP_HIST_LEN - 1U)
#if (BP_HIST_LEN <= (MWI_LEN + ECG_FIDUCIAL_SEARCH_EXTRA))
#error "BP_HIST_LEN too small for the fiducial search window"
#endif

#define BEAT_MASK               (ECG_BEAT_BUF_LEN - 1U)
#define STREAM_MASK             (ECG_STREAM_FIFO_LEN - 1U)

#define PI_F                    3.14159265358979f

/* ===================================================================== */
/* State                                                                 */
/* ===================================================================== */

static ADC_HandleTypeDef *s_hadc;
static TIM_HandleTypeDef *s_htim;

/* --- DMA ping-pong --- */
static uint16_t          s_dma_buf[ECG_DMA_BUF_LEN];
static volatile uint8_t  s_block_ready;      /* bit0 = lower half, bit1 = upper */
static volatile uint32_t s_block_overruns;

/* --- per-block scratch (main context only) --- */
static float32_t s_blk_in[ECG_DMA_HALF_LEN];
static float32_t s_blk_bp[ECG_DMA_HALF_LEN];

/* --- band-pass: high-pass biquad followed by low-pass biquad --- */
static arm_biquad_casd_df1_inst_f32 s_bp_inst;
static float32_t s_bp_coeffs[2 * 5];         /* {b0,b1,b2,-a1,-a2} per stage   */
static float32_t s_bp_state[2 * 4];

/* --- derivative delay line: x[n-1] .. x[n-4] --- */
static float32_t s_d_z[4];

/* --- moving window integrator --- */
static float32_t s_mwi_buf[MWI_LEN];
static uint32_t  s_mwi_idx;
static float32_t s_mwi_sum;
static uint32_t  s_mwi_resync;               /* countdown to a sum rebuild     */

/* --- band-passed history for fiducial refinement --- */
static float32_t s_bp_hist[BP_HIST_LEN];
static uint32_t  s_bp_hist_wr;               /* index of the NEXT write        */

/* --- detector --- */
static uint32_t  s_sample_index;             /* samples since ecg_init()       */
static float32_t s_mwi_prev1, s_mwi_prev2;   /* for local-maximum detection    */
static float32_t s_spki, s_npki, s_thr;
static uint32_t  s_last_qrs_idx;             /* MWI index of the last accepted QRS */
static uint32_t  s_last_r_idx;               /* refined fiducial index          */
static uint8_t   s_have_last_r;
static float32_t s_rr_mean_samples;          /* smoothed R-R, used by searchback */
/* searchback: best sub-threshold candidate since the last accepted QRS */
static float32_t s_sb_val;
static uint32_t  s_sb_idx;
/* learning phase 1 */
static uint8_t   s_learning;
static float32_t s_learn_max;
static float32_t s_learn_sum;
static uint32_t  s_learn_n;

/* --- beats --- */
static ecg_beat_t s_beats[ECG_BEAT_BUF_LEN];
static uint32_t   s_beat_count;              /* total beats ever recorded      */
static uint32_t   s_last_beat_ms;

/* --- LED --- */
static uint32_t  s_led_off_ms;
static uint8_t   s_led_on;

/* --- raw sample stream to the host --- */
static uint16_t          s_stream_fifo[ECG_STREAM_FIFO_LEN];
static volatile uint32_t s_stream_wr, s_stream_rd;
static uint32_t          s_stream_drops;
static uint32_t          s_dec_count;
static uint32_t          s_dec_acc;

/* ===================================================================== */
/* Biquad design (RBJ cookbook, evaluated once on the FPU at init)       */
/* ===================================================================== */

/* CMSIS DF1 expects {b0, b1, b2, a1, a2} with a1/a2 already NEGATED and all
 * five terms divided by a0, because it computes
 *   y[n] = b0 x[n] + b1 x[n-1] + b2 x[n-2] + a1 y[n-1] + a2 y[n-2].
 */
static void design_highpass(float32_t fc, float32_t fs, float32_t q, float32_t *c)
{
    const float32_t w0    = 2.0f * PI_F * fc / fs;
    const float32_t cw    = cosf(w0);
    const float32_t sw    = sinf(w0);
    const float32_t alpha = sw / (2.0f * q);
    const float32_t a0    = 1.0f + alpha;

    c[0] =  ((1.0f + cw) * 0.5f) / a0;
    c[1] = -(1.0f + cw) / a0;
    c[2] =  c[0];
    c[3] =  (2.0f * cw) / a0;
    c[4] = -(1.0f - alpha) / a0;
}

static void design_lowpass(float32_t fc, float32_t fs, float32_t q, float32_t *c)
{
    const float32_t w0    = 2.0f * PI_F * fc / fs;
    const float32_t cw    = cosf(w0);
    const float32_t sw    = sinf(w0);
    const float32_t alpha = sw / (2.0f * q);
    const float32_t a0    = 1.0f + alpha;

    c[0] = ((1.0f - cw) * 0.5f) / a0;
    c[1] =  (1.0f - cw) / a0;
    c[2] =  c[0];
    c[3] =  (2.0f * cw) / a0;
    c[4] = -(1.0f - alpha) / a0;
}

/* ===================================================================== */
/* Small helpers                                                         */
/* ===================================================================== */

/* The LED pulse is timed on SysTick, not the ADC sample counter. The sample
 * counter only advances while a DMA block is being processed, so a 25 ms
 * "pulse" measured in that timebase would expire within the same block and
 * the flash would never be visible. */
static inline void led_on(void)
{
    HAL_GPIO_WritePin(CFG_LED_PORT, CFG_LED_PIN, CFG_LED_ON_STATE);
    s_led_on     = 1U;
    s_led_off_ms = HAL_GetTick() + ECG_LED_PULSE_MS;
}

static void led_service(void)
{
    if (s_led_on && (int32_t)(HAL_GetTick() - s_led_off_ms) >= 0) {
        HAL_GPIO_WritePin(CFG_LED_PORT, CFG_LED_PIN,
                          (CFG_LED_ON_STATE == GPIO_PIN_RESET) ? GPIO_PIN_SET
                                                               : GPIO_PIN_RESET);
        s_led_on = 0U;
    }
}

/** Read a band-passed sample by absolute sample index (must be recent). */
static inline float32_t bp_at(uint32_t abs_index)
{
    const uint32_t back = s_sample_index - abs_index;  /* 0 == most recent */
    return s_bp_hist[(s_bp_hist_wr - 1U - back) & BP_HIST_MASK];
}

static void stream_push(uint16_t v)
{
    const uint32_t next = (s_stream_wr + 1U) & STREAM_MASK;
    if (next == s_stream_rd) {          /* host is not keeping up - drop, never block */
        s_stream_drops++;
        return;
    }
    s_stream_fifo[s_stream_wr] = v;
    s_stream_wr = next;
}

/* ===================================================================== */
/* Beat bookkeeping                                                      */
/* ===================================================================== */

static void record_beat(uint32_t r_idx, float32_t amp)
{
    ecg_beat_t *b = &s_beats[s_beat_count & BEAT_MASK];

    b->t_ms = SAMPLES_TO_MS(r_idx);
    b->amp  = amp;

    if (s_have_last_r && r_idx > s_last_r_idx) {
        b->rr_ms = (float32_t)(r_idx - s_last_r_idx) * (float32_t)MS_PER_SAMPLE;
        /* The plausibility gate is the cheapest artefact rejector we have: a
         * 120 ms or 4 s "interval" is a detector error, not a heartbeat. */
        b->valid = (b->rr_ms >= ECG_RR_MIN_MS && b->rr_ms <= ECG_RR_MAX_MS) ? 1U : 0U;
    } else {
        b->rr_ms = 0.0f;
        b->valid = 0U;                  /* first beat has no interval          */
    }

    if (b->valid) {
        /* Smoothed R-R drives the searchback timeout. */
        const float32_t rr_samples = b->rr_ms / (float32_t)MS_PER_SAMPLE;
        s_rr_mean_samples = (s_rr_mean_samples > 0.0f)
                          ? (0.75f * s_rr_mean_samples + 0.25f * rr_samples)
                          : rr_samples;
    }

    s_last_r_idx   = r_idx;
    s_have_last_r  = 1U;
    s_last_beat_ms = b->t_ms;
    s_beat_count++;

    led_on();
}

/**
 * Walk backwards through the band-passed signal to find the real R fiducial,
 * then log the beat. Returns nothing; refuses to go backwards in time.
 */
static void accept_qrs(uint32_t mwi_idx)
{
    uint32_t  best_idx = mwi_idx;
    float32_t best_val = 0.0f;
    uint32_t  i;

    /* Only refine if the whole search window is still inside the history
     * ring; a very old searchback candidate keeps its unrefined timestamp
     * rather than reading recycled samples. */
    const uint32_t oldest_needed = (s_sample_index - mwi_idx) + FIDUCIAL_SEARCH;

    if (oldest_needed <= BP_HIST_LEN) {
        for (i = 0U; i < FIDUCIAL_SEARCH; i++) {
            const uint32_t idx = mwi_idx - i;
            float32_t      v;
            if (idx > mwi_idx) { break; }          /* underflow guard at boot   */
            v = fabsf(bp_at(idx));
            if (v > best_val) { best_val = v; best_idx = idx; }
        }
    } else {
        best_val = fabsf(bp_at(s_sample_index));   /* best available estimate   */
    }

    /* Monotonicity guard: the refinement may never place this beat at or
     * before the previous one, or the R-R series would go negative. */
    if (s_have_last_r && best_idx <= s_last_r_idx) {
        best_idx = mwi_idx;
    }

    s_last_qrs_idx = mwi_idx;
    s_sb_val       = 0.0f;                          /* searchback candidate consumed */
    record_beat(best_idx, best_val);
}

/* ===================================================================== */
/* Adaptive threshold                                                    */
/* ===================================================================== */

static inline void update_threshold(void)
{
    s_thr = s_npki + ECG_THR_SIGNAL_FRACTION * (s_spki - s_npki);
    if (s_thr < ECG_THR_ABS_FLOOR) { s_thr = ECG_THR_ABS_FLOOR; }
}

/**
 * Classify one local maximum of the integrated signal.
 *
 * A maximum above threshold and outside the refractory period is a QRS and
 * pulls SPKI up; a maximum below threshold is noise and pulls NPKI up. A
 * maximum above threshold but INSIDE the refractory window (almost always a
 * T-wave or the tail of the same complex) is discarded without updating
 * either estimate - feeding it to NPKI would inflate the noise floor and
 * feeding it to SPKI would inflate the signal floor.
 */
static void classify_peak(float32_t val, uint32_t idx)
{
    if (val >= s_thr) {
        if (!s_have_last_r || (idx - s_last_qrs_idx) >= REFRACTORY_SAMPLES) {
            s_spki = ECG_THR_PEAK_ALPHA * val + (1.0f - ECG_THR_PEAK_ALPHA) * s_spki;
            accept_qrs(idx);
        }
        /* else: inside refractory -> ignore entirely */
    } else {
        s_npki = ECG_THR_PEAK_ALPHA * val + (1.0f - ECG_THR_PEAK_ALPHA) * s_npki;
        /* Remember the best half-threshold candidate for searchback. */
        if (val > s_sb_val && val >= (0.5f * s_thr)) {
            s_sb_val = val;
            s_sb_idx = idx;
        }
    }
    update_threshold();
}

/**
 * Pan-Tompkins searchback. If we have gone 1.66 average R-R intervals without
 * a beat, the threshold has probably drifted above a genuine (but small) QRS -
 * typically after a burst of motion artefact raised SPKI. Re-examine the best
 * candidate that sat between half-threshold and threshold.
 */
static void searchback(void)
{
    uint32_t limit;

    if (!s_have_last_r || s_rr_mean_samples <= 0.0f || s_sb_val <= 0.0f) { return; }

    limit = (uint32_t)(ECG_THR_SEARCHBACK_FACTOR * s_rr_mean_samples);
    if ((s_sample_index - s_last_qrs_idx) < limit) { return; }
    if (s_sb_idx <= s_last_qrs_idx) { s_sb_val = 0.0f; return; }
    if ((s_sb_idx - s_last_qrs_idx) < REFRACTORY_SAMPLES) { s_sb_val = 0.0f; return; }

    s_spki = ECG_THR_SEARCHBACK_ALPHA * s_sb_val
           + (1.0f - ECG_THR_SEARCHBACK_ALPHA) * s_spki;
    update_threshold();
    accept_qrs(s_sb_idx);
}

/* ===================================================================== */
/* Per-block processing                                                  */
/* ===================================================================== */

static void process_block(const uint16_t *raw)
{
    uint32_t i;

    /* Convert to float and stream a decimated copy to the host. Working in raw
     * ADC counts (not volts) keeps one multiply out of the hot path; the
     * 5 Hz high-pass removes the 1.65 V bias anyway. */
    for (i = 0U; i < ECG_DMA_HALF_LEN; i++) {
        const uint16_t v = raw[i];
        s_blk_in[i] = (float32_t)v;

#if (ECG_STREAM_AVERAGE != 0U)
        s_dec_acc += v;
#else
        s_dec_acc  = v;
#endif
        if (++s_dec_count >= ECG_STREAM_DECIMATION) {
#if (ECG_STREAM_AVERAGE != 0U)
            stream_push((uint16_t)(s_dec_acc / ECG_STREAM_DECIMATION));
#else
            stream_push((uint16_t)s_dec_acc);
#endif
            s_dec_count = 0U;
            s_dec_acc   = 0U;
        }
    }

    /* Block-mode CMSIS-DSP: one call filters the whole half-buffer on the FPU,
     * which is several times cheaper than 32 single-sample calls. */
    arm_biquad_cascade_df1_f32(&s_bp_inst, s_blk_in, s_blk_bp, ECG_DMA_HALF_LEN);

    for (i = 0U; i < ECG_DMA_HALF_LEN; i++) {
        const float32_t x = s_blk_bp[i];
        float32_t d, sq, mwi;

        s_sample_index++;
        s_bp_hist[s_bp_hist_wr] = x;
        s_bp_hist_wr = (s_bp_hist_wr + 1U) & BP_HIST_MASK;

        /* 5-point central derivative, written causally. Its output belongs to
         * time n-2; that 2-sample group delay is accounted for by the fiducial
         * search window. The 1/8 scale keeps the squared value in range. */
        d = 0.125f * (x + 2.0f * s_d_z[0] - 2.0f * s_d_z[2] - s_d_z[3]);
        s_d_z[3] = s_d_z[2];
        s_d_z[2] = s_d_z[1];
        s_d_z[1] = s_d_z[0];
        s_d_z[0] = x;

        sq = d * d;

        /* Moving window integration as a running sum: O(1) per sample instead
         * of O(window). Periodically rebuilt from the buffer so float32
         * rounding cannot drift over a long recording. */
        s_mwi_sum -= s_mwi_buf[s_mwi_idx];
        s_mwi_buf[s_mwi_idx] = sq;
        s_mwi_sum += sq;
        if (++s_mwi_idx >= MWI_LEN) { s_mwi_idx = 0U; }

        if (--s_mwi_resync == 0U) {
            float32_t mean;
            arm_mean_f32(s_mwi_buf, MWI_LEN, &mean);
            s_mwi_sum     = mean * (float32_t)MWI_LEN;
            s_mwi_resync  = ECG_MWI_RESYNC_SAMPLES;
        }
        if (s_mwi_sum < 0.0f) { s_mwi_sum = 0.0f; }

        mwi = s_mwi_sum * (1.0f / (float32_t)MWI_LEN);

        if (s_learning) {
            /* Learning phase 1: no detection, just characterise the signal so
             * the very first threshold is in the right decade. */
            if (mwi > s_learn_max) { s_learn_max = mwi; }
            s_learn_sum += mwi;
            s_learn_n++;
            if (s_sample_index >= LEARN_SAMPLES) {
                s_spki     = s_learn_max;
                s_npki     = s_learn_sum / (float32_t)s_learn_n;
                s_learning = 0U;
                update_threshold();
            }
        } else {
            /* Local maximum test on the previous sample. The MWI output is
             * smooth by construction, so a simple three-point test finds one
             * candidate per hump without extra hysteresis. */
            if (s_mwi_prev1 > s_mwi_prev2 && s_mwi_prev1 >= mwi) {
                classify_peak(s_mwi_prev1, s_sample_index - 1U);
            }
            searchback();
        }

        s_mwi_prev2 = s_mwi_prev1;
        s_mwi_prev1 = mwi;
    }
}

/* ===================================================================== */
/* Public API                                                            */
/* ===================================================================== */

void ecg_init(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim)
{
    s_hadc = hadc;
    s_htim = htim;

    memset(s_dma_buf,  0, sizeof(s_dma_buf));
    memset(s_mwi_buf,  0, sizeof(s_mwi_buf));
    memset(s_bp_hist,  0, sizeof(s_bp_hist));
    memset(s_bp_state, 0, sizeof(s_bp_state));
    memset(s_d_z,      0, sizeof(s_d_z));
    memset(s_beats,    0, sizeof(s_beats));

    s_block_ready     = 0U;
    s_block_overruns  = 0U;
    s_mwi_idx         = 0U;
    s_mwi_sum         = 0.0f;
    s_mwi_resync      = ECG_MWI_RESYNC_SAMPLES;
    s_bp_hist_wr      = 0U;
    s_sample_index    = 0U;
    s_mwi_prev1       = 0.0f;
    s_mwi_prev2       = 0.0f;
    s_spki            = 0.0f;
    s_npki            = 0.0f;
    s_thr             = ECG_THR_ABS_FLOOR;
    s_last_qrs_idx    = 0U;
    s_last_r_idx      = 0U;
    s_have_last_r     = 0U;
    s_rr_mean_samples = 0.0f;
    s_sb_val          = 0.0f;
    s_sb_idx          = 0U;
    s_learning        = 1U;
    s_learn_max       = 0.0f;
    s_learn_sum       = 0.0f;
    s_learn_n         = 0U;
    s_beat_count      = 0U;
    s_last_beat_ms    = 0U;
    s_led_on          = 0U;
    s_stream_wr       = 0U;
    s_stream_rd       = 0U;
    s_stream_drops    = 0U;
    s_dec_count       = 0U;
    s_dec_acc         = 0U;

    /* High-pass first: it strips the DC bias and baseline wander before the
     * low-pass stage, which keeps the second biquad's state well scaled. */
    design_highpass(ECG_BP_HIGHPASS_HZ, (float32_t)ECG_SAMPLE_RATE_HZ,
                    ECG_BP_Q, &s_bp_coeffs[0]);
    design_lowpass (ECG_BP_LOWPASS_HZ,  (float32_t)ECG_SAMPLE_RATE_HZ,
                    ECG_BP_Q, &s_bp_coeffs[5]);
    arm_biquad_cascade_df1_init_f32(&s_bp_inst, 2U, s_bp_coeffs, s_bp_state);

    HAL_GPIO_WritePin(CFG_LED_PORT, CFG_LED_PIN,
                      (CFG_LED_ON_STATE == GPIO_PIN_RESET) ? GPIO_PIN_SET
                                                           : GPIO_PIN_RESET);

    /* DMA must be armed before the trigger source starts running. */
    (void)HAL_ADC_Start_DMA(s_hadc, (uint32_t *)s_dma_buf, ECG_DMA_BUF_LEN);
    (void)HAL_TIM_Base_Start(s_htim);
}

void ecg_on_dma_half(void)
{
    if (s_block_ready & 0x01U) { s_block_overruns++; }
    s_block_ready |= 0x01U;
}

void ecg_on_dma_full(void)
{
    if (s_block_ready & 0x02U) { s_block_overruns++; }
    s_block_ready |= 0x02U;
}

void ecg_process(void)
{
    uint8_t  mask;
    uint32_t primask;

    led_service();

    primask = __get_PRIMASK();
    __disable_irq();
    mask = s_block_ready;
    s_block_ready = 0U;
    __set_PRIMASK(primask);

    /* Lower half always precedes the upper half in a circular transfer, so
     * handling them in this order is correct even when we were late and both
     * flags are set (which is counted as an overrun above). */
    if (mask & 0x01U) { process_block(&s_dma_buf[0]); }
    if (mask & 0x02U) { process_block(&s_dma_buf[ECG_DMA_HALF_LEN]); }
}

uint32_t ecg_now_ms(void)
{
    return SAMPLES_TO_MS(s_sample_index);
}

uint32_t ecg_get_beat_count(void)
{
    return s_beat_count;
}

float32_t ecg_get_heart_rate(void)
{
    float32_t tmp[ECG_HR_MEDIAN_N];
    uint32_t  n = 0U;
    uint32_t  i;

    if (s_beat_count == 0U) { return 0.0f; }
    if ((ecg_now_ms() - s_last_beat_ms) > ECG_HR_TIMEOUT_MS) { return 0.0f; }

    /* Median rather than mean: one missed or doubled beat would drag a mean
     * by tens of BPM, but moves a median by at most one rank. */
    for (i = 1U; i <= s_beat_count && n < ECG_HR_MEDIAN_N; i++) {
        const ecg_beat_t *b = &s_beats[(s_beat_count - i) & BEAT_MASK];
        if (i > ECG_BEAT_BUF_LEN) { break; }
        if (b->valid) { tmp[n++] = b->rr_ms; }
    }
    if (n < 3U) { return 0.0f; }

    for (i = 1U; i < n; i++) {                      /* insertion sort, n <= 7  */
        const float32_t key = tmp[i];
        int32_t j = (int32_t)i - 1;
        while (j >= 0 && tmp[j] > key) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = key;
    }

    return 60000.0f / tmp[n / 2U];
}

float32_t ecg_get_rmssd(void)
{
    float32_t sum_sq = 0.0f;
    float32_t prev   = 0.0f;
    uint8_t   have_prev = 0U;
    uint32_t  pairs  = 0U;
    uint32_t  i;
    uint32_t  window = (s_beat_count < ECG_RMSSD_WINDOW) ? s_beat_count
                                                         : ECG_RMSSD_WINDOW;

    /* RMSSD is the root mean square of SUCCESSIVE differences, so a rejected
     * beat breaks the chain - we must not difference across the gap. */
    for (i = 0U; i < window; i++) {
        const ecg_beat_t *b = &s_beats[(s_beat_count - window + i) & BEAT_MASK];
        if (!b->valid) { have_prev = 0U; continue; }
        if (have_prev) {
            const float32_t d = b->rr_ms - prev;
            sum_sq += d * d;
            pairs++;
        }
        prev = b->rr_ms;
        have_prev = 1U;
    }

    if (pairs < ECG_RMSSD_MIN_PAIRS) { return -1.0f; }
    return sqrtf(sum_sq / (float32_t)pairs);
}

ecg_quality_t ecg_get_quality(void)
{
    uint32_t bad = 0U;
    uint32_t n;
    uint32_t i;

    if (HAL_GPIO_ReadPin(CFG_LEADOFF_P_PORT, CFG_LEADOFF_P_PIN) == CFG_LEADOFF_ACTIVE_STATE ||
        HAL_GPIO_ReadPin(CFG_LEADOFF_N_PORT, CFG_LEADOFF_N_PIN) == CFG_LEADOFF_ACTIVE_STATE) {
        return ECG_QUALITY_LEADS_OFF;
    }

    if (s_beat_count == 0U ||
        (ecg_now_ms() - s_last_beat_ms) > ECG_QUALITY_BEAT_TIMEOUT_MS) {
        return ECG_QUALITY_NOISY;
    }

    /* Electrodes are attached and beats are arriving, but if many of the
     * recent intervals are physiologically impossible we are detecting motion
     * artefact rather than heartbeats. */
    n = (s_beat_count < ECG_QUALITY_WINDOW) ? s_beat_count : ECG_QUALITY_WINDOW;
    for (i = 0U; i < n; i++) {
        if (!s_beats[(s_beat_count - 1U - i) & BEAT_MASK].valid) { bad++; }
    }
    if (n >= 4U && bad > ECG_QUALITY_MAX_BAD) { return ECG_QUALITY_NOISY; }

    return ECG_QUALITY_GOOD;
}

uint32_t ecg_copy_beats(ecg_beat_t *dst, uint32_t max)
{
    uint32_t n = s_beat_count;
    uint32_t start;
    uint32_t i;

    if (n > ECG_BEAT_BUF_LEN) { n = ECG_BEAT_BUF_LEN; }
    if (n > max)              { n = max; }

    start = s_beat_count - n;
    for (i = 0U; i < n; i++) {
        dst[i] = s_beats[(start + i) & BEAT_MASK];
    }
    return n;
}

int ecg_stream_pop(uint16_t *sample)
{
    if (s_stream_rd == s_stream_wr) { return 0; }
    *sample = s_stream_fifo[s_stream_rd];
    s_stream_rd = (s_stream_rd + 1U) & STREAM_MASK;
    return 1;
}

uint32_t ecg_get_block_overruns(void) { return s_block_overruns; }
uint32_t ecg_get_stream_drops(void)   { return s_stream_drops;   }

/* ===================================================================== */
/* HAL DMA callbacks                                                     */
/* ===================================================================== */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == s_hadc) { ecg_on_dma_half(); }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == s_hadc) { ecg_on_dma_full(); }
}
