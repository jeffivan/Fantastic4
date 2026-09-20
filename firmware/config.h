/**
 * @file    config.h
 * @brief   Single source of truth for every tunable constant in the firmware.
 *
 * EVERYTHING in this file is expected to be overwritten by the offline Python
 * tuning script. Nothing here may depend on a HAL type, so the file can be
 * parsed/regenerated mechanically. GPIO port/pin tokens appear as bare macros;
 * they are only expanded at the use site, where the HAL headers are in scope.
 *
 * Each constant carries its UNITS and the reason it exists. If you change a
 * value and cannot explain which stage of the pipeline it affects, don't.
 */

#ifndef CONFIG_H
#define CONFIG_H

/* ===================================================================== */
/* 1. CLOCK                                                              */
/* ===================================================================== */

/* WeAct Black Pill v3.x ships a 25 MHz HSE crystal.
 * PLL: 25 MHz / PLLM = 1 MHz VCO input, x PLLN = 336 MHz VCO, / PLLP = 84 MHz.
 * For an 8 MHz HSE board set CFG_PLL_M to 8. To run from the 16 MHz HSI set
 * CFG_PLL_M to 16 and switch the oscillator selection in SystemClock_Config(). */
#define CFG_HSE_FREQ_HZ             25000000UL  /* Hz  - crystal on the board  */
#define CFG_PLL_M                   25U         /* -   - VCO input = HSE/M = 1 MHz */
#define CFG_PLL_N                   336U        /* -   - VCO output = 336 MHz  */
#define CFG_PLL_P_DIV               4U          /* -   - SYSCLK = 336/4 = 84 MHz */
#define CFG_PLL_Q                   7U          /* -   - 48 MHz branch (unused) */
#define CFG_SYSCLK_HZ               84000000UL  /* Hz  - resulting core clock  */

/* ===================================================================== */
/* 2. PIN MAP (fixed by the hardware, do not change)                     */
/* ===================================================================== */

#define CFG_ECG_ADC_CHANNEL         ADC_CHANNEL_4  /* PA4  - AD8232 OUTPUT     */

#define CFG_LEADOFF_P_PORT          GPIOB          /* PB0  - AD8232 LO+        */
#define CFG_LEADOFF_P_PIN           GPIO_PIN_0
#define CFG_LEADOFF_N_PORT          GPIOB          /* PB1  - AD8232 LO-        */
#define CFG_LEADOFF_N_PIN           GPIO_PIN_1
/* AD8232 drives LO+/LO- HIGH when that electrode has lost skin contact. */
#define CFG_LEADOFF_ACTIVE_STATE    GPIO_PIN_SET

#define CFG_OW_PORT                 GPIOA          /* PA1  - DS18B20 DQ        */
#define CFG_OW_PIN                  GPIO_PIN_1

#define CFG_LED_PORT                GPIOC          /* PC13 - onboard LED       */
#define CFG_LED_PIN                 GPIO_PIN_13
#define CFG_LED_ON_STATE            GPIO_PIN_RESET /* active LOW               */

/* ===================================================================== */
/* 3. ECG ACQUISITION                                                    */
/* ===================================================================== */

/* 250 Hz is the classic Pan-Tompkins rate: it is >5x the 40 Hz analog corner
 * of the AD8232 heart-rate configuration, and 1000 % 250 == 0 so the
 * sample-index -> millisecond conversion is an exact integer multiply. */
#define ECG_SAMPLE_RATE_HZ          250U        /* Hz                          */

/* DMA ping-pong buffer. Half-block latency = HALF / Fs.
 * 64 total / 32 per half = 128 ms of latency per processing block, which is
 * invisible next to the 1 Hz telemetry cadence but gives the CPU large,
 * cache-friendly blocks for the CMSIS-DSP vector calls. */
#define ECG_DMA_BUF_LEN             64U         /* samples (total, circular)   */
#define ECG_DMA_HALF_LEN            (ECG_DMA_BUF_LEN / 2U)

/* ---- Pan-Tompkins band-pass -----------------------------------------
 * The QRS complex has most of its energy between 5 and 15 Hz. The high-pass
 * kills baseline wander (respiration, electrode drift, motion); the low-pass
 * kills T-waves, EMG and the 50/60 Hz mains residue. Implemented as two
 * RBJ-cookbook biquads (Butterworth Q) whose coefficients are computed at
 * runtime in ecg_init(), so these stay human-readable frequencies.          */
#define ECG_BP_HIGHPASS_HZ          5.0f        /* Hz  - -3 dB corner          */
#define ECG_BP_LOWPASS_HZ           15.0f       /* Hz  - -3 dB corner          */
#define ECG_BP_Q                    0.70710678f /* -   - Butterworth (maximally flat) */

/* Moving-window integrator. The window must be about as wide as the widest
 * expected QRS (~100-150 ms) so it produces ONE hump per complex; too wide and
 * QRS merges with the T-wave, too narrow and you get multiple humps per beat. */
#define ECG_MWI_WINDOW_MS           150U        /* ms                          */

/* ---- Adaptive threshold (Pan-Tompkins) ------------------------------ */
#define ECG_THR_SIGNAL_FRACTION     0.25f       /* -   - THRESHOLD = NPKI + k*(SPKI-NPKI) */
#define ECG_THR_PEAK_ALPHA          0.125f      /* -   - IIR rate for SPKI/NPKI updates */
#define ECG_THR_SEARCHBACK_ALPHA    0.25f       /* -   - faster SPKI update on a searchback find */
#define ECG_THR_SEARCHBACK_FACTOR   1.66f       /* -   - re-search after 1.66 * mean R-R */
#define ECG_THR_ABS_FLOOR           50.0f       /* MWI units - stops the detector chasing
                                                 *   quantisation noise when the leads are off */
#define ECG_LEARN_MS                2000U       /* ms  - Pan-Tompkins "learning phase 1" */

/* Physiological gates. 200 ms is the absolute cardiac refractory period: no
 * two genuine R-waves can be closer, so anything inside it is a T-wave or an
 * artefact. The 300-2000 ms R-R gate is 30-200 BPM. */
#define ECG_REFRACTORY_MS           200U        /* ms                          */
#define ECG_RR_MIN_MS               300.0f      /* ms  - 200 BPM               */
#define ECG_RR_MAX_MS               2000.0f     /* ms  - 30 BPM                */

/* The MWI output lags the true R peak by (derivative delay + half the MWI
 * window). We search backwards over this many samples in the BAND-PASSED
 * signal for the real fiducial point, which is what makes the R-R series
 * accurate enough to see respiratory sinus arrhythmia at all. */
#define ECG_FIDUCIAL_SEARCH_EXTRA   10U         /* samples added to the MWI width */

#define ECG_BEAT_BUF_LEN            256U        /* beats - must be a power of two */
#define ECG_HR_MEDIAN_N             7U          /* beats - median filter length for BPM */
#define ECG_HR_TIMEOUT_MS           5000U       /* ms  - no beat for this long => HR unknown */
#define ECG_RMSSD_WINDOW            32U         /* beats used for the RMSSD estimate */
#define ECG_RMSSD_MIN_PAIRS         10U         /* minimum successive-difference pairs */

/* Signal quality */
#define ECG_QUALITY_BEAT_TIMEOUT_MS 3000U       /* ms  - silence => NOISY      */
#define ECG_QUALITY_WINDOW          16U         /* beats inspected for plausibility */
#define ECG_QUALITY_MAX_BAD         4U          /* rejected intervals allowed in that window */

#define ECG_LED_PULSE_MS            25U         /* ms  - visible beat flash    */

/* Running-sum drift guard: recompute the integrator sum from scratch this
 * often so float32 rounding cannot accumulate over a long session. */
#define ECG_MWI_RESYNC_SAMPLES      15000U      /* samples (= 60 s at 250 Hz)  */

/* ===================================================================== */
/* 4. RESPIRATION (RSA extraction from the R-R series)                   */
/* ===================================================================== */

/* 4 Hz is far above the highest breathing rate we care about (0.4 Hz) yet low
 * enough that a 128-point FFT covers a full 32 s window - long enough to
 * resolve a 9 breaths/min rhythm (needs ~7 s per cycle, so ~4.5 cycles). */
#define RESP_RESAMPLE_HZ            4.0f        /* Hz  - uniform tachogram grid */
#define RESP_FFT_LEN                128U        /* samples - MUST be a CMSIS rfft size */
#define RESP_WINDOW_MS              32000U      /* ms  = RESP_FFT_LEN / RESP_RESAMPLE_HZ */

/* Respiratory band. 0.15-0.40 Hz == 9-24 breaths/min. Anything below is the
 * Mayer wave / baroreflex band (~0.1 Hz) and would be mistaken for breathing. */
#define RESP_BAND_LOW_HZ            0.15f       /* Hz  = 9  breaths/min        */
#define RESP_BAND_HIGH_HZ           0.40f       /* Hz  = 24 breaths/min        */

/* Bin width is RESP_RESAMPLE_HZ / RESP_FFT_LEN = 0.03125 Hz = 1.875 brpm, so
 * the band holds only 8 bins. Parabolic interpolation over the 3 bins around
 * the maximum is what buys back sub-bin (~0.2 brpm) resolution. */

#define RESP_UPDATE_PERIOD_MS       5000U       /* ms  - breathing changes slowly */
#define RESP_MIN_BEATS              20U         /* beats required inside the window */
#define RESP_MAX_BEATS              160U        /* beats - snapshot capacity (200 BPM * 32 s) */
#define RESP_EDGE_TOLERANCE_MS      2500U       /* ms  - allowed gap at either window edge */

/* Confidence = (peak / band mean). A flat spectrum gives 1.0; a single clean
 * respiratory peak in an 8-bin band can reach ~6. Below MIN the estimate is
 * reported invalid rather than guessed. */
#define RESP_CONF_RATIO_MIN         2.0f        /* -   - reject below this     */
#define RESP_CONF_RATIO_FULL        5.0f        /* -   - confidence saturates at 1.0 here */
#define RESP_CONF_ACCEPT            0.25f       /* -   - fuzzy engine ignores resp below this */

/* ===================================================================== */
/* 5. DS18B20 / 1-WIRE                                                   */
/* ===================================================================== */

#define DS18B20_SAMPLE_PERIOD_MS    2000U       /* ms  - body temp is slow     */
#define DS18B20_CONV_TIME_MS        780U        /* ms  - 750 ms spec + margin (12-bit) */
#define DS18B20_RETRY_MS            1000U       /* ms  - backoff after a failed transaction */
#define DS18B20_STALE_MS            10000U      /* ms  - last good reading expires */
#define DS18B20_MIN_C               20.0f       /* C   - plausibility gate, low  */
#define DS18B20_MAX_C               45.0f       /* C   - plausibility gate, high */

/* 1-Wire bit timings, microseconds. Tuned for the STM32 INTERNAL pull-up
 * (~40 kOhm), which is ~4x weaker than the usual 4k7. With ~30 pF of bus
 * capacitance the rise time is ~3.6 us, so the master low pulse in a read slot
 * is kept short and the sample point is pushed late - but still inside the
 * 15 us that the DS18B20 guarantees. Verify on a scope if you change the
 * cable; keep it under ~20 cm. */
#define OW_RESET_LOW_US             480U        /* us  - master reset pulse    */
#define OW_RESET_PRESENCE_WAIT_US   70U         /* us  - sample point for the presence pulse */
#define OW_RESET_RECOVERY_US        410U        /* us  - remainder of the 960 us slot */
#define OW_WRITE1_LOW_US            6U          /* us                          */
#define OW_WRITE1_HIGH_US           64U         /* us                          */
#define OW_WRITE0_LOW_US            60U         /* us                          */
#define OW_WRITE0_HIGH_US           10U         /* us                          */
#define OW_READ_LOW_US              3U          /* us  - short, the weak pull-up needs the time */
#define OW_READ_SAMPLE_US           10U         /* us  - after release => 13 us into the slot */
#define OW_READ_RECOVERY_US         55U         /* us                          */

/* DS18B20 ROM / function commands */
#define DS18B20_CMD_SKIP_ROM        0xCCU
#define DS18B20_CMD_CONVERT_T       0x44U
#define DS18B20_CMD_READ_SCRATCH    0xBEU
#define DS18B20_SCRATCHPAD_LEN      9U

/* ===================================================================== */
/* 6. FUZZY CLASSIFIER                                                   */
/* ===================================================================== */

/* Each input has three membership functions given as (a, b, c) triples.
 * LOW is LEFT-shouldered  : mu = 1 for x <= b, ramps to 0 at c.
 * NORMAL is a true triangle: 0 at a, 1 at b, 0 at c.
 * HIGH is RIGHT-shouldered: 0 at a, ramps to 1 at b, stays 1 above.
 * Shouldering the outer sets guarantees that an extreme value always has
 * membership somewhere - a plain triangle would return all-zeros at, say,
 * 200 BPM and the engine would silently stop firing. */

/* -- Heart rate, BPM -- */
#define FZ_HR_LOW_A                 30.0f
#define FZ_HR_LOW_B                 45.0f
#define FZ_HR_LOW_C                 60.0f
#define FZ_HR_NRM_A                 50.0f
#define FZ_HR_NRM_B                 72.0f
#define FZ_HR_NRM_C                 95.0f
#define FZ_HR_HIGH_A                85.0f
#define FZ_HR_HIGH_B                105.0f
#define FZ_HR_HIGH_C                180.0f

/* -- RMSSD (short-term HRV), ms. Low RMSSD == sympathetic dominance. -- */
#define FZ_RMSSD_LOW_A              0.0f
#define FZ_RMSSD_LOW_B              15.0f
#define FZ_RMSSD_LOW_C              30.0f
#define FZ_RMSSD_NRM_A              20.0f
#define FZ_RMSSD_NRM_B              40.0f
#define FZ_RMSSD_NRM_C              70.0f
#define FZ_RMSSD_HIGH_A             55.0f
#define FZ_RMSSD_HIGH_B             90.0f
#define FZ_RMSSD_HIGH_C             200.0f

/* -- Respiration, breaths/min -- */
#define FZ_RESP_LOW_A               5.0f
#define FZ_RESP_LOW_B               9.0f
#define FZ_RESP_LOW_C               12.0f
#define FZ_RESP_NRM_A               10.0f
#define FZ_RESP_NRM_B               15.0f
#define FZ_RESP_NRM_C               20.0f
#define FZ_RESP_HIGH_A              18.0f
#define FZ_RESP_HIGH_B              24.0f
#define FZ_RESP_HIGH_C              40.0f

/* -- Temperature, degrees C (AFTER FZ_TEMP_OFFSET_C is applied) -- */
#define FZ_TEMP_LOW_A               30.0f
#define FZ_TEMP_LOW_B               35.5f
#define FZ_TEMP_LOW_C               36.3f
#define FZ_TEMP_NRM_A               36.0f
#define FZ_TEMP_NRM_B               36.8f
#define FZ_TEMP_NRM_C               37.4f
#define FZ_TEMP_HIGH_A              37.2f
#define FZ_TEMP_HIGH_B              38.5f
#define FZ_TEMP_HIGH_C              43.0f

/* A DS18B20 taped to skin or held in the armpit reads 1-2 C BELOW core. The
 * membership functions above are written in CORE temperature, so calibrate
 * the sensor to core here rather than distorting the fuzzy sets. */
#define FZ_TEMP_OFFSET_C            0.0f        /* C   - added to the raw sensor reading */

/* -- Output (risk) singleton sets on a 0..100 universe -- */
#define FZ_OUT_LOW_A                0.0f
#define FZ_OUT_LOW_B                10.0f
#define FZ_OUT_LOW_C                35.0f
#define FZ_OUT_MOD_A                20.0f
#define FZ_OUT_MOD_B                40.0f
#define FZ_OUT_MOD_C                60.0f
#define FZ_OUT_ELEV_A               45.0f
#define FZ_OUT_ELEV_B               65.0f
#define FZ_OUT_ELEV_C               85.0f
#define FZ_OUT_HIGH_A               70.0f
#define FZ_OUT_HIGH_B               90.0f
#define FZ_OUT_HIGH_C               100.0f

/* Centroid defuzzification grid: 0..100 in steps of 2 => 51 samples. Fine
 * enough that the centroid error is well under 1 risk point, cheap enough to
 * run every second without noticing it. */
#define FZ_OUT_STEPS                51U
#define FZ_OUT_STEP_SIZE            2.0f

/* -- Rule weights (see the rule table in fuzzy.c) -- */
#define FZ_W_R1_BASELINE_OK         1.0f
#define FZ_W_R2_ATHLETIC_REST       0.9f
#define FZ_W_R3_HR_RESP_OK          0.7f
#define FZ_W_R4_TACHY_LOW_HRV       1.0f
#define FZ_W_R5_TACHYPNEA_LOW_HRV   1.0f
#define FZ_W_R6_TACHY_TACHYPNEA     0.9f
#define FZ_W_R7_FEVER_TACHY         1.0f
#define FZ_W_R8_SIRS_TRIAD          1.0f
#define FZ_W_R9_FEVER_ALONE         0.8f
#define FZ_W_R10_HYPOTHERMIA        0.5f
#define FZ_W_R11_LOW_HRV_ALONE      0.7f
#define FZ_W_R12_BRADY_BRADYPNEA    0.5f
#define FZ_W_R13_TACHY_FALLBACK     0.4f
#define FZ_W_R14_NORMAL_FALLBACK    0.4f

/* -- Risk score -> state label -- */
#define FZ_STATE_STRESSED_MIN       32.0f       /* risk points                 */
#define FZ_STATE_FEVERISH_MIN       35.0f       /* risk points                 */
#define FZ_STATE_CONCERNING_MIN     68.0f       /* risk points                 */
#define FZ_FEVER_TEMP_MU_MIN        0.50f       /* -   - mu(temp IS HIGH) needed for FEVERISH */

/* Confidence model: how much of the answer rests on real measurements.
 * conf = (inputs_available / 4) * (FLOOR + (1 - FLOOR) * strongest_rule) */
#define FZ_CONF_ACTIVATION_FLOOR    0.40f       /* -                           */

/* ===================================================================== */
/* 7. TELEMETRY / UART                                                   */
/* ===================================================================== */

#define TELEMETRY_PERIOD_MS         1000U       /* ms  - one summary line/second */
#define UART_BAUDRATE               115200U     /* bps - 8N1                   */
#define UART_TX_RING_SIZE           1024U       /* bytes - MUST be a power of two */

/* Reserve space so a burst of ECG samples can never starve the once-per-second
 * summary line. ECG lines are only queued when at least this much room is free. */
#define UART_TX_RESERVE_BYTES       160U        /* bytes                       */

/* ---- Raw ECG streaming decimation -----------------------------------
 * Link budget at 115200 8N1 (10 bits/byte incl. start+stop) = 11520 byte/s.
 *   "E:1234\r\n"  = 8 bytes worst case
 *   summary line  = ~80 bytes, once per second = 80 byte/s (0.7 %)
 * N = 1 -> 250 Hz * 8 B = 2000 B/s = 18 % of the link
 * N = 2 -> 125 Hz * 8 B = 1000 B/s =  9 % of the link   <-- chosen
 * N = 4 ->  62 Hz * 8 B =  500 B/s =  4 % but starts to visibly square off
 *          the QRS upstroke on the plot.
 * N = 2 wins because the AD8232 heart-rate configuration band-limits the
 * analog signal to ~40 Hz, so 125 Hz is still >3x oversampled and nothing
 * aliases; it halves the host's parse load and leaves ~90 % of the link free
 * for burst jitter and ad-hoc debug output. */
#define ECG_STREAM_DECIMATION       2U          /* keep 1 sample in N          */
#define ECG_STREAM_FIFO_LEN         64U         /* samples - MUST be a power of two */
#define ECG_STREAM_MAX_PER_LOOP     16U         /* samples drained per main-loop pass */

/* Decimating by simple subsampling would fold anything above Fs/(2N) = 62.5 Hz
 * back into the plot. A 2-tap boxcar average before dropping a sample is a
 * (gentle) anti-alias filter with a null exactly at 125 Hz. */
#define ECG_STREAM_AVERAGE          1U          /* 1 = average the N samples, 0 = subsample */

#endif /* CONFIG_H */
