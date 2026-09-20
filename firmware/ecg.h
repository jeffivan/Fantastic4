/**
 * @file    ecg.h
 * @brief   ECG acquisition (TIM2 -> ADC1 -> DMA ping-pong) and Pan-Tompkins
 *          R-peak detection.
 *
 * Threading model: ecg_on_dma_half()/ecg_on_dma_full() run in the DMA ISR and
 * do nothing but set a flag. Everything else runs in main context from
 * ecg_process(), so no locking is needed between this module and the other
 * modules - they are all cooperative main-loop tasks.
 */

#ifndef ECG_H
#define ECG_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ECG_QUALITY_GOOD = 0,
    ECG_QUALITY_NOISY,
    ECG_QUALITY_LEADS_OFF
} ecg_quality_t;

/**
 * One detected heartbeat. R-R interval and R-wave amplitude live in the same
 * record rather than in two parallel circular buffers: they are always written
 * and read together, and one array halves the index bookkeeping.
 */
typedef struct {
    uint32_t  t_ms;     /**< time of the R fiducial, ms in the ADC timebase   */
    float32_t rr_ms;    /**< interval since the previous R peak, ms           */
    float32_t amp;      /**< R-wave amplitude, band-passed ADC counts         */
    uint8_t   valid;    /**< 1 when rr_ms passed the 300-2000 ms gate         */
} ecg_beat_t;

/** Configure the filters and start TIM2 -> ADC1 -> DMA. */
void ecg_init(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim);

/** Main-loop task. Processes any DMA half-block that is ready, services the
 *  beat LED, and returns immediately when there is nothing to do. */
void ecg_process(void);

/* --- DMA ISR hooks (call from HAL_ADC_Conv{Half,}CpltCallback) --- */
void ecg_on_dma_half(void);
void ecg_on_dma_full(void);

/* --- Results ---------------------------------------------------------- */

/** Median-filtered heart rate in BPM, or 0.0f when unknown/stale. */
float32_t     ecg_get_heart_rate(void);

/** RMSSD over the recent beat window in ms, or -1.0f when not yet estimable. */
float32_t     ecg_get_rmssd(void);

ecg_quality_t ecg_get_quality(void);

/** Milliseconds since ecg_init(), derived from the ADC sample counter. This is
 *  the timebase all beat timestamps use - see ecg.c for why it is not SysTick. */
uint32_t      ecg_now_ms(void);

/** Copy up to @p max of the most recent beats, oldest first. Returns the count. */
uint32_t      ecg_copy_beats(ecg_beat_t *dst, uint32_t max);

uint32_t      ecg_get_beat_count(void);

/** Pop one decimated raw sample for the host plot. Returns 1 on success. */
int           ecg_stream_pop(uint16_t *sample);

/* Diagnostics */
uint32_t      ecg_get_block_overruns(void);
uint32_t      ecg_get_stream_drops(void);

#ifdef __cplusplus
}
#endif

#endif /* ECG_H */
