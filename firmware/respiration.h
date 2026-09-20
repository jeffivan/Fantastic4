/**
 * @file    respiration.h
 * @brief   Respiration rate inferred from respiratory sinus arrhythmia in the
 *          R-R interval series. No respiration sensor is present.
 */

#ifndef RESPIRATION_H
#define RESPIRATION_H

#include <stdint.h>
#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float32_t rate_brpm;    /**< breaths per minute; meaningless unless valid  */
    float32_t confidence;   /**< 0..1, from spectral peak prominence           */
    float32_t peak_ratio;   /**< peak magnitude / mean magnitude in the band   */
    float32_t peak_hz;      /**< refined peak frequency, Hz                    */
    uint16_t  beats_used;   /**< valid beats that contributed to the window    */
    uint8_t   valid;        /**< 1 when the estimate may be used               */
} respiration_result_t;

void respiration_init(void);

/**
 * Rebuild the tachogram and re-run the spectral estimate.
 * @param now_ms  current time in the ECG timebase (ecg_now_ms()).
 * @return 1 if a new valid estimate was produced, 0 otherwise.
 *
 * Cost is roughly 60 us on an 84 MHz M4F; call it once every
 * RESP_UPDATE_PERIOD_MS, never per sample.
 */
uint8_t respiration_update(uint32_t now_ms);

const respiration_result_t *respiration_get(void);

#ifdef __cplusplus
}
#endif

#endif /* RESPIRATION_H */
