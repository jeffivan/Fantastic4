/**
 * @file    fuzzy.h
 * @brief   Mamdani fuzzy inference over HR / RMSSD / respiration / temperature.
 */

#ifndef FUZZY_H
#define FUZZY_H

#include <stdint.h>
#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FUZZY_STATE_UNKNOWN = 0,
    FUZZY_STATE_NORMAL,
    FUZZY_STATE_STRESSED,
    FUZZY_STATE_FEVERISH,
    FUZZY_STATE_CONCERNING
} fuzzy_state_t;

/**
 * Each input carries its own validity flag. A flag of 0 means "not measured";
 * the engine skips every rule that mentions that input. It never substitutes a
 * default, an average or a last-known value - a fabricated input would
 * propagate into the risk score indistinguishably from a real one.
 */
typedef struct {
    float32_t hr_bpm;     uint8_t hr_valid;
    float32_t rmssd_ms;   uint8_t rmssd_valid;
    float32_t resp_brpm;  uint8_t resp_valid;
    float32_t temp_c;     uint8_t temp_valid;
} fuzzy_inputs_t;

typedef struct {
    float32_t     risk;         /**< 0..100, centroid of the aggregated output */
    fuzzy_state_t state;
    float32_t     confidence;   /**< 0..1                                      */
    float32_t     mu_temp_high; /**< membership used for the FEVERISH decision */
    uint8_t       inputs_used;  /**< 0..4                                      */
    uint8_t       rules_fired;
} fuzzy_result_t;

void        fuzzy_init(void);
void        fuzzy_evaluate(const fuzzy_inputs_t *in, fuzzy_result_t *out);
const char *fuzzy_state_name(fuzzy_state_t s);

#ifdef __cplusplus
}
#endif

#endif /* FUZZY_H */
