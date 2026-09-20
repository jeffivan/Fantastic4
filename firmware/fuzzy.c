/**
 * @file    fuzzy.c
 * @brief   Hand-written Mamdani fuzzy inference engine.
 *
 * WHY FUZZY AND NOT THRESHOLDS
 *   Every physiological boundary here is genuinely fuzzy: 37.4 C is not
 *   healthy and 37.5 C feverish. A threshold ladder produces a state that
 *   flickers whenever a measurement sits on a boundary, which on a 1 Hz
 *   display looks broken. Graded memberships plus centroid defuzzification
 *   give a score that moves continuously with the inputs, so the state only
 *   changes when the evidence actually changes.
 *
 * PIPELINE
 *   fuzzify  : each input -> three memberships (LOW / NORMAL / HIGH)
 *   inference: min for AND within a rule, scaled by the rule weight
 *   aggregate: max across rules, over a discretised 0..100 output universe
 *   defuzzify: centroid (centre of area)
 *
 * GRACEFUL DEGRADATION
 *   Rules are skipped, not defaulted, when an antecedent is unmeasured. The
 *   rule base deliberately contains single-antecedent fallback rules so that
 *   even one surviving input still produces a defensible score. Confidence
 *   then reports how much of the answer rests on real data.
 */

#include "fuzzy.h"
#include "config.h"
#include <string.h>

/* ===================================================================== */
/* Types                                                                 */
/* ===================================================================== */

typedef enum { IN_HR = 0, IN_RMSSD, IN_RESP, IN_TEMP, N_IN } fz_input_t;
typedef enum { MF_LOW = 0, MF_NORM, MF_HIGH, N_MF } fz_mf_t;
typedef enum { OUT_LOW = 0, OUT_MOD, OUT_ELEV, OUT_HIGH, N_OUT } fz_out_t;

typedef enum { SH_NONE = 0, SH_LEFT, SH_RIGHT } fz_shoulder_t;

typedef struct {
    float32_t a, b, c;
    uint8_t   shoulder;
} fz_mf_def_t;

#define MAX_ANT 3
typedef struct {
    uint8_t   n_ant;
    uint8_t   var[MAX_ANT];
    uint8_t   mf[MAX_ANT];
    uint8_t   out;
    float32_t w;
    const char *why;        /* kept for readability; the compiler folds it into
                             * .rodata and it costs nothing at runtime         */
} fz_rule_t;

/* ===================================================================== */
/* Membership functions (breakpoints from config.h)                      */
/* ===================================================================== */

static const fz_mf_def_t s_in_mf[N_IN][N_MF] = {
    /* IN_HR */ {
        { FZ_HR_LOW_A,     FZ_HR_LOW_B,     FZ_HR_LOW_C,     SH_LEFT  },
        { FZ_HR_NRM_A,     FZ_HR_NRM_B,     FZ_HR_NRM_C,     SH_NONE  },
        { FZ_HR_HIGH_A,    FZ_HR_HIGH_B,    FZ_HR_HIGH_C,    SH_RIGHT },
    },
    /* IN_RMSSD */ {
        { FZ_RMSSD_LOW_A,  FZ_RMSSD_LOW_B,  FZ_RMSSD_LOW_C,  SH_LEFT  },
        { FZ_RMSSD_NRM_A,  FZ_RMSSD_NRM_B,  FZ_RMSSD_NRM_C,  SH_NONE  },
        { FZ_RMSSD_HIGH_A, FZ_RMSSD_HIGH_B, FZ_RMSSD_HIGH_C, SH_RIGHT },
    },
    /* IN_RESP */ {
        { FZ_RESP_LOW_A,   FZ_RESP_LOW_B,   FZ_RESP_LOW_C,   SH_LEFT  },
        { FZ_RESP_NRM_A,   FZ_RESP_NRM_B,   FZ_RESP_NRM_C,   SH_NONE  },
        { FZ_RESP_HIGH_A,  FZ_RESP_HIGH_B,  FZ_RESP_HIGH_C,  SH_RIGHT },
    },
    /* IN_TEMP */ {
        { FZ_TEMP_LOW_A,   FZ_TEMP_LOW_B,   FZ_TEMP_LOW_C,   SH_LEFT  },
        { FZ_TEMP_NRM_A,   FZ_TEMP_NRM_B,   FZ_TEMP_NRM_C,   SH_NONE  },
        { FZ_TEMP_HIGH_A,  FZ_TEMP_HIGH_B,  FZ_TEMP_HIGH_C,  SH_RIGHT },
    },
};

static const fz_mf_def_t s_out_mf[N_OUT] = {
    { FZ_OUT_LOW_A,  FZ_OUT_LOW_B,  FZ_OUT_LOW_C,  SH_LEFT  },
    { FZ_OUT_MOD_A,  FZ_OUT_MOD_B,  FZ_OUT_MOD_C,  SH_NONE  },
    { FZ_OUT_ELEV_A, FZ_OUT_ELEV_B, FZ_OUT_ELEV_C, SH_NONE  },
    { FZ_OUT_HIGH_A, FZ_OUT_HIGH_B, FZ_OUT_HIGH_C, SH_RIGHT },
};

/* ===================================================================== */
/* Rule base                                                             */
/* ===================================================================== */

#define RULE1(v,m,o,w,why)              { 1U, {(v),0U,0U}, {(m),0U,0U}, (o), (w), (why) }
#define RULE2(v1,m1,v2,m2,o,w,why)      { 2U, {(v1),(v2),0U}, {(m1),(m2),0U}, (o), (w), (why) }
#define RULE3(v1,m1,v2,m2,v3,m3,o,w,why){ 3U, {(v1),(v2),(v3)}, {(m1),(m2),(m3)}, (o), (w), (why) }

static const fz_rule_t s_rules[] = {
    /* --- everything agrees the subject is fine ------------------------ */
    RULE3(IN_HR, MF_NORM, IN_RMSSD, MF_NORM, IN_TEMP, MF_NORM,
          OUT_LOW,  FZ_W_R1_BASELINE_OK,
          "textbook resting baseline"),
    RULE2(IN_HR, MF_LOW,  IN_RMSSD, MF_HIGH,
          OUT_LOW,  FZ_W_R2_ATHLETIC_REST,
          "low HR with high HRV is vagal tone, not bradycardia"),
    RULE2(IN_HR, MF_NORM, IN_RESP, MF_NORM,
          OUT_LOW,  FZ_W_R3_HR_RESP_OK,
          "cardio-respiratory coupling intact (fires when temp is missing)"),

    /* --- sympathetic activation / stress ------------------------------ */
    RULE2(IN_HR, MF_HIGH, IN_RMSSD, MF_LOW,
          OUT_ELEV, FZ_W_R4_TACHY_LOW_HRV,
          "tachycardia with HRV withdrawal: classic sympathetic dominance"),
    RULE2(IN_RESP, MF_HIGH, IN_RMSSD, MF_LOW,
          OUT_ELEV, FZ_W_R5_TACHYPNEA_LOW_HRV,
          "fast shallow breathing plus low HRV: anxiety / acute stress"),
    RULE2(IN_HR, MF_HIGH, IN_RESP, MF_HIGH,
          OUT_ELEV, FZ_W_R6_TACHY_TACHYPNEA,
          "tachycardia and tachypnoea together"),

    /* --- febrile / multi-system --------------------------------------- */
    RULE2(IN_TEMP, MF_HIGH, IN_HR, MF_HIGH,
          OUT_HIGH, FZ_W_R7_FEVER_TACHY,
          "febrile tachycardia"),
    RULE3(IN_TEMP, MF_HIGH, IN_RESP, MF_HIGH, IN_RMSSD, MF_LOW,
          OUT_HIGH, FZ_W_R8_SIRS_TRIAD,
          "fever + tachypnoea + autonomic withdrawal: SIRS-like triad"),
    RULE1(IN_TEMP, MF_HIGH,
          OUT_ELEV, FZ_W_R9_FEVER_ALONE,
          "raised temperature is significant on its own"),
    RULE1(IN_TEMP, MF_LOW,
          OUT_MOD,  FZ_W_R10_HYPOTHERMIA,
          "low reading: hypothermia or a poorly coupled sensor - worth a look"),

    /* --- single-system findings --------------------------------------- */
    RULE2(IN_RMSSD, MF_LOW, IN_HR, MF_NORM,
          OUT_MOD,  FZ_W_R11_LOW_HRV_ALONE,
          "HRV suppressed while HR still looks normal: early strain"),
    RULE2(IN_RESP, MF_LOW, IN_HR, MF_LOW,
          OUT_MOD,  FZ_W_R12_BRADY_BRADYPNEA,
          "bradycardia with bradypnoea: depressed drive"),

    /* --- single-antecedent fallbacks ----------------------------------
     * These exist so the engine still fires when the leads are off (temp
     * only) or when RMSSD/respiration have not converged yet. Their weights
     * are deliberately small: one input is weak evidence.                */
    RULE1(IN_HR, MF_HIGH,
          OUT_MOD,  FZ_W_R13_TACHY_FALLBACK,
          "HR-only fallback"),
    RULE1(IN_HR, MF_NORM,
          OUT_LOW,  FZ_W_R14_NORMAL_FALLBACK,
          "HR-only fallback"),
};

#define N_RULES ((uint32_t)(sizeof(s_rules) / sizeof(s_rules[0])))

/* ===================================================================== */
/* Precomputed output sets                                               */
/* ===================================================================== */

/* The output membership functions never change, so sample them once at init.
 * Defuzzification then costs N_RULES * FZ_OUT_STEPS compares - no transcendental
 * maths, no branching on shoulder type, every second. */
static float32_t s_out_tab[N_OUT][FZ_OUT_STEPS];
static float32_t s_out_x[FZ_OUT_STEPS];
static float32_t s_agg[FZ_OUT_STEPS];

static float32_t mf_eval(const fz_mf_def_t *m, float32_t x)
{
    switch (m->shoulder) {
    case SH_LEFT:
        if (x <= m->b) { return 1.0f; }
        if (x >= m->c) { return 0.0f; }
        return (m->c - x) / (m->c - m->b);

    case SH_RIGHT:
        if (x >= m->b) { return 1.0f; }
        if (x <= m->a) { return 0.0f; }
        return (x - m->a) / (m->b - m->a);

    default:
        if (x <= m->a || x >= m->c) { return 0.0f; }
        if (x < m->b) { return (x - m->a) / (m->b - m->a); }
        if (x > m->b) { return (m->c - x) / (m->c - m->b); }
        return 1.0f;
    }
}

void fuzzy_init(void)
{
    uint32_t o, i;

    for (i = 0U; i < FZ_OUT_STEPS; i++) {
        s_out_x[i] = (float32_t)i * FZ_OUT_STEP_SIZE;
    }
    for (o = 0U; o < N_OUT; o++) {
        for (i = 0U; i < FZ_OUT_STEPS; i++) {
            s_out_tab[o][i] = mf_eval(&s_out_mf[o], s_out_x[i]);
        }
    }
}

/* ===================================================================== */
/* Inference                                                             */
/* ===================================================================== */

void fuzzy_evaluate(const fuzzy_inputs_t *in, fuzzy_result_t *out)
{
    float32_t mu[N_IN][N_MF];
    uint8_t   avail[N_IN];
    float32_t num = 0.0f, den = 0.0f, strongest = 0.0f;
    uint32_t  r, a, i, v;
    uint8_t   n_avail = 0U;

    memset(out, 0, sizeof(*out));
    memset(mu, 0, sizeof(mu));

    /* ---- fuzzify ---- */
    avail[IN_HR]    = in->hr_valid;
    avail[IN_RMSSD] = in->rmssd_valid;
    avail[IN_RESP]  = in->resp_valid;
    avail[IN_TEMP]  = in->temp_valid;

    {
        const float32_t x[N_IN] = {
            in->hr_bpm, in->rmssd_ms, in->resp_brpm, in->temp_c
        };
        for (v = 0U; v < N_IN; v++) {
            if (!avail[v]) { continue; }
            n_avail++;
            for (i = 0U; i < N_MF; i++) {
                mu[v][i] = mf_eval(&s_in_mf[v][i], x[v]);
            }
        }
    }

    out->inputs_used  = n_avail;
    out->mu_temp_high = avail[IN_TEMP] ? mu[IN_TEMP][MF_HIGH] : 0.0f;

    if (n_avail == 0U) {
        out->state = FUZZY_STATE_UNKNOWN;    /* leads off AND no temperature */
        return;
    }

    /* ---- inference and aggregation ---- */
    memset(s_agg, 0, sizeof(s_agg));

    for (r = 0U; r < N_RULES; r++) {
        const fz_rule_t *rule = &s_rules[r];
        float32_t strength = 1.0f;
        uint8_t   usable   = 1U;

        for (a = 0U; a < rule->n_ant; a++) {
            const uint8_t var = rule->var[a];
            if (!avail[var]) { usable = 0U; break; }     /* skip, never guess */
            /* AND == min */
            if (mu[var][rule->mf[a]] < strength) { strength = mu[var][rule->mf[a]]; }
        }
        if (!usable || strength <= 0.0f) { continue; }

        strength *= rule->w;
        if (strength > strongest) { strongest = strength; }
        out->rules_fired++;

        /* Mamdani implication: clip the consequent set at the firing strength,
         * then aggregate with max. */
        {
            const float32_t *tab = s_out_tab[rule->out];
            for (i = 0U; i < FZ_OUT_STEPS; i++) {
                const float32_t clipped = (tab[i] < strength) ? tab[i] : strength;
                if (clipped > s_agg[i]) { s_agg[i] = clipped; }
            }
        }
    }

    if (out->rules_fired == 0U) {
        /* Inputs exist but every one of them sits in a region no rule covers.
         * Saying UNKNOWN is honest; inventing a midpoint is not. */
        out->state      = FUZZY_STATE_UNKNOWN;
        out->risk       = 0.0f;
        out->confidence = 0.0f;
        return;
    }

    /* ---- centroid defuzzification ---- */
    for (i = 0U; i < FZ_OUT_STEPS; i++) {
        num += s_out_x[i] * s_agg[i];
        den += s_agg[i];
    }
    out->risk = (den > 1e-6f) ? (num / den) : 0.0f;

    /* ---- confidence ----
     * Two independent discounts: how many of the four inputs were actually
     * measured, and how strongly the winning rule fired. A single weakly
     * matching input must not read as a confident diagnosis. */
    out->confidence = ((float32_t)n_avail / (float32_t)N_IN)
                    * (FZ_CONF_ACTIVATION_FLOOR
                       + (1.0f - FZ_CONF_ACTIVATION_FLOOR) * strongest);
    if (out->confidence > 1.0f) { out->confidence = 1.0f; }

    /* ---- risk score -> label ----
     * CONCERNING is checked first: once the score is that high, more than one
     * subsystem is flagged and "FEVERISH" would under-describe it. FEVERISH is
     * the specific label for the case where temperature is the dominant
     * finding, so it needs the temperature membership, not just the score. */
    if (out->risk >= FZ_STATE_CONCERNING_MIN) {
        out->state = FUZZY_STATE_CONCERNING;
    } else if (out->mu_temp_high >= FZ_FEVER_TEMP_MU_MIN &&
               out->risk >= FZ_STATE_FEVERISH_MIN) {
        out->state = FUZZY_STATE_FEVERISH;
    } else if (out->risk >= FZ_STATE_STRESSED_MIN) {
        out->state = FUZZY_STATE_STRESSED;
    } else {
        out->state = FUZZY_STATE_NORMAL;
    }
}

const char *fuzzy_state_name(fuzzy_state_t s)
{
    switch (s) {
    case FUZZY_STATE_NORMAL:     return "NORMAL";
    case FUZZY_STATE_STRESSED:   return "STRESSED";
    case FUZZY_STATE_FEVERISH:   return "FEVERISH";
    case FUZZY_STATE_CONCERNING: return "CONCERNING";
    default:                     return "UNKNOWN";
    }
}
