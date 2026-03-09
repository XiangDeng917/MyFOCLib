/**
 * @file foc.h
 * @brief Field-Oriented Control (FOC) core algorithm public API.
 *
 * Provides the fundamental coordinate transformations and space-vector
 * modulation (SVM) routines used by the BLDC motor driver:
 *
 *  - Clarke transform  (abc → αβ)
 *  - Park transform    (αβ → dq)
 *  - Inverse Park      (dq → αβ)
 *  - Inverse Clarke    (αβ → abc)
 *  - Space-vector PWM  (αβ → duty cycles)
 *
 * All angles are in radians.  The convention used is:
 *   - α-axis aligned with phase-A magnetic axis
 *   - d-axis aligned with rotor flux
 */

#ifndef FOC_H
#define FOC_H

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Data types
 * ---------------------------------------------------------------------- */

/** Two-component αβ (stationary) or dq (rotating) vector. */
typedef struct {
    float x; /**< α or d component */
    float y; /**< β or q component */
} foc_vec2_t;

/** Three-phase (abc) voltage or current vector. */
typedef struct {
    float a; /**< Phase-A component */
    float b; /**< Phase-B component */
    float c; /**< Phase-C component */
} foc_vec3_t;

/** PWM duty cycle outputs for the three half-bridges. */
typedef struct {
    float ta; /**< Phase-A duty cycle [0, 1] */
    float tb; /**< Phase-B duty cycle [0, 1] */
    float tc; /**< Phase-C duty cycle [0, 1] */
} foc_pwm_t;

/* -------------------------------------------------------------------------
 * Coordinate transformations
 * ---------------------------------------------------------------------- */

/**
 * @brief Clarke transform: balanced three-phase → stationary αβ frame.
 *
 * Assumes balanced three-phase quantities (ia + ib + ic = 0), so only
 * ia and ib are required as inputs.
 *
 * @param[in]  abc  Three-phase input vector (only .a and .b are used).
 * @param[out] ab   Resulting αβ vector.
 */
void foc_clarke(foc_vec3_t abc, foc_vec2_t *ab);

/**
 * @brief Inverse Clarke transform: stationary αβ → balanced three-phase.
 *
 * @param[in]  ab   αβ voltage reference vector.
 * @param[out] abc  Resulting three-phase reference vector.
 */
void foc_clarke_inv(foc_vec2_t ab, foc_vec3_t *abc);

/**
 * @brief Park transform: stationary αβ → rotating dq frame.
 *
 * @param[in]  ab       Stationary αβ vector.
 * @param[in]  theta    Electrical rotor angle [rad].
 * @param[out] dq       Resulting dq vector.
 */
void foc_park(foc_vec2_t ab, float theta, foc_vec2_t *dq);

/**
 * @brief Inverse Park transform: rotating dq → stationary αβ frame.
 *
 * @param[in]  dq       Rotating dq reference vector.
 * @param[in]  theta    Electrical rotor angle [rad].
 * @param[out] ab       Resulting αβ reference vector.
 */
void foc_park_inv(foc_vec2_t dq, float theta, foc_vec2_t *ab);

/* -------------------------------------------------------------------------
 * Space-vector modulation
 * ---------------------------------------------------------------------- */

/**
 * @brief Generate three-phase SVM duty cycles from an αβ voltage reference.
 *
 * The αβ reference is normalised by @p vbus so that the output duty cycles
 * are always in the range [0, 1].  The modulation index must satisfy
 * |v_ab| ≤ vbus / √3 for linear operation.
 *
 * @param[in]  ab    Voltage reference in αβ frame [V].
 * @param[in]  vbus  DC bus voltage [V] (used for normalisation).
 * @param[out] pwm   Three-phase PWM duty cycles.
 */
void foc_svm(foc_vec2_t ab, float vbus, foc_pwm_t *pwm);

#ifdef __cplusplus
}
#endif

#endif /* FOC_H */
