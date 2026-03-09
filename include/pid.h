/**
 * @file pid.h
 * @brief PID controller public API for FOC motor control.
 *
 * This module provides a generic incremental/positional PID controller
 * suitable for current-loop, speed-loop and position-loop control in
 * brushless-DC (BLDC) motor applications.
 */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Data types
 * ---------------------------------------------------------------------- */

/**
 * @brief PID controller state and parameter structure.
 *
 * All members must be initialised (typically to zero) before the first
 * call to pid_calc().
 */
typedef struct {
    float kp;           /**< Proportional gain */
    float ki;           /**< Integral gain     */
    float kd;           /**< Derivative gain   */
    float integral;     /**< Accumulated integral term */
    float prev_error;   /**< Error value from the previous sample */
    float output_min;   /**< Lower saturation limit for the output */
    float output_max;   /**< Upper saturation limit for the output */
} pid_t;

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/**
 * @brief Initialise a PID controller instance.
 *
 * Resets the internal state (integral, previous error) and stores the
 * supplied tuning parameters and output limits.
 *
 * @param[out] pid        Pointer to the PID instance to initialise.
 * @param[in]  kp         Proportional gain.
 * @param[in]  ki         Integral gain.
 * @param[in]  kd         Derivative gain.
 * @param[in]  output_min Lower saturation limit (e.g. -100.0f for ±100 %).
 * @param[in]  output_max Upper saturation limit.
 */
void pid_init(pid_t *pid, float kp, float ki, float kd,
              float output_min, float output_max);

/**
 * @brief Compute one PID output sample.
 *
 * Implements a standard positional PID algorithm with output clamping:
 * @code
 *   output = kp*e + ki*integral + kd*(e - prev_e)
 * @endcode
 * Anti-windup is applied by clamping the integral accumulator whenever
 * the output saturates.
 *
 * @param[in,out] pid     Pointer to the PID instance.
 * @param[in]     setpoint Desired target value.
 * @param[in]     measured Actual measured value.
 * @return Computed controller output, clamped to [output_min, output_max].
 */
float pid_calc(pid_t *pid, float setpoint, float measured);

/**
 * @brief Reset the PID controller state.
 *
 * Clears the integral accumulator and the stored previous error.
 * Tuning parameters are left unchanged.
 *
 * @param[in,out] pid Pointer to the PID instance.
 */
void pid_reset(pid_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
