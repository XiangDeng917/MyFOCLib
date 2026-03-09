/**
 * @file pid.c
 * @brief Positional PID controller implementation.
 *
 * The controller applies integral anti-windup by clamping the accumulator
 * when the computed output is outside [output_min, output_max].
 */

#include "pid.h"

/* -------------------------------------------------------------------------
 * Internal helpers
 * ---------------------------------------------------------------------- */

static float clampf(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

void pid_init(pid_t *pid, float kp, float ki, float kd,
              float output_min, float output_max)
{
    if (!pid) return;

    pid->kp         = kp;
    pid->ki         = ki;
    pid->kd         = kd;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

float pid_calc(pid_t *pid, float setpoint, float measured)
{
    if (!pid) return 0.0f;

    float error      = setpoint - measured;
    float derivative = error - pid->prev_error;

    /* Accumulate integral */
    pid->integral += error;

    /* Compute raw output */
    float output = pid->kp * error
                 + pid->ki * pid->integral
                 + pid->kd * derivative;

    /* Anti-windup: undo integral contribution when saturated */
    if (output > pid->output_max || output < pid->output_min) {
        pid->integral -= error;
        output = clampf(output, pid->output_min, pid->output_max);
    }

    pid->prev_error = error;
    return output;
}

void pid_reset(pid_t *pid)
{
    if (!pid) return;

    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}
