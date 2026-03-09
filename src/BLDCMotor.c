/**
 * @file BLDCMotor.c
 * @brief BLDC motor driver implementation (FOC speed control loop).
 *
 * The update function executes a simplified FOC cycle:
 *   sensor update → speed feedback → PID → SVM duty cycles
 */

#include "BLDCMotor.h"
#include "foc.h"
#include <string.h>  /* memset */

/* -------------------------------------------------------------------------
 * Constants
 * ---------------------------------------------------------------------- */

#define RAD_S_TO_RPM   (60.0f / 6.28318530718f)

/* Default speed PID output limits (normalised duty cycle -1 … +1) */
#define SPEED_PID_OUT_MIN   (-1.0f)
#define SPEED_PID_OUT_MAX   ( 1.0f)

/* Default speed filter smoothing coefficient */
#define SPEED_FILTER_ALPHA  0.1f

/* Default DC bus voltage used for SVM normalisation */
#define DEFAULT_VBUS        12.0f

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

void bldc_motor_init(bldc_motor_t *motor, uint8_t pole_pairs,
                     mag_sensor_t *sensor)
{
    if (!motor) return;

    memset(motor, 0, sizeof(*motor));

    motor->pole_pairs = pole_pairs;
    motor->sensor     = sensor;
    motor->mode       = BLDC_MODE_IDLE;

    pid_init(&motor->speed_pid, 0.5f, 0.01f, 0.0f,
             SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX);

    filter_init(&motor->speed_filter, SPEED_FILTER_ALPHA, 0.0f);
}

void bldc_set_speed_pid(bldc_motor_t *motor, float kp, float ki, float kd)
{
    if (!motor) return;

    pid_init(&motor->speed_pid, kp, ki, kd,
             SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX);
}

void bldc_set_target_speed(bldc_motor_t *motor, float speed_rpm)
{
    if (!motor) return;

    motor->target_speed = speed_rpm;
    motor->mode         = BLDC_MODE_SPEED;
}

void bldc_update(bldc_motor_t *motor)
{
    if (!motor || !motor->sensor || motor->mode == BLDC_MODE_IDLE) return;

    /* 1. Update sensor */
    mag_sensor_update(motor->sensor);

    /* 2. Convert angular velocity to RPM */
    float vel_rads = mag_sensor_get_velocity(motor->sensor);
    float raw_rpm  = vel_rads * RAD_S_TO_RPM * (float)motor->pole_pairs;

    /* 3. Low-pass filter the speed */
    motor->current_speed = filter_update(&motor->speed_filter, raw_rpm);

    if (motor->mode == BLDC_MODE_SPEED) {
        /* 4. Run speed PID */
        float pid_out = pid_calc(&motor->speed_pid,
                                 motor->target_speed,
                                 motor->current_speed);

        /* 5. Simple SVM: map scalar duty directly to α-axis reference */
        foc_vec2_t v_ab;
        v_ab.x = pid_out * DEFAULT_VBUS;
        v_ab.y = 0.0f;

        /* Rotate reference to align with electrical angle */
        float theta = mag_sensor_get_angle(motor->sensor)
                      * (float)motor->pole_pairs;
        foc_vec2_t v_dq;
        foc_park(v_ab, theta, &v_dq);
        foc_park_inv(v_dq, theta, &v_ab);

        foc_pwm_t pwm;
        foc_svm(v_ab, DEFAULT_VBUS, &pwm);

        motor->pwm_duty[0] = pwm.ta;
        motor->pwm_duty[1] = pwm.tb;
        motor->pwm_duty[2] = pwm.tc;
    }
}

float bldc_get_rpm(const bldc_motor_t *motor)
{
    if (!motor) return 0.0f;
    return motor->current_speed;
}

void bldc_stop(bldc_motor_t *motor)
{
    if (!motor) return;

    motor->mode          = BLDC_MODE_IDLE;
    motor->pwm_duty[0]   = 0.0f;
    motor->pwm_duty[1]   = 0.0f;
    motor->pwm_duty[2]   = 0.0f;
    motor->current_speed = 0.0f;
    pid_reset(&motor->speed_pid);
    filter_reset(&motor->speed_filter, 0.0f);
}
