/**
 * @file BLDCMotor.h
 * @brief Brushless DC (BLDC) motor driver public API.
 *
 * This module manages a single three-phase BLDC motor running under
 * Field-Oriented Control (FOC).  It owns the speed/position PID loops,
 * interfaces with the magnetic sensor for feedback, and drives the PWM
 * duty cycles that are forwarded to the power stage.
 */

#ifndef BLDC_MOTOR_H
#define BLDC_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pid.h"
#include "my_filter.h"
#include "MagneticSensor.h"

/* -------------------------------------------------------------------------
 * Constants
 * ---------------------------------------------------------------------- */

/** Number of electrical commutation steps (6 for standard BLDC). */
#define BLDC_COMMUTATION_STEPS  6U

/* -------------------------------------------------------------------------
 * Data types
 * ---------------------------------------------------------------------- */

/** Motor operating mode. */
typedef enum {
    BLDC_MODE_IDLE    = 0, /**< Motor is disabled / coasting */
    BLDC_MODE_SPEED   = 1, /**< Closed-loop speed control     */
    BLDC_MODE_TORQUE  = 2  /**< Open-loop torque (duty-cycle) control */
} bldc_mode_t;

/**
 * @brief BLDC motor instance.
 *
 * Aggregates all runtime state required to run one motor axis.
 */
typedef struct {
    /* Physical parameters */
    uint8_t         pole_pairs;     /**< Number of electrical pole pairs */

    /* Control mode */
    bldc_mode_t     mode;           /**< Active operating mode */
    float           target_speed;   /**< Target speed in RPM (speed mode) */
    float           target_torque;  /**< Target duty cycle 0–1 (torque mode) */

    /* Sensor */
    mag_sensor_t   *sensor;         /**< Pointer to the position/speed sensor */

    /* Controllers */
    pid_t           speed_pid;      /**< Speed-loop PID controller */

    /* Signal filtering */
    lp_filter_t     speed_filter;   /**< Low-pass filter for speed feedback */

    /* Outputs */
    float           pwm_duty[3];    /**< PWM duty cycles for phases U, V, W (0–1) */
    float           current_speed;  /**< Filtered speed estimate in RPM */
} bldc_motor_t;

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/**
 * @brief Initialise a BLDC motor instance.
 *
 * @param[out] motor       Pointer to the motor instance.
 * @param[in]  pole_pairs  Number of magnetic pole pairs.
 * @param[in]  sensor      Pointer to an already-initialised magnetic sensor.
 */
void bldc_motor_init(bldc_motor_t *motor, uint8_t pole_pairs,
                     mag_sensor_t *sensor);

/**
 * @brief Configure the speed-loop PID gains.
 *
 * @param[in,out] motor Pointer to the motor instance.
 * @param[in]     kp    Proportional gain.
 * @param[in]     ki    Integral gain.
 * @param[in]     kd    Derivative gain.
 */
void bldc_set_speed_pid(bldc_motor_t *motor, float kp, float ki, float kd);

/**
 * @brief Set the target speed for closed-loop speed control.
 *
 * Switches the operating mode to BLDC_MODE_SPEED.
 *
 * @param[in,out] motor     Pointer to the motor instance.
 * @param[in]     speed_rpm Target speed in RPM.
 */
void bldc_set_target_speed(bldc_motor_t *motor, float speed_rpm);

/**
 * @brief Run one FOC update tick.
 *
 * Must be called at the configured sampling rate.  The function:
 *  1. Reads the sensor angle and velocity.
 *  2. Converts velocity to RPM.
 *  3. Applies the speed filter.
 *  4. Runs the PID controller (in speed mode).
 *  5. Populates @c motor->pwm_duty[] with the new duty cycles.
 *
 * @param[in,out] motor Pointer to the motor instance.
 */
void bldc_update(bldc_motor_t *motor);

/**
 * @brief Read the current filtered speed estimate.
 *
 * @param[in] motor Pointer to the motor instance.
 * @return Filtered speed in RPM.
 */
float bldc_get_rpm(const bldc_motor_t *motor);

/**
 * @brief Disable the motor and reset all controllers.
 *
 * @param[in,out] motor Pointer to the motor instance.
 */
void bldc_stop(bldc_motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* BLDC_MOTOR_H */
