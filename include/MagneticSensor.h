/**
 * @file MagneticSensor.h
 * @brief Magnetic angle sensor (e.g. AS5048A / AS5600) public API.
 *
 * Abstracts a SPI- or I2C-attached magnetic encoder into a simple
 * interface that returns the mechanical shaft angle in radians and the
 * electrical speed in rad/s.  The concrete hardware read function must
 * be supplied by the user at initialisation time (dependency injection),
 * making the module portable across different MCU platforms.
 */

#ifndef MAGNETIC_SENSOR_H
#define MAGNETIC_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* -------------------------------------------------------------------------
 * Constants
 * ---------------------------------------------------------------------- */

/** Maximum raw count for a 14-bit encoder (e.g. AS5048A). */
#define MAGNETIC_SENSOR_MAX_COUNT_14BIT  16384U

/** Conversion factor: raw counts → radians (2π / MAX_COUNT). */
#define MAGNETIC_SENSOR_CPR_TO_RAD       (6.28318530718f / (float)MAGNETIC_SENSOR_MAX_COUNT_14BIT)

/* -------------------------------------------------------------------------
 * Data types
 * ---------------------------------------------------------------------- */

/**
 * @brief Pointer to a platform-specific raw-count read function.
 *
 * The function must return the absolute encoder position as a 16-bit
 * unsigned integer (0 … MAX_COUNT-1).
 */
typedef uint16_t (*mag_read_fn_t)(void);

/**
 * @brief Magnetic sensor driver state.
 */
typedef struct {
    mag_read_fn_t   read_raw;       /**< Hardware read callback (must not be NULL) */
    uint32_t        cpr;            /**< Counts per revolution (e.g. 16384 for 14-bit) */
    float           angle_rad;      /**< Last computed mechanical angle [rad] */
    float           angle_prev;     /**< Angle from previous sample [rad]    */
    float           velocity_rads;  /**< Electrical angular velocity [rad/s] */
    float           sample_time_s;  /**< Sampling period [s]                 */
} mag_sensor_t;

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/**
 * @brief Initialise a magnetic sensor instance.
 *
 * @param[out] sensor         Pointer to the sensor instance.
 * @param[in]  read_raw_fn    Platform-specific read callback.
 * @param[in]  cpr            Counts per revolution of the encoder.
 * @param[in]  sample_time_s  Sampling period in seconds (used for velocity estimation).
 */
void mag_sensor_init(mag_sensor_t *sensor, mag_read_fn_t read_raw_fn,
                     uint32_t cpr, float sample_time_s);

/**
 * @brief Update the angle and velocity estimate.
 *
 * Must be called at a regular interval equal to @p sample_time_s.
 *
 * @param[in,out] sensor Pointer to the sensor instance.
 */
void mag_sensor_update(mag_sensor_t *sensor);

/**
 * @brief Return the latest mechanical shaft angle.
 *
 * @param[in] sensor Pointer to the sensor instance.
 * @return Shaft angle in radians, in the range [0, 2π).
 */
float mag_sensor_get_angle(const mag_sensor_t *sensor);

/**
 * @brief Return the latest angular velocity estimate.
 *
 * @param[in] sensor Pointer to the sensor instance.
 * @return Angular velocity in rad/s (positive = forward direction).
 */
float mag_sensor_get_velocity(const mag_sensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif /* MAGNETIC_SENSOR_H */
