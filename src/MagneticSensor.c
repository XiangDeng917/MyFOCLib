/**
 * @file MagneticSensor.c
 * @brief Magnetic angle sensor driver implementation.
 *
 * Velocity is estimated using a simple backward-difference of consecutive
 * angle measurements, with wrap-around handling for the 0/2π boundary.
 */

#include "MagneticSensor.h"
#include <math.h>   /* fabsf */

/* -------------------------------------------------------------------------
 * Constants
 * ---------------------------------------------------------------------- */

#define TWO_PI  6.28318530718f

/* -------------------------------------------------------------------------
 * Internal helpers
 * ---------------------------------------------------------------------- */

/**
 * @brief Normalise an angle to [0, 2π).
 */
static float normalise_angle(float angle)
{
    while (angle < 0.0f)        angle += TWO_PI;
    while (angle >= TWO_PI)     angle -= TWO_PI;
    return angle;
}

/**
 * @brief Compute the shortest signed angle difference (handles wrap-around).
 */
static float angle_diff(float new_angle, float old_angle)
{
    float diff = new_angle - old_angle;
    if (diff >  (TWO_PI / 2.0f)) diff -= TWO_PI;
    if (diff < -(TWO_PI / 2.0f)) diff += TWO_PI;
    return diff;
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

void mag_sensor_init(mag_sensor_t *sensor, mag_read_fn_t read_raw_fn,
                     uint32_t cpr, float sample_time_s)
{
    if (!sensor || !read_raw_fn) return;

    sensor->read_raw      = read_raw_fn;
    sensor->cpr           = cpr;
    sensor->angle_rad     = 0.0f;
    sensor->angle_prev    = 0.0f;
    sensor->velocity_rads = 0.0f;
    sensor->sample_time_s = sample_time_s;
}

void mag_sensor_update(mag_sensor_t *sensor)
{
    if (!sensor) return;

    /* Read raw encoder count and convert to radians */
    uint16_t raw     = sensor->read_raw();
    float    raw_rad = (float)raw * (TWO_PI / (float)sensor->cpr);
    float    new_angle = normalise_angle(raw_rad);

    /* Estimate angular velocity */
    float dt = sensor->sample_time_s;
    if (dt > 0.0f) {
        sensor->velocity_rads = angle_diff(new_angle, sensor->angle_prev) / dt;
    }

    sensor->angle_prev = sensor->angle_rad;
    sensor->angle_rad  = new_angle;
}

float mag_sensor_get_angle(const mag_sensor_t *sensor)
{
    if (!sensor) return 0.0f;
    return sensor->angle_rad;
}

float mag_sensor_get_velocity(const mag_sensor_t *sensor)
{
    if (!sensor) return 0.0f;
    return sensor->velocity_rads;
}
