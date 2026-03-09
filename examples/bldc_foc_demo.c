/**
 * @file bldc_foc_demo.c
 * @brief Complete demonstration of MyFOCLib usage.
 *
 * This example shows how to:
 *   1. Implement the platform sensor read callback.
 *   2. Initialise the magnetic sensor and the BLDC motor.
 *   3. Set PID gains and a target speed.
 *   4. Run the FOC control loop for a fixed number of iterations.
 *   5. Read back the filtered speed estimate.
 *
 * On a real embedded target, replace the simulated sensor_read_raw() with
 * actual hardware SPI/I2C reads and drive the PWM hardware with the values
 * returned in motor.pwm_duty[].
 *
 * Build with CMake (see README):
 *   cmake -B build && cmake --build build
 *   ./build/bldc_foc_demo
 */

#include <stdio.h>
#include <stdint.h>

#include "BLDCMotor.h"
#include "MagneticSensor.h"
#include "delay.h"

/* -------------------------------------------------------------------------
 * Simulated sensor
 * ---------------------------------------------------------------------- */

/** Simulated 14-bit encoder count that increments each call. */
static uint16_t s_encoder_count = 0;

/**
 * @brief Stub sensor read function.
 *
 * In a real application this would perform an SPI or I2C transaction to
 * read the absolute angle from the magnetic encoder IC (e.g. AS5048A).
 */
static uint16_t sensor_read_raw(void)
{
    /* Simulate constant ~1000 RPM rotation:
     *   electrical_speed = 1000 RPM × (2π/60) × pole_pairs
     *   ≈ 104.7 rad/s  (for 1 pole pair)
     *   Δcount per 1 ms tick = 104.7 × 0.001 × (16384 / 2π) ≈ 273 counts */
    s_encoder_count = (uint16_t)((s_encoder_count + 273U)
                                 % MAGNETIC_SENSOR_MAX_COUNT_14BIT);
    return s_encoder_count;
}

/* -------------------------------------------------------------------------
 * Main
 * ---------------------------------------------------------------------- */

int main(void)
{
    printf("=== MyFOCLib BLDC FOC Demo ===\n\n");

    /* -----------------------------------------------------------------
     * Step 1: Initialise the magnetic sensor
     * Sample time 1 ms, 14-bit encoder (16384 counts/rev), 1 pole pair.
     * ----------------------------------------------------------------- */
    mag_sensor_t sensor;
    mag_sensor_init(&sensor, sensor_read_raw,
                    MAGNETIC_SENSOR_MAX_COUNT_14BIT, 0.001f);

    /* -----------------------------------------------------------------
     * Step 2: Initialise the BLDC motor
     * 1 pole pair, attach the sensor created above.
     * ----------------------------------------------------------------- */
    bldc_motor_t motor;
    bldc_motor_init(&motor, 1, &sensor);

    /* -----------------------------------------------------------------
     * Step 3: Configure speed-loop PID gains
     * Tune kp/ki/kd to match your motor and mechanical load.
     * ----------------------------------------------------------------- */
    bldc_set_speed_pid(&motor, 0.8f, 0.05f, 0.01f);

    /* -----------------------------------------------------------------
     * Step 4: Set target speed and enter closed-loop control
     * ----------------------------------------------------------------- */
    float target_rpm = 1000.0f;
    bldc_set_target_speed(&motor, target_rpm);
    printf("Target speed : %.1f RPM\n", (double)target_rpm);

    /* -----------------------------------------------------------------
     * Step 5: Run the control loop
     * In embedded code this would be triggered by a 1 kHz timer ISR.
     * ----------------------------------------------------------------- */
    printf("Running 200 update ticks (200 ms simulated) ...\n\n");

    for (int tick = 0; tick < 200; tick++) {
        bldc_update(&motor);

        /* Print status every 50 ticks */
        if ((tick + 1) % 50 == 0) {
            float rpm  = bldc_get_rpm(&motor);
            float angle_rad = mag_sensor_get_angle(&sensor);
            printf("  tick %3d | speed = %7.2f RPM | angle = %.4f rad"
                   " | duty[U,V,W] = [%.3f, %.3f, %.3f]\n",
                   tick + 1, (double)rpm, (double)angle_rad,
                   (double)motor.pwm_duty[0], (double)motor.pwm_duty[1],
                   (double)motor.pwm_duty[2]);
        }

        /* On a real target, insert a 1 ms hardware timer wait here */
        delay_ms(1);
    }

    /* -----------------------------------------------------------------
     * Step 6: Stop the motor
     * ----------------------------------------------------------------- */
    bldc_stop(&motor);
    printf("\nMotor stopped.  Final speed = %.2f RPM\n", (double)bldc_get_rpm(&motor));

    return 0;
}
