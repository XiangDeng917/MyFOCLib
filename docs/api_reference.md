# MyFOCLib API Reference

This document describes every public function and data type exported by
**MyFOCLib**.  All declarations live in the `include/` directory.

---

## Table of Contents

1. [PID Controller — `pid.h`](#1-pid-controller)
2. [Low-Pass Filter — `my_filter.h`](#2-low-pass-filter)
3. [Magnetic Sensor — `MagneticSensor.h`](#3-magnetic-sensor)
4. [BLDC Motor Driver — `BLDCMotor.h`](#4-bldc-motor-driver)
5. [FOC Core — `foc.h`](#5-foc-core)
6. [Delay Utilities — `delay.h`](#6-delay-utilities)

---

## 1. PID Controller

**Header:** `include/pid.h`  
**Source:** `src/pid.c`

### Data type: `pid_t`

```c
typedef struct {
    float kp;           /* Proportional gain       */
    float ki;           /* Integral gain           */
    float kd;           /* Derivative gain         */
    float integral;     /* Accumulated integral    */
    float prev_error;   /* Previous sample error   */
    float output_min;   /* Lower saturation limit  */
    float output_max;   /* Upper saturation limit  */
} pid_t;
```

### `pid_init()`

```c
void pid_init(pid_t *pid, float kp, float ki, float kd,
              float output_min, float output_max);
```

Initialises the PID instance and resets all state (integral, prev_error).

| Parameter    | Description                              |
|--------------|------------------------------------------|
| `pid`        | Pointer to the PID instance              |
| `kp`         | Proportional gain                        |
| `ki`         | Integral gain                            |
| `kd`         | Derivative gain                          |
| `output_min` | Minimum clamped output (e.g. `-100.0f`) |
| `output_max` | Maximum clamped output (e.g.  `100.0f`) |

**Example:**

```c
pid_t speed_pid;
pid_init(&speed_pid, 0.8f, 0.05f, 0.01f, -1.0f, 1.0f);
```

---

### `pid_calc()`

```c
float pid_calc(pid_t *pid, float setpoint, float measured);
```

Computes one output sample.  Anti-windup is applied: when the output
saturates, the integral accumulator is rolled back.

| Parameter  | Description                 |
|------------|-----------------------------|
| `pid`      | PID instance                |
| `setpoint` | Desired target value        |
| `measured` | Current measured value      |
| **Returns**| PID output, clamped to `[output_min, output_max]` |

**Example:**

```c
float duty = pid_calc(&speed_pid, target_rpm, current_rpm);
```

---

### `pid_reset()`

```c
void pid_reset(pid_t *pid);
```

Clears `integral` and `prev_error`.  Gains and limits are preserved.

---

## 2. Low-Pass Filter

**Header:** `include/my_filter.h`  
**Source:** `src/my_filter.c`

Implements a first-order IIR filter:

```
y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
```

### Data type: `lp_filter_t`

```c
typedef struct {
    float alpha;   /* Smoothing coefficient in (0, 1] */
    float output;  /* Current filter state            */
} lp_filter_t;
```

### `filter_init()`

```c
void filter_init(lp_filter_t *f, float alpha, float initial_value);
```

| Parameter       | Description                                    |
|-----------------|------------------------------------------------|
| `f`             | Filter instance                                |
| `alpha`         | Smoothing factor: smaller = more filtering     |
| `initial_value` | Starting output value (use `0.0f` if unknown)  |

**Choosing alpha:** For a sampling frequency *fs* and desired cutoff *fc*:

```
dt = 1/fs
tau = 1/(2*pi*fc)
alpha ≈ dt / (tau + dt)
```

**Example:** 1 kHz sampling, 10 Hz cutoff → `alpha ≈ 0.059`

---

### `filter_update()`

```c
float filter_update(lp_filter_t *f, float input);
```

| Parameter | Description     |
|-----------|-----------------|
| `f`       | Filter instance |
| `input`   | Raw input sample |
| **Returns** | Filtered output |

---

### `filter_reset()`

```c
void filter_reset(lp_filter_t *f, float value);
```

Sets the filter state to `value` for bumpless transfer.

---

## 3. Magnetic Sensor

**Header:** `include/MagneticSensor.h`  
**Source:** `src/MagneticSensor.c`

Abstracts a 14-bit magnetic encoder (e.g. AS5048A / AS5600).

### Data type: `mag_sensor_t`

```c
typedef struct {
    mag_read_fn_t   read_raw;       /* Hardware read callback  */
    uint32_t        cpr;            /* Counts per revolution   */
    float           angle_rad;      /* Latest angle [rad]      */
    float           angle_prev;     /* Previous angle [rad]    */
    float           velocity_rads;  /* Angular velocity [rad/s]*/
    float           sample_time_s;  /* Sampling period [s]     */
} mag_sensor_t;
```

### `mag_sensor_init()`

```c
void mag_sensor_init(mag_sensor_t *sensor, mag_read_fn_t read_raw_fn,
                     uint32_t cpr, float sample_time_s);
```

`read_raw_fn` must be a pointer to a function with signature `uint16_t fn(void)`.

---

### `mag_sensor_update()`

```c
void mag_sensor_update(mag_sensor_t *sensor);
```

Call at the rate specified by `sample_time_s`.  Updates `angle_rad` and
`velocity_rads`.

---

### `mag_sensor_get_angle()` / `mag_sensor_get_velocity()`

```c
float mag_sensor_get_angle(const mag_sensor_t *sensor);
float mag_sensor_get_velocity(const mag_sensor_t *sensor);
```

Return the latest angle [rad, 0..2π) and velocity [rad/s].

---

## 4. BLDC Motor Driver

**Header:** `include/BLDCMotor.h`  
**Source:** `src/BLDCMotor.c`

### `bldc_motor_init()`

```c
void bldc_motor_init(bldc_motor_t *motor, uint8_t pole_pairs,
                     mag_sensor_t *sensor);
```

Initialises the motor with default speed PID (kp=0.5, ki=0.01, kd=0) and
speed filter (α=0.1).

---

### `bldc_set_speed_pid()`

```c
void bldc_set_speed_pid(bldc_motor_t *motor, float kp, float ki, float kd);
```

Reconfigures speed-loop PID gains at runtime.

---

### `bldc_set_target_speed()`

```c
void bldc_set_target_speed(bldc_motor_t *motor, float speed_rpm);
```

Sets the target speed and switches to `BLDC_MODE_SPEED`.

---

### `bldc_update()`

```c
void bldc_update(bldc_motor_t *motor);
```

One FOC tick: reads sensor → estimates speed → runs PID → updates
`motor->pwm_duty[3]`.  Call at the same rate as sensor sampling.

---

### `bldc_get_rpm()`

```c
float bldc_get_rpm(const bldc_motor_t *motor);
```

Returns the filtered speed estimate in RPM.

---

### `bldc_stop()`

```c
void bldc_stop(bldc_motor_t *motor);
```

Sets all duty cycles to 0 and resets all controllers.

---

## 5. FOC Core

**Header:** `include/foc.h`  
**Source:** `src/foc.c`

### Coordinate transforms

| Function          | Direction          |
|-------------------|--------------------|
| `foc_clarke()`    | abc → αβ           |
| `foc_clarke_inv()`| αβ → abc           |
| `foc_park()`      | αβ → dq (rotating) |
| `foc_park_inv()`  | dq → αβ            |

All angles in **radians**.

---

### `foc_svm()`

```c
void foc_svm(foc_vec2_t ab, float vbus, foc_pwm_t *pwm);
```

Converts an αβ voltage reference to three-phase PWM duty cycles in [0, 1].

| Parameter | Description                  |
|-----------|------------------------------|
| `ab`      | αβ voltage reference [V]     |
| `vbus`    | DC bus voltage [V]           |
| `pwm`     | Output duty cycles for U,V,W |

---

## 6. Delay Utilities

**Header:** `include/delay.h`  
**Source:** `src/delay.c`

```c
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
```

Blocking delays.  On POSIX systems these call `nanosleep()`; on bare-metal
replace with hardware timer loops.

> **Porting note:** Never call delay functions inside ISRs.
