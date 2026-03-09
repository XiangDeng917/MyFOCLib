# FOC Library Usage Guide

## 1. Build On Host (Smoke Test)

```powershell
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

## 2. Add To Your MCU Project

1. Copy `foc.c`, `foc.h`, and all files under `src/`.
2. Implement hardware hooks declared in `foc.h`:
	- `setPWM(float duty_a, float duty_b, float duty_c)`
	- `getRawCount()`
	- `getSysTickVal()` and `getSysTickLoad()`
	- `motor_enable_fn()` and `motor_disable_fn()`
3. Configure motor/sensor parameters in `BLDCMotor_s` and `MagneticSensor_s`.
4. Call `FOC_init(&foc_ctl, &motor, &sensor)` once at startup.
5. In control loop:
	- call `FOC_closedLoop(&foc_ctl, &motor)`
	- call `FOC_move(target, &foc_ctl, &motor)`

## 3. Typical Runtime Sequence

1. `Motor_init(&motor)`
2. `FOC_init(&foc_ctl, &motor, &sensor)`
3. Periodic loop:
	- update control mode/target
	- run `FOC_closedLoop`
	- run `FOC_move`

## 4. Porting Checklist

1. Confirm encoder direction and CPR.
2. Confirm PWM polarity and dead-time settings.
3. Verify `SYSTICK_FREQUENCY` matches your clock.
4. Tune `pid_velocity` then `pid_angle`.
5. Validate current limits and `voltage_limit` before load tests.

## 5. Common Pitfalls

1. Function signatures must match headers exactly.
2. Avoid duplicating global definitions in headers.
3. Keep platform code out of generic control logic.
