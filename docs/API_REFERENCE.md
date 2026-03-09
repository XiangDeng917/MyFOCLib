# API Reference (Current)

## Core Types

1. `BLDCMotor_s`: pole pairs, bus voltage, alignment voltage, voltage limit.
2. `MagneticSensor_s`: CPR and angle/velocity tracking state.
3. `FOC_Controller_s`: control mode, dq targets, measured shaft states, offsets.

## Lifecycle

1. `void Motor_init(BLDCMotor_s *motor)`
2. `void FOC_init(FOC_Controller_s *foc_ctl, BLDCMotor_s *motor, MagneticSensor_s *sensor)`

## Runtime Control

1. `void FOC_closedLoop(FOC_Controller_s *foc_ctl, BLDCMotor_s *motor)`
2. `void FOC_move(f32 new_target, FOC_Controller_s *foc_ctl, BLDCMotor_s *motor)`

## Open Loop Helpers

1. `f32 FOC_velocityOpenloop(f32 target_velocity, FOC_Controller_s *foc_ctl, BLDCMotor_s *motor)`
2. `f32 FOC_angleOpenloop(f32 target_angle, FOC_Controller_s *foc_ctl, BLDCMotor_s *motor)`

## Sensor + Math

1. `f32 getAngle(MagneticSensor_s *sensor)`
2. `f32 getVelocity(MagneticSensor_s *sensor)`
3. `f32 _normalizeAngle(f32 angle)`
4. `f32 _electricalAngle(f32 shaft_angle, int pole_pairs)`

## Platform Hooks (User Must Implement)

1. `void setPWM(float duty_a, float duty_b, float duty_c)`
2. `u16 getRawCount()`
3. `u32 getSysTickVal()`
4. `u32 getSysTickLoad()`
5. `b8 motor_enable_fn(void)`
6. `b8 motor_disable_fn(void)`
