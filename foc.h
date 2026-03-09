#ifndef MY_FOC_H
#define MY_FOC_H

#include <stdint.h>
#include <math.h>
#include "myTypes.h"
#include "pid.h"

// encoder
#define M_AS5600 1
#define M_TLE5012B 0
// ...

/*
example(gd32e23):

b8 motor_enable_fn()
{
    gpio_bit_set(GPIOB,GPIO_PIN_15);
}
b8 motor_disable_fn()
{
    gpio_bit_reset(GPIOB,GPIO_PIN_15);
}
*/

#define M_Enable motor_enable_fn()   // Active-high enable
#define M_Disable motor_disable_fn() // Active-low disable

// systick
#define SYSTICK_FREQUENCY (9.0f * 1e6f) // 9MHz when using 72MHz clock with 8 divider

extern const u16 sine_array[200];

/************************************ Types ******************************************/
// dq current structure
typedef struct
{
    f32 d;
    f32 q;
} DQCurrent_s;

// dq voltage structs
typedef struct
{
    f32 d;
    f32 q;
} DQVoltage_s;

// phase current structure
typedef struct
{
    f32 a;
    f32 b;
    f32 c;
} PhaseCurrent_s;

typedef enum
{
    Type_torque,   //!< Torque control
    Type_velocity, //!< Velocity motion control
    Type_angle,    //!< Position/angle motion control
    Type_velocity_openloop,
    Type_angle_openloop
} MotionControlType;

/**
 *  Motiron control type
 */
typedef enum
{
    Type_voltage,    //!< Torque control using voltage
    Type_dc_current, //!< Torque control using DC current (one current magnitude)
    Type_foc_current //!< torque control using dq currents
} TorqueControlType;

typedef enum
{
    CW = 1,     // clockwise
    CCW = -1,   // counter clockwise
    UNKNOWN = 0 // not yet known or invalid state
} Direction;

typedef struct
{
    i32 cpr;                     // Counts per revolution.
    f32 full_rotation_offset;    // Multi-turn angle offset.
    i32 angle_prev_i32;          // Previous raw angle count.
    u32 velocity_calc_timestamp; // Timestamp for velocity update.
    f32 angle_prev_f32;          // Previous angle in radians.
} MagneticSensor_s;

typedef struct
{
	u16 pole_pairs;
	f32 voltage_power_supply;
	f32 voltage_sensor_align;
	f32 voltage_limit;
} BLDCMotor_s;

typedef struct
{
    f32 shaft_angle; //!< current motor angle
    f32 shaft_velocity;
    f32 shaft_velocity_prev;
    
    f32 shaft_angle_target;
    f32 shaft_velocity_target;
    f32 current_target;
    
    f32 electrical_angle;
    f32 zero_electric_angle;

    f32 sensor_offset;
    Direction sensor_direction;
    
    DQVoltage_s voltage;
    DQCurrent_s current;
    TorqueControlType torque_type;
    MotionControlType motion_type;
    f32 target;
    f32 velocity_limit;
    u32 open_loop_timestamp;
    
} FOC_Controller_s;

// Functions

// User-provided platform hooks
//motor
b8 motor_enable_fn(void);
b8 motor_disable_fn(void);
// systick
u32 getSysTickVal(void);
u32 getSysTickLoad(void);
// Hardware interface
void setPWM(float duty_a, float duty_b, float duty_c);
u16 getRawCount();

f32 _sin(f32 a);
f32 _cos(f32 a);
f32 _normalizeAngle(f32 angle);
f32 _electricalAngle(f32 shaft_angle, int pole_pairs);
f32 _sqrtApprox(f32 number);

// Motor-related helpers
f32 shaftAngle(FOC_Controller_s *foc_ctl);
f32 shaftVelocity(FOC_Controller_s *foc_ctl);
f32 electricalAngle(FOC_Controller_s *foc_ctl, BLDCMotor_s *bldc_motor);

// foc main
void FOC_move(f32 new_target, FOC_Controller_s *foc_ctl, BLDCMotor_s *motor);
f32 FOC_velocityOpenloop(f32 target_velocity, FOC_Controller_s *foc_ctl, BLDCMotor_s *motor);
f32 FOC_angleOpenloop(f32 target_angle, FOC_Controller_s *foc_ctl, BLDCMotor_s *motor);
void FOC_closedLoop(FOC_Controller_s *foc_ctl, BLDCMotor_s *motor);
void FOC_init(FOC_Controller_s *foc_ctl, BLDCMotor_s *motor, MagneticSensor_s *sensor);

//sensor
f32 getAngle(MagneticSensor_s *sensor);
f32 getVelocity(MagneticSensor_s *sensor);

//motor
void Motor_init(BLDCMotor_s *motor);
i32 alignSensor(BLDCMotor_s *motor, FOC_Controller_s *foc_ctl, MagneticSensor_s *sensor);
void setPhaseVoltage(f32 Uq, f32 Ud, f32 angle_electrical, BLDCMotor_s *motor);

#endif