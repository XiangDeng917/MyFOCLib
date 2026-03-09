#ifndef MY_FOC_H
#define MY_FOC_H

#include <stdint.h>
#include <math.h>

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define i8 int8_t
#define i16 int16_t
#define i32 int32_t
#define f32 float
#define b8 u8
#define b16 u16
#define b32 u32

#include "BLDCMotor.h"
#include "MagneticSensor.h"
#include "my_filter.h"
#include "pid.h"

b8 motor_enable_fn(void);
b8 motor_disable_fn(void);

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

// encoder
#define M_AS5600 1
#define M_TLE5012B 0
// ...

#define M_Enable motor_enable_fn()   // Active-high enable
#define M_Disable motor_disable_fn() // Active-low disable

/******************************************************************************/
// Small math helpers
#define _sign(a) (((a) < 0) ? -1 : ((a) > 0))
#define _round(x) ((x) >= 0 ? (i32)((x) + 0.5) : (i32)((x) - 0.5))
#define _constrain(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))
#define _sqrt(a) (_sqrtApprox(a))
#define _isset(a) ((a) != (NOT_SET))

// utility defines
#define _2_SQRT3 1.15470053838 // 2/sqrt(3) for voltage calculations
#define _SQRT3 1.73205080757   // sqrt(3) for voltage calculations
#define _1_SQRT3 0.57735026919 // 1/sqrt(3) for voltage calculations
#define _SQRT3_2 0.86602540378
#define _SQRT2 1.41421356237
#define _120_D2R 2.09439510239 // 120 degrees to radians
#define _PI 3.14159265359
#define _PI_2 1.57079632679
#define _PI_3 1.0471975512
#define _2PI 6.28318530718
#define _3PI_2 4.71238898038
#define _PI_6 0.52359877559

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

// User-provided platform hooks

// systick
u32 getSysTickVal(void);
u32 getSysTickLoad(void);

// Hardware interface
void setPWM(float duty_a, float duty_b, float duty_c);
u16 getRawCount();

#endif