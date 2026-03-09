#ifndef BLDCMotor_H
#define BLDCMotor_H

#include "foc.h"

typedef enum
{
    CW = 1,     // clockwise
    CCW = -1,   // counter clockwise
    UNKNOWN = 0 // not yet known or invalid state
} Direction;

typedef struct
{
	u16 pole_pairs;
	f32 voltage_power_supply;
	f32 voltage_sensor_align;
	f32 voltage_limit;
} BLDCMotor_s;

void Motor_init(BLDCMotor_s *motor);
i32 alignSensor(BLDCMotor_s *motor, FOC_Controller_s *foc_ctl, MagneticSensor_s *sensor);
void setPhaseVoltage(f32 Uq, f32 Ud, f32 angle_electrical, BLDCMotor_s *motor);

#endif
