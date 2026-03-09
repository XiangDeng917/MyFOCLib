
#include "MagneticSensor.h"

MagneticSensor_s AS5600 =
	{
		.cpr = 4096,
		.full_rotation_offset = 0,
		.angle_prev_i32 = 0,
		.velocity_calc_timestamp = 0,
		.angle_prev_f32 = 0};

f32 getAngle(MagneticSensor_s *sensor)
{
	f32 angle_data, d_angle;

	angle_data = getRawCount();

	// tracking the number of rotations
	// in order to expand angle range form [0,2PI] to basically infinity
	d_angle = angle_data - sensor->angle_prev_i32;
	// if overflow happened track it as full rotation
	if (fabs(d_angle) > (0.8 * sensor->cpr))
		sensor->full_rotation_offset += d_angle > 0 ? -_2PI : _2PI;
	// save the current angle value for the next steps
	// in order to know if overflow happened
	sensor->angle_prev_i32 = angle_data;
	// return the full angle
	// (number of full rotations)*2PI + current sensor angle
	return (sensor->full_rotation_offset + (angle_data / (f32)sensor->cpr) * _2PI);
}
// Shaft velocity calculation
f32 getVelocity(MagneticSensor_s *sensor)
{
	u32 now_us;
	f32 Ts, angle_c, vel;

	// calculate sample time
	now_us = getSysTickVal(); //_micros();
	if (now_us < sensor->velocity_calc_timestamp)
		Ts = (f32)(sensor->velocity_calc_timestamp - now_us) / 9 * 1e-6;
	else
		Ts = (f32)(0xFFFFFF - now_us + sensor->velocity_calc_timestamp) / 9 * 1e-6; // systick默认配置72MHz/8=9MHz
	// quick fix for strange cases (micros overflow)
	if (Ts == 0 || Ts > 0.5)
		Ts = 1e-3;

	// current angle
	angle_c = getAngle(sensor);
	// velocity calculation
	vel = (angle_c - sensor->angle_prev_f32) / Ts;

	// save variables for future pass
	sensor->angle_prev_f32 = angle_c;
	sensor->velocity_calc_timestamp = now_us;
	return vel;
}
/******************************************************************************/
