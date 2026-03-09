#include "foc.h"

const u16 sine_array[200] = {0, 79, 158, 237, 316, 395, 473, 552, 631, 710, 789, 867, 946, 1024, 1103, 1181, 1260, 1338, 1416, 1494, 1572, 1650, 1728, 1806, 1883, 1961, 2038, 2115, 2192, 2269, 2346, 2423, 2499, 2575, 2652, 2728, 2804, 2879, 2955, 3030, 3105, 3180, 3255, 3329, 3404, 3478, 3552, 3625, 3699, 3772, 3845, 3918, 3990, 4063, 4135, 4206, 4278, 4349, 4420, 4491, 4561, 4631, 4701, 4770, 4840, 4909, 4977, 5046, 5113, 5181, 5249, 5316, 5382, 5449, 5515, 5580, 5646, 5711, 5775, 5839, 5903, 5967, 6030, 6093, 6155, 6217, 6279, 6340, 6401, 6461, 6521, 6581, 6640, 6699, 6758, 6815, 6873, 6930, 6987, 7043, 7099, 7154, 7209, 7264, 7318, 7371, 7424, 7477, 7529, 7581, 7632, 7683, 7733, 7783, 7832, 7881, 7930, 7977, 8025, 8072, 8118, 8164, 8209, 8254, 8298, 8342, 8385, 8428, 8470, 8512, 8553, 8594, 8634, 8673, 8712, 8751, 8789, 8826, 8863, 8899, 8935, 8970, 9005, 9039, 9072, 9105, 9138, 9169, 9201, 9231, 9261, 9291, 9320, 9348, 9376, 9403, 9429, 9455, 9481, 9506, 9530, 9554, 9577, 9599, 9621, 9642, 9663, 9683, 9702, 9721, 9739, 9757, 9774, 9790, 9806, 9821, 9836, 9850, 9863, 9876, 9888, 9899, 9910, 9920, 9930, 9939, 9947, 9955, 9962, 9969, 9975, 9980, 9985, 9989, 9992, 9995, 9997, 9999, 10000, 10000};

// math
f32 _sin(f32 a)
{
    if (a < _PI_2)
    {
        return 0.0001 * sine_array[_round(126.6873 * a)]; // int array optimized
    }
    else if (a < _PI)
    {
        return 0.0001 * sine_array[398 - _round(126.6873 * a)]; // int array optimized
    }
    else if (a < _3PI_2)
    {
        return -0.0001 * sine_array[-398 + _round(126.6873 * a)]; // int array optimized
    }
    else
    {
        return -0.0001 * sine_array[796 - _round(126.6873 * a)]; // int array optimized
    }
}
f32 _cos(f32 a)
{
    f32 a_sin = a + _PI_2;
    a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
    return _sin(a_sin);
}
// normalizing radian angle to [0,2PI]
f32 _normalizeAngle(f32 angle)
{
    f32 a = fmodf(angle, _2PI);
    return a >= 0 ? a : (a + _2PI);
}
// Electrical angle calculation
f32 _electricalAngle(f32 shaft_angle, int pole_pairs)
{
    return (shaft_angle * pole_pairs);
}
// square root approximation function using
f32 _sqrtApprox(f32 number)
{ // low in fat
    i32 i;
    f32 y;
    // f32 x;
    // const f32 f = 1.5F; // better precision
    // x = number * 0.5F;
    y = number;
    i = *(i32 *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(f32 *)&i;
    // y = y * ( f - ( x * y * y ) ); // better precision
    return number * y;
}

// motor
// shaft angle calculation
f32 shaftAngle(FOC_Controller_s *foc_ctl)
{
    // if no sensor linked return previous value ( for open loop )
    // if(!sensor) return shaft_angle;
    return foc_ctl->sensor_direction * getAngle(foc_ctl) -foc_ctl->sensor_offset;
}

// shaft velocity calculation
f32 shaftVelocity(FOC_Controller_s *foc_ctl)
{
    // if no sensor linked return previous value ( for open loop )
    // if(!sensor) return shaft_velocity;
    return foc_ctl->sensor_direction *lowpassFilter_fn(getVelocity(foc_ctl), &foc_ctl->shaft_velocity_prev,0.9f);
}

f32 electricalAngle(FOC_Controller_s *foc_ctl, BLDCMotor_s* bldc_motor)
{
    return _normalizeAngle((foc_ctl->shaft_angle + foc_ctl->sensor_offset) *bldc_motor->pole_pairs - foc_ctl->zero_electric_angle);
}

// foc main

void FOC_move(f32 new_target, FOC_Controller_s *foc_ctl,BLDCMotor_s *motor)
{
	foc_ctl->shaft_velocity = shaftVelocity(foc_ctl);

	switch (foc_ctl->motion_type)
	{
	case Type_torque:
		if (foc_ctl->torque_type == Type_voltage)
			foc_ctl->voltage.q = new_target; // if voltage torque control
		else
			foc_ctl->current_target = new_target; // if current/foc_current torque control
		break;
	case Type_angle:
		// angle set point
		foc_ctl->shaft_angle_target = new_target;
		// calculate velocity set point
		pid_angle.target = foc_ctl->shaft_angle_target;
		foc_ctl->shaft_velocity_target = PID_update(&pid_angle, foc_ctl->shaft_angle);
		// calculate the torque command
		pid_velocity.target = foc_ctl->shaft_velocity_target;
		foc_ctl->current_target = PID_update(&pid_velocity, foc_ctl->shaft_velocity);
		// if torque controlled through voltage
		if (foc_ctl->torque_type == Type_voltage)
		{
			foc_ctl->voltage.q = foc_ctl->current_target;
			foc_ctl->voltage.d = 0;
		}
		break;
	case Type_velocity:
		// velocity set point
		foc_ctl->shaft_velocity_target = new_target;
		// calculate the torque command
		pid_velocity.target = foc_ctl->shaft_velocity_target;
		foc_ctl->current_target = PID_update(&pid_velocity, foc_ctl->shaft_velocity);
		// if torque controlled through voltage control
		if (foc_ctl->torque_type == Type_voltage)
		{
			foc_ctl->voltage.q = foc_ctl->current_target; // use voltage if phase-resistance not provided
			foc_ctl->voltage.d = 0;
		}
		break;
	case Type_velocity_openloop:
		// velocity control in open loop
		foc_ctl->shaft_velocity_target = new_target;
		foc_ctl->voltage.q = FOC_velocityOpenloop(foc_ctl->shaft_velocity_target, foc_ctl, motor); // returns the voltage that is set to the motor
		foc_ctl->voltage.d = 0;
		break;
	case Type_angle_openloop:
		// angle control in open loop
		foc_ctl->shaft_angle_target = new_target;
		foc_ctl->voltage.q = FOC_angleOpenloop(foc_ctl->shaft_angle_target, foc_ctl, motor); // returns the voltage that is set to the motor
		foc_ctl->voltage.d = 0;
		break;
	}
}

f32 FOC_velocityOpenloop(f32 target_velocity, FOC_Controller_s *foc_ctl, BLDCMotor_s *motor)
{
	u32 now_us;
	f32 Ts, Uq;

	now_us = getSysTickVal(); //_micros();
	if (now_us < foc_ctl->open_loop_timestamp)
		Ts = (f32)(foc_ctl->open_loop_timestamp - now_us) / SYSTICK_FREQUENCY;
	else
		Ts = (f32)(0xFFFFFF - now_us + foc_ctl->open_loop_timestamp) / SYSTICK_FREQUENCY;
	foc_ctl->open_loop_timestamp = now_us; // save timestamp for next call
	// quick fix for strange cases (micros overflow)
	if (Ts == 0 || Ts > 0.5)
		Ts = 1e-3;

	// calculate the necessary angle to achieve target velocity
	foc_ctl->shaft_angle = _normalizeAngle(foc_ctl->shaft_angle + target_velocity * Ts);

	Uq = motor->voltage_limit;
	// set the maximal allowed voltage (voltage_limit) with the necessary angle
	setPhaseVoltage(Uq, 0, _electricalAngle(foc_ctl->shaft_angle, motor->pole_pairs), motor);

	return Uq;
}

f32 FOC_angleOpenloop(f32 target_angle, FOC_Controller_s *foc_ctl, BLDCMotor_s *motor)
{
	u32 now_us;
	f32 Ts, Uq;

	now_us = getSysTickVal(); //_micros();
	if (now_us < foc_ctl->open_loop_timestamp)
		Ts = (f32)(foc_ctl->open_loop_timestamp - now_us) / SYSTICK_FREQUENCY;
	else
		Ts = (f32)(0xFFFFFF - now_us + foc_ctl->open_loop_timestamp) / SYSTICK_FREQUENCY;
	foc_ctl->open_loop_timestamp = now_us; // save timestamp for next call
	// quick fix for strange cases (micros overflow)
	if (Ts == 0 || Ts > 0.5)
		Ts = 1e-3;

	// calculate the necessary angle to move from current position towards target angle
	// with maximal velocity (velocity_limit)
	if (fabs(target_angle - foc_ctl->shaft_angle) > foc_ctl->velocity_limit * Ts)
	{
		foc_ctl->shaft_angle += _sign(target_angle - foc_ctl->shaft_angle) * foc_ctl->velocity_limit * Ts;
		// shaft_velocity = velocity_limit;
	}
	else
	{
		foc_ctl->shaft_angle = target_angle;
		// shaft_velocity = 0;
	}

	Uq = motor->voltage_limit;
	// set the maximal allowed voltage (voltage_limit) with the necessary angle
	setPhaseVoltage(Uq, 0, _electricalAngle(foc_ctl->shaft_angle, motor->pole_pairs), motor);

	return Uq;
}

void FOC_closedLoop(FOC_Controller_s *foc_ctl,BLDCMotor_s *motor)
{
	if (foc_ctl->motion_type == Type_angle_openloop || foc_ctl->motion_type == Type_velocity_openloop)
		return;

	foc_ctl->shaft_angle = shaftAngle(foc_ctl);		   // shaft angle
	foc_ctl->electrical_angle = electricalAngle(foc_ctl, motor); // electrical angle - need shaftAngle to be called first

	switch (foc_ctl->torque_type)
	{
	case Type_voltage: // no need to do anything really
		break;
	case Type_dc_current:
		break;
	case Type_foc_current:
		break;
	default:
		printf("MOT: no torque control selected!");
		break;
	}
	// set the phase voltage - FOC heart function :)
	setPhaseVoltage(foc_ctl->voltage.q, foc_ctl->voltage.d, foc_ctl->electrical_angle, motor);
}

void FOC_init(FOC_Controller_s *foc_ctl, BLDCMotor_s *motor, MagneticSensor_s *sensor)
{
	alignSensor(motor, foc_ctl, sensor); // Detect sensor direction, zero offset, and pole pairs.

	// added the shaft_angle update
	sensor->angle_prev_f32 = getAngle(sensor); // getVelocity(),make sure velocity=0 after power on
	delay_ms(5);
	foc_ctl->shaft_velocity = shaftVelocity(foc_ctl); // Prime filtered velocity state once.
	delay_ms(5);
	foc_ctl->shaft_angle = shaftAngle(foc_ctl); // shaft angle
	if (foc_ctl->motion_type == Type_angle)
		foc_ctl->target = foc_ctl->shaft_angle; // Hold current position in angle mode at startup.

	delay_ms(200);
}


