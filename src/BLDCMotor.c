
#include "foc.h"

extern f32 target;

BLDCMotor_s bldc_2804 = {
	.pole_pairs = 7,
	.voltage_power_supply = 12,
	.voltage_sensor_align = 2.5,
	.voltage_limit = 6,
};

FOC_Controller_s foc_2804 = {
	.velocity_limit = 20,
	.sensor_direction = UNKNOWN,
	.target = 2,
	.torque_type = Type_voltage,
	.motion_type = Type_velocity,
	.open_loop_timestamp = 0,
};

void Motor_init(BLDCMotor_s *motor);
i32 alignSensor(BLDCMotor_s *motor, FOC_Controller_s *foc_ctl, MagneticSensor_s *sensor);
void setPhaseVoltage(f32 Uq, f32 Ud, f32 angle_electrical, BLDCMotor_s *motor);

void Motor_init(BLDCMotor_s *motor)
{
	printf("MOT: Init\r\n");
	//	new_voltage_limit = current_limit * phase_resistance;
	//	voltage_limit = new_voltage_limit < voltage_limit ? new_voltage_limit : voltage_limit;
	if (motor->voltage_sensor_align > motor->voltage_limit)
		motor->voltage_sensor_align = motor->voltage_limit;
	M_Enable;
	printf("MOT: Enable driver.\r\n");
}

i32 alignSensor(BLDCMotor_s *motor, FOC_Controller_s *foc_ctl, MagneticSensor_s *sensor)
{
	long i;
	f32 angle;
	f32 mid_angle = 0.0f, end_angle = 0.0f;
	f32 delta_angle;
	f32 signed_diff;
	f32 sample_sum;
	i32 sample_count = 8;
	i32 estimated_pp;

	printf("MOT: Align sensor.\r\n");

	// find natural direction
	// move one electrical revolution forward
	for (i = 0; i <= 500; i++)
	{
		angle = _3PI_2 + _2PI * i / 500.0;
		setPhaseVoltage(motor->voltage_sensor_align, 0, angle, motor);
		delay_ms(2);
	}
	sample_sum = 0.0f;
	for (i = 0; i < sample_count; i++)
	{
		sample_sum += getAngle(sensor);
		delay_ms(1);
	}
	mid_angle = sample_sum / sample_count;

	for (i = 500; i >= 0; i--)
	{
		angle = _3PI_2 + _2PI * i / 500.0;
		setPhaseVoltage(motor->voltage_sensor_align, 0, angle, motor);
		delay_ms(2);
	}
	sample_sum = 0.0f;
	for (i = 0; i < sample_count; i++)
	{
		sample_sum += getAngle(sensor);
		delay_ms(1);
	}
	end_angle = sample_sum / sample_count;

	setPhaseVoltage(0, 0, 0, motor);
	delay_ms(200);

	printf("mid_angle=%.4f\r\n", mid_angle);
	printf("end_angle=%.4f\r\n", end_angle);

	// Use shortest angular distance to avoid 0/2PI wrap-around issues.
	signed_diff = mid_angle - end_angle;
	if (signed_diff > _PI)
		signed_diff -= _2PI;
	else if (signed_diff < -_PI)
		signed_diff += _2PI;

	delta_angle = fabs(signed_diff);
	if (delta_angle < 0.02f)
	{
		printf("MOT: Failed to notice movement loop222.\r\n");
		M_Disable;
		return 0;
	}
	else if (signed_diff < 0.0f)
	{
		printf("MOT: sensor_direction==CCW\r\n");
		foc_ctl->sensor_direction = CCW;
	}
	else
	{
		printf("MOT: sensor_direction==CW\r\n");
		foc_ctl->sensor_direction = CW;
	}

	printf("MOT: PP check: ");
	if (fabs(delta_angle * motor->pole_pairs - _2PI) > 0.5) // 0.5 is arbitrary number it can be lower or higher!
	{
		printf("fail - estimated pp:");
		estimated_pp = (i32)(_2PI / delta_angle + 0.5f);
		if (estimated_pp < 1)
			estimated_pp = 1;
		motor->pole_pairs = estimated_pp;
		printf("%d\r\n", motor->pole_pairs);
	}
	else
		printf("OK!\r\n");

	setPhaseVoltage(motor->voltage_sensor_align, 0, _3PI_2, motor);
	delay_ms(700);
	sample_sum = 0.0f;
	for (i = 0; i < sample_count; i++)
	{
		sample_sum += getAngle(sensor);
		delay_ms(1);
	}
	foc_ctl->zero_electric_angle = _normalizeAngle(_electricalAngle(foc_ctl->sensor_direction * (sample_sum / sample_count), motor->pole_pairs));
	delay_ms(20);
	printf("MOT: Zero elec. angle:");
	printf("%.4f\r\n", foc_ctl->zero_electric_angle);

	setPhaseVoltage(0, 0, 0, motor);
	delay_ms(200);
	printf("calibrated pp: %d\r\n", motor->pole_pairs);

	return 1;
}

/*
 * SVPWM from dq voltage command (simplified):
 * 1) Voltage magnitude normalization:
 *    Uout = sqrt(Ud^2 + Uq^2) / Vdc, or Uout = Uq / Vdc when Ud = 0.
 * 2) Electrical angle used by modulation:
 *    theta = normalize(angle_electrical + atan2(Uq, Ud)), or theta = normalize(angle_electrical + PI/2) when Ud = 0.
 * 3) Sector and vector times (Ts normalized to 1):
 *    sector = floor(theta / (PI/3)) + 1
 *    T1 = sqrt(3) * sin(sector*PI/3 - theta) * Uout
 *    T2 = sqrt(3) * sin(theta - (sector-1)*PI/3) * Uout
 *    T0 = 1 - T1 - T2
 * 4) Ta/Tb/Tc are obtained from sector switching table, then mapped to PWM compare values.
 */
void setPhaseVoltage(f32 Uq, f32 Ud, f32 angle_electrical, BLDCMotor_s *motor)
{
	f32 Uout;
	f32 theta;
	f32 T0, T1, T2;
	f32 Ta, Tb, Tc;
	f32 t0_half;
	f32 sector_angle;
	u32 sector;

	if (Ud != 0.0f)
	{
		Uout = _sqrt(Ud * Ud + Uq * Uq) / motor->voltage_power_supply;
		theta = angle_electrical + atan2(Uq, Ud);
	}
	else
	{
		Uout = Uq / motor->voltage_power_supply;
		theta = angle_electrical + _PI_2;
	}

	// Keep modulation magnitude positive and move sign into angle.
	if (Uout < 0.0f)
	{
		Uout = -Uout;
		theta += _PI;
	}

	if (Uout > 0.577f)
		Uout = 0.577f;

	theta = _normalizeAngle(theta);
	sector = (uint32_t)(theta / _PI_3) + 1u;
	if (sector > 6u)
		sector = 6u;

	sector_angle = sector * _PI_3;
	T1 = _SQRT3 * _sin(sector_angle - theta) * Uout;
	T2 = _SQRT3 * _sin(theta - (sector_angle - _PI_3)) * Uout;
	T0 = 1.0f - T1 - T2;
	t0_half = 0.5f * T0;

	switch (sector)
	{
	case 1:
		Ta = T1 + T2 + t0_half;
		Tb = T2 + t0_half;
		Tc = t0_half;
		break;
	case 2:
		Ta = T1 + t0_half;
		Tb = T1 + T2 + t0_half;
		Tc = t0_half;
		break;
	case 3:
		Ta = t0_half;
		Tb = T1 + T2 + t0_half;
		Tc = T2 + t0_half;
		break;
	case 4:
		Ta = t0_half;
		Tb = T1 + t0_half;
		Tc = T1 + T2 + t0_half;
		break;
	case 5:
		Ta = T2 + t0_half;
		Tb = t0_half;
		Tc = T1 + T2 + t0_half;
		break;
	default: // sector 6 and fallback
		Ta = T1 + T2 + t0_half;
		Tb = t0_half;
		Tc = T1 + t0_half;
		break;
	}

	// Clamp duty to valid timer compare range.
	if (Ta < 0.0f)
		Ta = 0.0f;
	else if (Ta > 1.0f)
		Ta = 1.0f;
	if (Tb < 0.0f)
		Tb = 0.0f;
	else if (Tb > 1.0f)
		Tb = 1.0f;
	if (Tc < 0.0f)
		Tc = 0.0f;
	else if (Tc > 1.0f)
		Tc = 1.0f;

	setPWM(Ta, Tb, Tc);
}
