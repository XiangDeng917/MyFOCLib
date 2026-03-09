

#include "pid.h"

static void PID_vel_init(void);
static void PID_angle_init(void);

PID_s pid_velocity = {
	.kp = 0.1f,
	.ki = 2.0f,
	.kd = 0.0f,
	.lowpass_filter_alpha = 1.0f,
	.integral_limit = 100.0f,
	.output_limit = 100.0f
};
PID_s pid_angle = {
	.kp = 10.0f,
	.ki = 0.0f,
	.kd = 0.5f,
	.lowpass_filter_alpha = 1.0f,
	.integral_limit = 100.0f,
	.output_limit = 100.0f
};

void PID_init(void)
{
	pid_velocity.timestamp =getSysTickVal();
	pid_angle.timestamp =getSysTickVal();
}

f32 PID_update(PID_s *pid, f32 feedback)
{
	f32 error, output;
	f32 p_out, i_out, d_out;
	f32 dt;
	u32 now_us;
	now_us = getSysTickVal();

	if (now_us < pid->timestamp)
		dt = (f32)(pid->timestamp - now_us) / SYSTICK_FREQUENCY; // 当Systick频率为72MHz/8=9MHz时，所以有/SYSTICK_FREQUENCY
	else
		dt = (f32)(0xFFFFFF - now_us + pid->timestamp) / SYSTICK_FREQUENCY;

	if (dt == 0 || dt > 0.5)
		dt = 1e-3f;

	error = pid->target - feedback;
	// p 	
	p_out = pid->kp * error;
	// i
	pid->integral += error * dt;
	i_out = _constrain(pid->ki * pid->integral, 0, pid->integral_limit);
	// d
	f32 derivetive_raw = (error - pid->error_prev) / dt;
	f32 derivative_filtered =lowpassFilter_fn(derivetive_raw, &pid->derivative_prev, pid->lowpass_filter_alpha);
	d_out = pid->kd * derivative_filtered;
	//output 
	output = p_out + i_out + d_out;
	output = _constrain(output, -pid->output_limit, pid->output_limit);

	pid->error_prev=error;
	pid->timestamp = now_us;

	return output;
}
