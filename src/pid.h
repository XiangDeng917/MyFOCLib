#ifndef PID_H
#define PID_H

#include "foc.h"

typedef struct
{
    u32 timestamp;

    f32 target;
    f32 integral;

    f32 kp, ki, kd;

    f32 lowpass_filter_alpha;

    f32 derivative_prev;
    f32 error_prev;

    f32 integral_limit;
    f32 output_limit;

} PID_s;

extern PID_s pid_velocity;
extern PID_s pid_angle;

void PID_init(void);
f32 PID_update(PID_s *pid, f32 feedback);

#endif
