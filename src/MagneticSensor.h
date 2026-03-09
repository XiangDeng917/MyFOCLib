#ifndef MAGNETICSENSOR_LIB_H
#define MAGNETICSENSOR_LIB_H

#include "foc.h"

typedef struct
{
    i32 cpr;                     // Counts per revolution.
    f32 full_rotation_offset;    // Multi-turn angle offset.
    i32 angle_prev_i32;          // Previous raw angle count.
    u32 velocity_calc_timestamp; // Timestamp for velocity update.
    f32 angle_prev_f32;          // Previous angle in radians.
} MagneticSensor_s;

f32 getAngle(MagneticSensor_s *sensor);
f32 getVelocity(MagneticSensor_s *sensor);

#endif
