#ifndef MY_TYPES_H
#define MY_TYPES_H

#include <stdint.h>

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

#endif