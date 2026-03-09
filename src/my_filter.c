#include "my_filter.h"

f32 lowpassFilter_fn(f32 input, f32 *prev_val, f32 alpha) //
{
	f32 output;
	output = alpha * input + (1.0f - alpha) * (*prev_val);
	*prev_val = output;
	return output;
}