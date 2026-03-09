/**
 * @file my_filter.c
 * @brief First-order IIR low-pass filter implementation.
 */

#include "my_filter.h"

void filter_init(lp_filter_t *f, float alpha, float initial_value)
{
    if (!f) return;

    f->alpha  = alpha;
    f->output = initial_value;
}

float filter_update(lp_filter_t *f, float input)
{
    if (!f) return input;

    f->output = f->alpha * input + (1.0f - f->alpha) * f->output;
    return f->output;
}

void filter_reset(lp_filter_t *f, float value)
{
    if (!f) return;

    f->output = value;
}
