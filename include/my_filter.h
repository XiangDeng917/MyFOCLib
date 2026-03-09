/**
 * @file my_filter.h
 * @brief First-order low-pass filter public API.
 *
 * Provides a lightweight, single-pole IIR (infinite-impulse-response)
 * low-pass filter intended for smoothing ADC readings, speed estimates
 * and other sensor signals in embedded FOC applications.
 *
 * Difference equation:
 * @code
 *   y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 * @endcode
 * where @p alpha ∈ (0, 1].  A smaller @p alpha produces heavier filtering
 * (slower response); alpha = 1 passes the input unchanged.
 */

#ifndef MY_FILTER_H
#define MY_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Data types
 * ---------------------------------------------------------------------- */

/**
 * @brief Low-pass filter state and configuration.
 */
typedef struct {
    float alpha;    /**< Smoothing coefficient ∈ (0, 1] */
    float output;   /**< Current filter output (previous sample) */
} lp_filter_t;

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/**
 * @brief Initialise a low-pass filter instance.
 *
 * @param[out] f      Pointer to the filter instance.
 * @param[in]  alpha  Smoothing coefficient ∈ (0, 1].
 *                    Typical value for 1 kHz sampling / 10 Hz cutoff: ~0.06.
 * @param[in]  initial_value  Initial output value (use 0.0f if unknown).
 */
void filter_init(lp_filter_t *f, float alpha, float initial_value);

/**
 * @brief Process one input sample through the filter.
 *
 * @param[in,out] f     Pointer to the filter instance.
 * @param[in]     input Raw input sample.
 * @return Filtered output value.
 */
float filter_update(lp_filter_t *f, float input);

/**
 * @brief Reset the filter output to a given value.
 *
 * Useful for bumpless transfer after a mode change.
 *
 * @param[in,out] f     Pointer to the filter instance.
 * @param[in]     value Value to set as the new filter output.
 */
void filter_reset(lp_filter_t *f, float value);

#ifdef __cplusplus
}
#endif

#endif /* MY_FILTER_H */
