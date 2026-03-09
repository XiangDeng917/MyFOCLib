/**
 * @file delay.h
 * @brief Portable busy-wait delay utilities.
 *
 * These functions provide simple blocking delays.  On bare-metal targets
 * they busy-loop; on hosted systems they delegate to the OS sleep API.
 * The implementation in delay.c must be ported for each target MCU by
 * adjusting the loop count or substituting a hardware timer.
 *
 * @note Avoid using these functions inside ISRs or time-critical loops.
 *       For production code, prefer hardware timer-based non-blocking delays.
 */

#ifndef DELAY_H
#define DELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/**
 * @brief Block for approximately @p ms milliseconds.
 *
 * @param[in] ms  Delay duration in milliseconds.
 */
void delay_ms(uint32_t ms);

/**
 * @brief Block for approximately @p us microseconds.
 *
 * @param[in] us  Delay duration in microseconds.
 */
void delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* DELAY_H */
