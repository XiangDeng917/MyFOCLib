/**
 * @file delay.c
 * @brief Portable delay implementation.
 *
 * On hosted (POSIX) systems this implementation uses nanosleep().
 * For bare-metal MCUs, replace the body of delay_ms() and delay_us()
 * with a hardware-timer-based busy-wait loop calibrated for your CPU
 * frequency (e.g. using DWT_CYCCNT on Cortex-M).
 */

/* _POSIX_C_SOURCE must be defined before any system header to expose nanosleep(). */
#if defined(__linux__) || defined(__APPLE__) || defined(__unix__)
#   define DELAY_POSIX 1
#   ifndef _POSIX_C_SOURCE
#       define _POSIX_C_SOURCE 199309L
#   endif
#   include <time.h>
#endif

#include "delay.h"

/* -------------------------------------------------------------------------
 * Implementation
 * ---------------------------------------------------------------------- */

#if defined(DELAY_POSIX)

void delay_ms(uint32_t ms)
{
    struct timespec ts;
    ts.tv_sec  = (time_t)(ms / 1000U);
    ts.tv_nsec = (long)((ms % 1000U) * 1000000L);
    nanosleep(&ts, 0);
}

void delay_us(uint32_t us)
{
    struct timespec ts;
    ts.tv_sec  = (time_t)(us / 1000000U);
    ts.tv_nsec = (long)((us % 1000000U) * 1000L);
    nanosleep(&ts, 0);
}

#else
/* -----------------------------------------------------------------------
 * Bare-metal stub — calibrate LOOPS_PER_MS to your MCU clock speed.
 * --------------------------------------------------------------------- */
#   define LOOPS_PER_MS  4000U   /* approx. for 48 MHz Cortex-M0 */

void delay_ms(uint32_t ms)
{
    volatile uint32_t count = (uint32_t)ms * LOOPS_PER_MS;
    while (count--) {
        __asm volatile("nop");
    }
}

void delay_us(uint32_t us)
{
    volatile uint32_t count = us * (LOOPS_PER_MS / 1000U);
    while (count--) {
        __asm volatile("nop");
    }
}
#endif /* DELAY_POSIX */
