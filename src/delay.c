#include "delay.h"

void delay_us(u32 target) {
    u32 tcnt = 0;
    u32 told, tnow;
    u32 reload =getSysTickLoad();

    // 9 ticks/us for a 9MHz SysTick clock.
    u32 ticks_needed = target * 9;

    told =getSysTickVal();

    while (tcnt < ticks_needed) {
        tnow =getSysTickVal();
        if (tnow != told) {
            // SysTick is a down-counter.
            if (tnow < told) {
                tcnt += told - tnow;
            } else {
                // Handle counter wrap-around.
                tcnt += reload - tnow + told;
            }
            told = tnow;
        }
    }
}

void delay_ms(u32 target) {
    // Avoid overflow from target * 1000.
    while(target--) {
        delay_us(1000);
    }
}
