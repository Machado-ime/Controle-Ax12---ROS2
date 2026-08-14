/* OPENCR: reescrito — ver systick.h. */
#include <Arduino.h>
#include "systick.h"

volatile uint32_t systick_ms = 0;

extern "C" uint32_t get_ticks(void)
{
    return millis();
}

extern "C" void delay_ms(uint32_t ms)
{
    delay(ms);
}
