/* Porte 1:1 do original — lógica pura, nenhuma mudança. */
#include "safety.h"

#define SAFETY_SERVO_COUNT 18U
#define SAFETY_OVERTEMP_C  65U

void safety_init(safety_state_t *state)
{
    state->error_flags = 0U;
    state->max_temp_c = 0U;
}

void safety_update(safety_state_t *state,
                   const uint8_t temps[18],
                   const uint8_t servo_present[18],
                   uint8_t stream_watchdog_tripped)
{
    uint16_t flags = 0U;
    uint8_t max_temp = 0U;

    for (uint8_t i = 0U; i < SAFETY_SERVO_COUNT; i++) {
        if (servo_present[i] == 0U) {
            flags |= SAFETY_BIT_SERVO_MISSING;
            continue;
        }

        if (temps[i] > max_temp) {
            max_temp = temps[i];
        }
    }

    if (max_temp >= SAFETY_OVERTEMP_C) {
        flags |= SAFETY_BIT_OVERTEMP;
    }

    if (stream_watchdog_tripped) {
        flags |= SAFETY_BIT_STREAM_WATCHDOG;
    }

    state->error_flags = flags;
    state->max_temp_c = max_temp;
}
