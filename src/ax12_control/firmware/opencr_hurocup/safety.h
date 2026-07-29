#ifndef SAFETY_H
#define SAFETY_H

/* Porte 1:1 do original — lógica pura, nenhuma mudança. */

#include <stdint.h>

/* SystemStatus.error_flags bits:
 * 0x01: MODE_JOINT_STREAM watchdog expired.
 * 0x02: At least one present servo reports temperature >= 65 C.
 * 0x04: At least one expected servo ID is missing from the presence map.
 */
enum {
    SAFETY_BIT_STREAM_WATCHDOG = 0x01U,
    SAFETY_BIT_OVERTEMP        = 0x02U,
    SAFETY_BIT_SERVO_MISSING   = 0x04U
};

typedef struct {
    uint16_t error_flags;
    uint8_t  max_temp_c;
} safety_state_t;

void safety_init(safety_state_t *state);
void safety_update(safety_state_t *state,
                   const uint8_t temps[18],
                   const uint8_t servo_present[18],
                   uint8_t stream_watchdog_tripped);

#endif /* SAFETY_H */
