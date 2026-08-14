/* Porte 1:1 do original — lógica pura, nenhuma mudança. */
#include "syscmd.h"

syscmd_t syscmd_parse(const uint8_t *payload, uint8_t len)
{
    if ((payload == (const uint8_t *)0) || (len < 1U)) {
        return SYSCMD_NONE;
    }
    switch (payload[0]) {
    case 0U:
        return SYSCMD_SOFT_RESET;
    case 1U:
        return SYSCMD_ENTER_BOOTLOADER;
    default:
        return SYSCMD_NONE;
    }
}
