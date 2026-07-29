#ifndef SYSCMD_H
#define SYSCMD_H

#include <stdint.h>

/* Módulo PURO (testável no host) — porte 1:1 do original: decisão do
 * SystemCommand (MSG_SYSTEM_CMD = 16). A EXECUÇÃO (reset / bootflag)
 * fica no .ino principal — este módulo não toca registrador. */

typedef enum {
    SYSCMD_NONE             = -1,
    SYSCMD_SOFT_RESET       = 0,
    SYSCMD_ENTER_BOOTLOADER = 1
} syscmd_t;

/* Payload '>B' (1 byte). len < 1, payload nulo ou comando desconhecido
 * devolvem SYSCMD_NONE; bytes extras são ignorados. */
syscmd_t syscmd_parse(const uint8_t *payload, uint8_t len);

#endif /* SYSCMD_H */
