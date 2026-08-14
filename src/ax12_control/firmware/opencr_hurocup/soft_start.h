#ifndef SOFT_START_H
#define SOFT_START_H

#include <stdint.h>

/*
 * soft_start — supervisor do Moving Speed dos AX-12A. Porte 1:1 do
 * original (módulo puro, sem acesso a hardware — ver soft_start.c).
 *
 * Motivo (bug 2026-07-11 do projeto original): o boot escreve
 * BOOT_MOVING_SPEED (cap suave) em todos os servos, e um handler ingênuo
 * de MSG_JOINT_CMD só escreveria Moving Speed quando spd > 0. Como todo
 * o stack usa a convenção "0 = velocidade máxima", o cap de boot NUNCA
 * seria removido sem este módulo.
 *
 *   - cache por servo do último Moving Speed escrito ("0 = max" agora é
 *     honrado com UMA escrita, não 18 por frame);
 *   - o cap de boot continua ativo até os servos CONVERGIREM no primeiro
 *     alvo comandado (|goal−pos| <= SOFT_START_DONE_TICKS) ou até o
 *     timeout — aí é liberado para a velocidade comandada (0 = max);
 *   - qualquer SALTO de pose entre frames consecutivos do stream
 *     (> SOFT_START_JUMP_TICKS num servo endereçado) re-engata o cap.
 */

typedef struct {
    uint8_t  converging;        /* 1 = cap ativo, esperando convergência    */
    uint32_t engaged_ms;        /* quando o cap (re)entrou — p/ timeout     */
    uint16_t cap;               /* valor do cap (BOOT_MOVING_SPEED)         */
    uint16_t last_written[18];  /* último Moving Speed escrito por servo    */
    uint16_t commanded[18];     /* último speed do fio (0 = max)            */
    uint32_t pending;           /* bit i = servo i+1 precisa de escrita     */
} soft_start_t;

/* Singleton usado pelo .ino principal e command_handler.c no alvo. */
soft_start_t *soft_start_instance(void);

void soft_start_init(soft_start_t *s, uint16_t cap_value, uint32_t now_ms);

void soft_start_on_joint_cmd(soft_start_t *s,
                             const uint16_t prev_goals[18],
                             const uint16_t new_goals[18],
                             uint32_t mask,
                             const uint16_t speeds[18],
                             uint32_t now_ms);

void soft_start_tick(soft_start_t *s,
                     const uint16_t goals[18],
                     const uint16_t positions[18],
                     const uint8_t present[18],
                     uint32_t now_ms);

uint8_t soft_start_flush(soft_start_t *s,
                         const uint8_t present[18],
                         uint8_t max_writes,
                         uint8_t (*write_speed)(uint8_t id, uint16_t spd));

#endif /* SOFT_START_H */
