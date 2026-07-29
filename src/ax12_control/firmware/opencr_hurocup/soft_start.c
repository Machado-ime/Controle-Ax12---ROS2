/* Porte 1:1 do original — lógica pura, nenhuma mudança. */
#include "soft_start.h"
#include "config.h"   /* SOFT_START_* knobs + BOOT_MOVING_SPEED racional */

static soft_start_t g_soft_start;

soft_start_t *soft_start_instance(void)
{
    return &g_soft_start;
}

/* Velocidade-alvo do servo i: durante a convergência, servos sem speed
 * explícito (commanded == 0 = "max") ficam no cap; fora dela, o comando
 * vale literalmente (0 = max — contrato do MSG_JOINT_CMD). Speeds
 * explícitos (> 0) NUNCA são sobrescritos pelo cap. */
static uint16_t target_speed(const soft_start_t *s, int i)
{
    if (s->converging && s->commanded[i] == 0U) {
        return s->cap;
    }
    return s->commanded[i];
}

/* Marca pendência para todo servo cujo alvo difere do último escrito. */
static void refresh_pending(soft_start_t *s)
{
    for (int i = 0; i < 18; i++) {
        if (target_speed(s, i) != s->last_written[i]) {
            s->pending |= (1UL << i);
        }
    }
}

void soft_start_init(soft_start_t *s, uint16_t cap_value, uint32_t now_ms)
{
    s->converging = 1U;
    s->engaged_ms = now_ms;
    s->cap        = cap_value;
    s->pending    = 0U;
    for (int i = 0; i < 18; i++) {
        s->last_written[i] = cap_value;  /* boot já escreveu o cap físico */
        s->commanded[i]    = 0U;         /* default do stack: 0 = max     */
    }
}

void soft_start_on_joint_cmd(soft_start_t *s,
                             const uint16_t prev_goals[18],
                             const uint16_t new_goals[18],
                             uint32_t mask,
                             const uint16_t speeds[18],
                             uint32_t now_ms)
{
    uint8_t jump = 0U;

    for (int i = 0; i < 18; i++) {
        if ((mask & (1UL << i)) == 0U) {
            continue;  /* servo não endereçado: nem speed nem salto */
        }
        s->commanded[i] = (speeds[i] > 1023U) ? 1023U : speeds[i];

        const int32_t delta = (int32_t)new_goals[i] - (int32_t)prev_goals[i];
        if (delta > (int32_t)SOFT_START_JUMP_TICKS ||
            delta < -(int32_t)SOFT_START_JUMP_TICKS) {
            jump = 1U;
        }
    }

    if (jump) {
        s->converging = 1U;
        s->engaged_ms = now_ms;
    }

    refresh_pending(s);
}

void soft_start_tick(soft_start_t *s,
                     const uint16_t goals[18],
                     const uint16_t positions[18],
                     const uint8_t present[18],
                     uint32_t now_ms)
{
    if (s->converging) {
        uint8_t valid = 0U;
        uint8_t all_close = 1U;
        for (int i = 0; i < 18; i++) {
            if (!present[i] || positions[i] == 0xFFFFU) {
                continue;  /* ausente ou leitura inválida não bloqueia */
            }
            valid++;
            const int32_t err = (int32_t)goals[i] - (int32_t)positions[i];
            if (err > (int32_t)SOFT_START_DONE_TICKS ||
                err < -(int32_t)SOFT_START_DONE_TICKS) {
                all_close = 0U;
            }
        }
        const uint8_t timed_out =
            (uint32_t)(now_ms - s->engaged_ms) >= SOFT_START_TIMEOUT_MS;
        if ((valid > 0U && all_close) || timed_out) {
            s->converging = 0U;   /* libera: alvo volta a ser o comandado */
        }
    }

    refresh_pending(s);
}

uint8_t soft_start_flush(soft_start_t *s,
                         const uint8_t present[18],
                         uint8_t max_writes,
                         uint8_t (*write_speed)(uint8_t id, uint16_t spd))
{
    uint8_t written = 0U;

    for (int i = 0; i < 18 && written < max_writes; i++) {
        if ((s->pending & (1UL << i)) == 0U) {
            continue;
        }
        if (!present[i]) {
            s->pending &= ~(1UL << i);   /* ausente: nunca escrever */
            continue;
        }
        const uint16_t t = target_speed(s, i);
        if (t != s->last_written[i]) {
            (void)write_speed((uint8_t)(i + 1), t);
            s->last_written[i] = t;
            written++;
        }
        s->pending &= ~(1UL << i);
    }

    return written;
}
