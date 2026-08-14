#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <stdint.h>
#include <stdbool.h>

/* Porte 1:1 do original — lógica pura, nenhuma mudança.
 *
 * current_commands_t — live command state updated by handle_command().
 * walk_* fields: mantidos só como estado do protocolo (contrato
 *   congelado) desde a remoção do engine de marcha do firmware — não
 *   comandam movimento (o gait roda no host e chega via
 *   MODE_JOINT_STREAM).
 * keyframe_active: quando 1, o loop principal copia kf_goals -> joint_goals.
 * kf_torque_mask: bit i = 1 aplica kf_goals[i] a joint_goals[i+1]; bit
 *   limpo mantém o valor anterior (comandos esparsos não resetam juntas
 *   não endereçadas pro centro).
 */
typedef struct {
    float    walk_forward;
    float    walk_lateral;
    float    walk_turn;
    uint8_t  walk_enabled;
    uint8_t  mode;
    uint32_t last_heartbeat_ms;
    uint32_t heartbeat_seq;
    bool     heartbeat_pending;
    uint8_t  keyframe_active;
    uint16_t kf_goals[18];
    uint32_t kf_torque_mask;
    int8_t   syscmd_pending;
} current_commands_t;

#ifdef __cplusplus
extern "C" {
#endif

void current_commands_init(current_commands_t *cmds);

void handle_command(uint8_t msg_type, uint8_t *payload, uint8_t len,
                    current_commands_t *cmds);

#ifdef __cplusplus
}
#endif

#endif /* COMMAND_HANDLER_H */
