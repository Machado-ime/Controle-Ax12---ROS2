#ifndef JOINT_EXEC_H
#define JOINT_EXEC_H

/* Porte 1:1 do original — lógica pura, nenhuma mudança. */

#include <stdint.h>

typedef struct {
    uint32_t last_stream_tick;
    uint8_t  stream_active;
    uint8_t  ramping;
    uint32_t ramp_start_tick;
    uint16_t target_positions[18];
    uint16_t ramp_start_pose[18];
} joint_exec_state_t;

void joint_exec_init(joint_exec_state_t *state);
void joint_exec_on_joint_cmd(joint_exec_state_t *state,
                             uint16_t positions[18],
                             uint32_t tick);
void joint_exec_tick(joint_exec_state_t *state,
                     uint8_t mode,
                     uint32_t tick,
                     uint16_t joint_goals[18],
                     const uint16_t safe_pose[18]);

#endif /* JOINT_EXEC_H */
