/* Porte 1:1 do original — lógica pura, nenhuma mudança. */
#include "joint_exec.h"
#include "config.h"

#define JOINT_EXEC_COUNT         18U
#define JOINT_STREAM_TIMEOUT    10U
#define JOINT_STREAM_RAMP_TICKS 50U

static void copy_pose(uint16_t dst[18], const uint16_t src[18])
{
    for (uint8_t i = 0U; i < JOINT_EXEC_COUNT; i++) {
        dst[i] = src[i];
    }
}

void joint_exec_init(joint_exec_state_t *state)
{
    state->last_stream_tick = 0U;
    state->stream_active = 0U;
    state->ramping = 0U;
    state->ramp_start_tick = 0U;
    for (uint8_t i = 0U; i < JOINT_EXEC_COUNT; i++) {
        state->target_positions[i] = AX12A_CENTER;
        state->ramp_start_pose[i] = AX12A_CENTER;
    }
}

void joint_exec_on_joint_cmd(joint_exec_state_t *state,
                             uint16_t positions[18],
                             uint32_t tick)
{
    copy_pose(state->target_positions, positions);
    state->last_stream_tick = tick;
    state->stream_active = 1U;
    state->ramping = 0U;
}

void joint_exec_tick(joint_exec_state_t *state,
                     uint8_t mode,
                     uint32_t tick,
                     uint16_t joint_goals[18],
                     const uint16_t safe_pose[18])
{
    if (mode != (uint8_t)MODE_JOINT_STREAM) {
        state->stream_active = 0U;
        state->ramping = 0U;
        return;
    }

    if (!state->stream_active) {
        return;
    }

    if (!state->ramping) {
        if ((uint32_t)(tick - state->last_stream_tick) <= JOINT_STREAM_TIMEOUT) {
            copy_pose(joint_goals, state->target_positions);
            return;
        }

        copy_pose(state->ramp_start_pose, joint_goals);
        state->ramp_start_tick = tick;
        state->ramping = 1U;
    }

    uint32_t elapsed = tick - state->ramp_start_tick;
    if (elapsed >= JOINT_STREAM_RAMP_TICKS) {
        copy_pose(joint_goals, safe_pose);
        state->stream_active = 0U;
        state->ramping = 0U;
        return;
    }

    for (uint8_t i = 0U; i < JOINT_EXEC_COUNT; i++) {
        int32_t start = (int32_t)state->ramp_start_pose[i];
        int32_t end = (int32_t)safe_pose[i];
        int32_t delta = end - start;
        joint_goals[i] = (uint16_t)(start + ((delta * (int32_t)elapsed)
                                  / (int32_t)JOINT_STREAM_RAMP_TICKS));
    }
}
