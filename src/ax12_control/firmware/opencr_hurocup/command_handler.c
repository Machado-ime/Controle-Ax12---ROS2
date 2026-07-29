/* Porte 1:1 do original — lógica pura, nenhuma mudança
 * (systick_ms agora vem do wrapper em systick.h/cpp, ver nota lá). */
#include <string.h>
#include "command_handler.h"
#include "config.h"
#include "serial_comm.h"
#include "soft_start.h"
#include "systick.h"
#include "syscmd.h"

static float get_float_be(const uint8_t *buf)
{
    uint32_t u = ((uint32_t)buf[0] << 24U)
               | ((uint32_t)buf[1] << 16U)
               | ((uint32_t)buf[2] <<  8U)
               |  (uint32_t)buf[3];
    float f;
    memcpy(&f, &u, sizeof(f));
    return f;
}

static uint32_t get_u32_be(const uint8_t *buf)
{
    return ((uint32_t)buf[0] << 24U)
         | ((uint32_t)buf[1] << 16U)
         | ((uint32_t)buf[2] <<  8U)
         |  (uint32_t)buf[3];
}

static uint16_t get_uint16_be(const uint8_t *buf)
{
    return (uint16_t)(((uint16_t)buf[0] << 8U) | (uint16_t)buf[1]);
}

void current_commands_init(current_commands_t *cmds)
{
    cmds->walk_forward       = 0.0f;
    cmds->walk_lateral       = 0.0f;
    cmds->walk_turn          = 0.0f;
    cmds->walk_enabled       = 0U;
    cmds->mode               = 0U;
    cmds->last_heartbeat_ms  = 0U;
    cmds->heartbeat_seq      = 0U;
    cmds->heartbeat_pending  = false;
    cmds->keyframe_active    = 0U;
    cmds->kf_torque_mask     = 0U;
    cmds->syscmd_pending     = (int8_t)SYSCMD_NONE;
    for (int i = 0; i < 18; i++) {
        cmds->kf_goals[i] = 512U;
    }
}

void handle_command(uint8_t msg_type, uint8_t *payload, uint8_t len,
                    current_commands_t *cmds)
{
    switch (msg_type) {

    case MSG_WALK_CMD:
        if (len < 13U) { break; }
        cmds->walk_forward = get_float_be(&payload[0]);
        cmds->walk_lateral = get_float_be(&payload[4]);
        cmds->walk_turn    = get_float_be(&payload[8]);
        cmds->walk_enabled = payload[12];
        if (cmds->walk_enabled) {
            cmds->keyframe_active = 0U;
        }
        break;

    case MSG_JOINT_CMD:
        if (len < 76U) { break; }
        if (cmds->mode != (uint8_t)MODE_ACTIVE
            && cmds->mode != (uint8_t)MODE_IDLE
            && cmds->mode != (uint8_t)MODE_JOINT_STREAM) {
            break;
        }

        {
            uint32_t mask = get_u32_be(payload + 72);
            if (mask == 0U) {
                mask = 0x0003FFFFU;
            }

            uint16_t new_goals[18];
            uint16_t speeds[18];
            for (int i = 0; i < 18; i++) {
                uint16_t pos = get_uint16_be(payload + i * 2);
                if (pos > 1023U) { pos = 1023U; }
                new_goals[i] = pos;
                uint16_t spd = get_uint16_be(payload + 36 + i * 2);
                if (spd > 1023U) { spd = 1023U; }
                speeds[i] = spd;
            }

            soft_start_on_joint_cmd(soft_start_instance(), cmds->kf_goals,
                                    new_goals, mask, speeds, systick_ms);

            for (int i = 0; i < 18; i++) {
                cmds->kf_goals[i] = new_goals[i];
            }
            cmds->kf_torque_mask = mask;
        }

        cmds->walk_enabled = 0U;
        if (cmds->mode == (uint8_t)MODE_JOINT_STREAM) {
            cmds->keyframe_active = 0U;
        } else {
            cmds->keyframe_active = 1U;
        }
        break;

    case MSG_MODE_CMD:
        if (len < 1U) { break; }
        cmds->mode = payload[0];
        break;

    case MSG_HEARTBEAT_REQ:
        if (len < 8U) { break; }
        cmds->last_heartbeat_ms = get_u32_be(&payload[0]);
        cmds->heartbeat_seq     = get_u32_be(&payload[4]);
        cmds->heartbeat_pending = true;
        break;

    case MSG_CONFIG_WALK:
        if (len < 36U) { break; }
        break;

    case MSG_SYSTEM_CMD:
        cmds->syscmd_pending = (int8_t)syscmd_parse(payload, len);
        break;

    default:
        break;
    }
}
