#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include <stdint.h>
#include "ring_buffer.h"
#include "imu.h"

/* Porte 1:1 do original — framing (COBS+CRC16), constantes de
 * protocolo e assinaturas IDÊNTICAS. Só o transporte (serial_comm.c)
 * muda: USART2/libopencm3 -> Serial1 (link do host) da OpenCR. */

/* Downlink (Jetson/host -> OpenCR) */
#define MSG_WALK_CMD       10U
#define MSG_JOINT_CMD      11U
#define MSG_MODE_CMD       12U
#define MSG_HEARTBEAT_REQ  13U
#define MSG_CONFIG_WALK    15U
#define MSG_SYSTEM_CMD     16U

/* Uplink (OpenCR -> Jetson/host) */
#define MSG_JOINT_STATE    10U
#define MSG_IMU_DATA       11U
#define MSG_SYSTEM_STATUS  12U
#define MSG_HEARTBEAT_ACK  13U

#define SERIAL_MAX_PAYLOAD    132U
#define SERIAL_MAX_FRAME_RAW  144U

#ifdef __cplusplus
extern "C" {
#endif

void serial_comm_init(void);

int serial_parse_frame(ring_buf_t *rb, uint8_t *msg_type,
                       uint8_t *payload, uint8_t *payload_len);

void serial_send_imu(imu_state_t *imu);

void serial_send_joint_state(uint16_t *positions, uint16_t *loads,
                             uint8_t *voltages, uint8_t *temperatures,
                             uint8_t *error_flags);

void serial_send_system_status(uint8_t fsm_state, uint16_t error_flags,
                               uint32_t uptime_ms, uint16_t battery_mv,
                               uint32_t loop_us);

void serial_send_heartbeat_ack(uint32_t jetson_ts, uint32_t seq,
                               uint8_t fsm, uint16_t errors);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_COMM_H */
