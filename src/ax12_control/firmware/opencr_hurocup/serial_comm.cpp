/* OPENCR: serial_comm.c adaptado. CRC16-CCITT, COBS encode/decode e o
 * parser de frames são BYTE-IDÊNTICOS ao original — só o transporte
 * físico muda: USART2 (libopencm3) -> Serial1 (link do host na
 * OpenCR, ver config.h). Arquivo .cpp porque HardwareSerial é C++;
 * wrappers extern "C" mantêm a assinatura igual ao serial_comm.h.
 */
#include <Arduino.h>
#include <string.h>
#include "serial_comm.h"
#include "systick.h"

#define HOST_PORT   Serial1   /* link com o Jetson/host — ver config.h */

/* =========================================================
 * CRC16-CCITT (poly=0x1021, init=0xFFFF) — idêntico ao original.
 * ========================================================= */
static uint16_t crc16_ccitt(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint8_t i = 0U; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (uint8_t j = 0U; j < 8U; j++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1U) ^ 0x1021U)
                                  : (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

/* =========================================================
 * COBS encode/decode — idêntico ao original.
 * ========================================================= */
static int cobs_encode(const uint8_t *input, uint8_t input_len,
                       uint8_t *output, uint8_t max_output_len)
{
    uint8_t out_idx   = 0U;
    uint8_t code_idx  = 0U;
    uint8_t code      = 1U;

    if (max_output_len < (uint8_t)(input_len + 2U)) {
        return -1;
    }

    output[out_idx++] = 0U;
    code_idx = 0U;

    for (uint8_t i = 0U; i < input_len; i++) {
        if (input[i] == 0x00U) {
            output[code_idx] = code;
            code_idx = out_idx;
            if (out_idx >= max_output_len) { return -1; }
            output[out_idx++] = 0U;
            code = 1U;
        } else {
            if (out_idx >= max_output_len) { return -1; }
            output[out_idx++] = input[i];
            code++;
            if (code == 0xFFU) {
                output[code_idx] = 0xFFU;
                code_idx = out_idx;
                if (out_idx >= max_output_len) { return -1; }
                output[out_idx++] = 0U;
                code = 1U;
            }
        }
    }
    output[code_idx] = code;
    return (int)out_idx;
}

static int cobs_decode(const uint8_t *input, uint8_t input_len,
                       uint8_t *output, uint8_t max_output_len)
{
    uint8_t in_idx  = 0U;
    uint8_t out_idx = 0U;

    while (in_idx < input_len) {
        uint8_t code = input[in_idx++];
        if (code == 0x00U) {
            return -1;
        }
        for (uint8_t i = 1U; i < code; i++) {
            if (in_idx >= input_len || out_idx >= max_output_len) {
                return -1;
            }
            output[out_idx++] = input[in_idx++];
        }
        if (code < 0xFFU && in_idx < input_len) {
            if (out_idx >= max_output_len) { return -1; }
            output[out_idx++] = 0x00U;
        }
    }
    return (int)out_idx;
}

/* =========================================================
 * Big-endian serialization helpers — idêntico ao original.
 * ========================================================= */
static void put_u16_be(uint8_t *buf, uint16_t v)
{
    buf[0] = (uint8_t)(v >> 8U);
    buf[1] = (uint8_t)(v);
}

static void put_u32_be(uint8_t *buf, uint32_t v)
{
    buf[0] = (uint8_t)(v >> 24U);
    buf[1] = (uint8_t)(v >> 16U);
    buf[2] = (uint8_t)(v >>  8U);
    buf[3] = (uint8_t)(v);
}

static void put_float_be(uint8_t *buf, float val)
{
    uint32_t u;
    memcpy(&u, &val, sizeof(u));
    put_u32_be(buf, u);
}

/* =========================================================
 * serial_send_frame (internal) — OPENCR: transmite em HOST_PORT
 * (Serial1) em vez de USART2. Wire format idêntico ao original.
 * ========================================================= */
static void serial_send_frame(uint8_t msg_type,
                              const uint8_t *payload, uint8_t payload_len)
{
    uint8_t raw[SERIAL_MAX_PAYLOAD + 3U];
    uint8_t raw_len = (uint8_t)(1U + payload_len + 2U);

    if (raw_len > (uint8_t)sizeof(raw)) { return; }

    raw[0] = msg_type;
    if (payload_len > 0U) {
        memcpy(&raw[1], payload, payload_len);
    }

    uint16_t crc = crc16_ccitt(raw, (uint8_t)(1U + payload_len));
    raw[1U + payload_len]      = (uint8_t)(crc >> 8U);
    raw[1U + payload_len + 1U] = (uint8_t)(crc);

    uint8_t cobs_buf[SERIAL_MAX_PAYLOAD + 8U];
    int enc_len = cobs_encode(raw, raw_len,
                              cobs_buf, (uint8_t)sizeof(cobs_buf));
    if (enc_len < 0) { return; }

    HOST_PORT.write((uint8_t)0x00U);
    for (int i = 0; i < enc_len; i++) {
        HOST_PORT.write(cobs_buf[i]);
    }
    HOST_PORT.write((uint8_t)0x00U);
}

static uint8_t rx_decode_buf[SERIAL_MAX_FRAME_RAW];
static uint8_t rx_decode_len = 0U;
static bool    rx_in_frame   = false;

extern "C" void serial_comm_init(void)
{
    rx_decode_len = 0U;
    rx_in_frame   = false;
}

/* serial_parse_frame — IDÊNTICO ao original (state machine sobre o
 * ring buffer, agnóstico do transporte físico). */
extern "C" int serial_parse_frame(ring_buf_t *rb, uint8_t *msg_type,
                       uint8_t *payload, uint8_t *payload_len)
{
    uint8_t byte;
    while (ring_buf_pop(rb, &byte)) {
        if (!rx_in_frame) {
            if (byte == 0x00U) {
                rx_in_frame   = true;
                rx_decode_len = 0U;
            }
        } else {
            if (byte == 0x00U) {
                uint8_t frame_len = rx_decode_len;
                rx_decode_len = 0U;
                rx_in_frame   = true;

                if (frame_len == 0U) {
                    continue;
                }

                uint8_t decoded[SERIAL_MAX_PAYLOAD + 3U];
                int dec_len = cobs_decode(rx_decode_buf, frame_len,
                                          decoded, (uint8_t)sizeof(decoded));
                if (dec_len < 3) {
                    return -1;
                }

                uint8_t data_len = (uint8_t)((uint8_t)dec_len - 2U);
                uint16_t crc_recv = ((uint16_t)decoded[dec_len - 2] << 8U)
                                  |  (uint16_t)decoded[dec_len - 1];
                uint16_t crc_calc = crc16_ccitt(decoded, data_len);
                if (crc_calc != crc_recv) {
                    return -1;
                }

                *msg_type    = decoded[0];
                *payload_len = (uint8_t)(data_len - 1U);
                if (*payload_len > 0U) {
                    memcpy(payload, &decoded[1], *payload_len);
                }
                return 1;

            } else {
                if (rx_decode_len < SERIAL_MAX_FRAME_RAW) {
                    rx_decode_buf[rx_decode_len++] = byte;
                } else {
                    rx_in_frame   = false;
                    rx_decode_len = 0U;
                }
            }
        }
    }
    return 0;
}

extern "C" void serial_send_imu(imu_state_t *imu)
{
    uint8_t buf[40];
    uint8_t *p = buf;

    put_float_be(p, imu->accel_x);  p += 4;
    put_float_be(p, imu->accel_y);  p += 4;
    put_float_be(p, imu->accel_z);  p += 4;
    put_float_be(p, imu->gyro_x);   p += 4;
    put_float_be(p, imu->gyro_y);   p += 4;
    put_float_be(p, imu->gyro_z);   p += 4;
    put_float_be(p, imu->pitch);    p += 4;
    put_float_be(p, imu->roll);     p += 4;
    put_float_be(p, imu->yaw);      p += 4;
    put_u32_be(p,   imu->timestamp_ms);

    serial_send_frame(MSG_IMU_DATA, buf, (uint8_t)sizeof(buf));
}

extern "C" void serial_send_joint_state(uint16_t *positions, uint16_t *loads,
                             uint8_t *voltages, uint8_t *temperatures,
                             uint8_t *error_flags)
{
    uint8_t buf[130];
    uint8_t *p = buf;

    for (uint8_t i = 0U; i < 18U; i++) { put_u16_be(p, positions[i]); p += 2; }
    for (uint8_t i = 0U; i < 18U; i++) { put_u16_be(p, loads[i]);     p += 2; }
    for (uint8_t i = 0U; i < 18U; i++) { *p++ = voltages[i]; }
    for (uint8_t i = 0U; i < 18U; i++) { *p++ = temperatures[i]; }
    for (uint8_t i = 0U; i < 18U; i++) { *p++ = error_flags[i]; }
    put_u32_be(p, get_ticks());

    serial_send_frame(MSG_JOINT_STATE, buf, (uint8_t)sizeof(buf));
}

extern "C" void serial_send_system_status(uint8_t fsm_state, uint16_t error_flags,
                               uint32_t uptime_ms, uint16_t battery_mv,
                               uint32_t loop_us)
{
    uint8_t buf[13];
    uint8_t *p = buf;

    *p++ = fsm_state;
    put_u16_be(p, error_flags); p += 2;
    put_u32_be(p, uptime_ms);   p += 4;
    put_u16_be(p, battery_mv);  p += 2;
    put_u32_be(p, loop_us);

    serial_send_frame(MSG_SYSTEM_STATUS, buf, (uint8_t)sizeof(buf));
}

extern "C" void serial_send_heartbeat_ack(uint32_t jetson_ts, uint32_t seq,
                               uint8_t fsm, uint16_t errors)
{
    uint8_t buf[15];
    uint8_t *p = buf;

    put_u32_be(p, jetson_ts);   p += 4;
    put_u32_be(p, get_ticks()); p += 4;
    put_u32_be(p, seq);         p += 4;
    *p++ = fsm;
    put_u16_be(p, errors);

    serial_send_frame(MSG_HEARTBEAT_ACK, buf, (uint8_t)sizeof(buf));
}
