#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <stdint.h>

/* Porte 1:1 do original — protocolo/constantes/assinaturas IDÊNTICAS.
 * Só a implementação (dynamixel.c) muda a troca de direção TX/RX
 * (HDSEL do F103 -> drv_dxl_tx_enable() da OpenCR). */

/* ---- Protocol 1.0 constants ---- */
#define DXL_BROADCAST_ID    0xFEU

/* Instruction codes */
#define DXL_PING            0x01U
#define DXL_READ            0x02U
#define DXL_WRITE           0x03U
#define DXL_REG_WRITE       0x04U
#define DXL_ACTION          0x05U
#define DXL_SYNC_WRITE      0x83U

/* AX-12A register addresses (RAM area) */
#define DXL_REG_RETURN_DELAY    5U
#define DXL_REG_TORQUE_ENABLE  24U
#define DXL_REG_GOAL_POSITION  30U
#define DXL_REG_MOVING_SPEED   32U
#define DXL_REG_PRESENT_POS    36U
#define DXL_REG_PRESENT_LOAD   40U
#define DXL_REG_PRESENT_VOLT   42U
#define DXL_REG_PRESENT_TEMP   43U

/* Number of servos on the bus (IDs 1-18) */
#define DXL_NUM_SERVOS         18U

/* Status packet parsed from servo response */
typedef struct {
    uint8_t id;
    uint8_t error;
    uint8_t data[8];
    uint8_t data_len;
} dxl_status_t;

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Low-level API ---- */
void dxl_init(void);

/* Reconfigure the DXL port to an arbitrary baud rate. Used by the
 * boot-time diagnostic scan to probe buses left at a non-default baud. */
void dxl_set_baudrate(uint32_t baud);

/* Loopback self-test do barramento half-duplex. 0=eco recebido,
 * 1=eco ausente. OPENCR: ver aviso em dynamixel.c — mecanismo elétrico
 * diferente do HDSEL original, verificar em bancada. */
uint8_t dxl_hdsel_selftest(void);

/* Returns 0=ok, error byte if servo reports error, 0xFF=timeout */
uint8_t  dxl_ping(uint8_t id);
uint8_t  dxl_ping_timeout(uint8_t id, uint32_t timeout_ms);

/* Returns 0=ok, non-zero on checksum error or timeout */
uint8_t  dxl_read(uint8_t id, uint8_t addr, uint8_t len, uint8_t *data);
uint8_t  dxl_write(uint8_t id, uint8_t addr, uint8_t *data, uint8_t len);
uint8_t  dxl_write16(uint8_t id, uint8_t addr, uint16_t val);
uint16_t dxl_read16(uint8_t id, uint8_t addr);

/* ---- High-level helpers ---- */
void     dxl_set_return_delay(uint8_t id, uint8_t val);
void     dxl_torque_enable(uint8_t id, uint8_t enable);
void     dxl_set_goal(uint8_t id, uint16_t position);
uint16_t dxl_get_position(uint8_t id);
uint8_t  dxl_get_temperature(uint8_t id);

void     dxl_reg_write(uint8_t id, uint8_t addr, uint16_t val);
void     dxl_action(void);

/* ---- Multi-servo API ---- */
void    dxl_sync_write(uint8_t start_addr, uint8_t data_len,
                       uint8_t *ids, uint16_t *values, uint8_t count);
void    dxl_sync_write_positions(uint16_t positions[18]);
void    dxl_read_all_positions(uint16_t positions[18], const uint8_t presence[18]);
void    dxl_read_all_loads(uint16_t loads[18], const uint8_t presence[18]);
void    dxl_read_all(uint16_t positions[18], uint16_t loads[18],
                     uint8_t voltages[18], uint8_t temperatures[18],
                     uint8_t error_flags[18],
                     const uint8_t presence[18]);
void    dxl_torque_enable_all(uint8_t enable);
void    dxl_init_all(void);
uint8_t dxl_scan(uint8_t *found_ids, uint8_t max_count);

#ifdef __cplusplus
}
#endif

#endif /* DYNAMIXEL_H */
