/* OPENCR: dynamixel.c adaptado. O framing/checksum/state-machine do
 * protocolo (dxl_send_packet, dxl_receive_packet e tudo que chama elas)
 * é IDÊNTICO ao original — só a troca de direção TX/RX muda:
 *
 *   Original (F103): bit HDSEL do USART1 — um único pino half-duplex
 *     nativo do periférico; usart_set_mode(TX) desliga o receptor por
 *     hardware, então o próprio eco do TX nunca entra no RX.
 *   OpenCR: TX/RX são pinos separados ligados no mesmo barramento por
 *     um transistor controlado por drv_dxl_tx_enable() (mesmo mecanismo
 *     do usb_to_dxl oficial da ROBOTIS). drv_dxl_tx_enable(true) assume
 *     o barramento pra escrever; drv_dxl_tx_enable(false) libera pra
 *     escutar.
 *
 * Arquivo .cpp (não .c) porque DXL_PORT é um objeto HardwareSerial
 * (Serial3) da API Arduino; os wrappers extern "C" abaixo mantêm a
 * assinatura idêntica ao dynamixel.h original.
 */
#include "dynamixel.h"
#include <Arduino.h>
#include <stddef.h>

#define DXL_PORT   Serial3   /* mesmo nome usado no usb_to_dxl oficial da ROBOTIS */

/* drv_dxl_tx_enable(BOOL) é declarada pelo core da OpenCR (visível via
 * <Arduino.h>/board.h) — chamada direto, sem prototype nosso, igual ao
 * usb_to_dxl.ino oficial. Não redeclaramos aqui de propósito: um
 * protótipo nosso com o tipo do parâmetro errado (o core pode usar
 * BOOL/bool, não necessariamente uint8_t) arriscaria incompatibilidade
 * silenciosa em vez de só compilar contra a declaração real. */

/* dxl_flush_rx — descarta bytes obsoletos/ruído antes de entrar em modo TX.
 * Equivalente ao original (que drenava USART_SR_RXNE/ORE manualmente);
 * aqui o buffer da HardwareSerial já resolve overrun internamente. */
static void dxl_flush_rx(void)
{
    while (DXL_PORT.available() > 0) {
        (void)DXL_PORT.read();
    }
}

/*
 * Instruction packet layout (Protocol 1.0) — idêntico ao original:
 *   [0xFF][0xFF][ID][Length][Instruction][Params...][Checksum]
 */
static void dxl_send_packet(uint8_t id, uint8_t instruction,
                             uint8_t *params, uint8_t param_len)
{
    uint8_t length   = (uint8_t)(param_len + 2U);
    uint8_t sum      = (uint8_t)(id + length + instruction);
    for (uint8_t i = 0U; i < param_len; i++) {
        sum = (uint8_t)(sum + params[i]);
    }
    uint8_t checksum = (uint8_t)(~sum & 0xFFU);

    dxl_flush_rx();

    drv_dxl_tx_enable(1U);

    DXL_PORT.write((uint8_t)0xFFU);
    DXL_PORT.write((uint8_t)0xFFU);
    DXL_PORT.write(id);
    DXL_PORT.write(length);
    DXL_PORT.write(instruction);
    for (uint8_t i = 0U; i < param_len; i++) {
        DXL_PORT.write(params[i]);
    }
    DXL_PORT.write(checksum);
    DXL_PORT.flush();   /* bloqueia até o shift register esvaziar (~TC) */

    drv_dxl_tx_enable(0U);
}

/*
 * Status packet layout — idêntico ao original.
 * Returns: 0=ok  1=checksum mismatch  0xFF=timeout
 */
static uint8_t dxl_receive_packet(dxl_status_t *status, uint8_t expected_id,
                                   uint32_t timeout_ms)
{
    uint32_t start = millis();
    uint8_t  b, ff = 0U, rx_id = 0U, rx_len = 0U, rx_err = 0U, di = 0U;
    uint8_t  phase = 0U;

    while ((millis() - start) < timeout_ms) {
        if (DXL_PORT.available() <= 0) { continue; }
        b = (uint8_t)DXL_PORT.read();

        switch (phase) {
        case 0U: /* sync to 0xFF 0xFF */
            if (b == 0xFFU) {
                if (++ff >= 2U) { phase = 1U; ff = 0U; }
            } else {
                ff = 0U;
            }
            break;

        case 1U: /* ID */
            if (b == 0xFFU) {
                break;   /* 0xFF extra no header: mantém sincronismo */
            }
            if (expected_id != DXL_BROADCAST_ID && b != expected_id) {
                phase = 0U; ff = 0U;
                break;
            }
            rx_id      = b;
            status->id = b;
            phase      = 2U;
            break;

        case 2U: /* Length — minimum valid value is 2 */
            if (b < 2U) { phase = 0U; ff = 0U; break; }
            rx_len = b;
            phase  = 3U;
            break;

        case 3U: /* Error */
            rx_err        = b;
            status->error = b;
            di            = 0U;
            phase         = (rx_len > 2U) ? 4U : 5U;
            break;

        case 4U: /* Data: (rx_len - 2) bytes */
            if (di < 8U) { status->data[di] = b; }
            di++;
            if (di >= (uint8_t)(rx_len - 2U)) { phase = 5U; }
            break;

        case 5U: { /* Checksum — verify and return */
            uint8_t s = (uint8_t)(rx_id + rx_len + rx_err);
            uint8_t n = (rx_len > 2U) ? (uint8_t)(rx_len - 2U) : 0U;
            if (n > 8U) { n = 8U; }
            for (uint8_t i = 0U; i < n; i++) {
                s = (uint8_t)(s + status->data[i]);
            }
            status->data_len = n;
            return (b == (~s & 0xFFU)) ? 0U : 1U;
        }

        default:
            phase = 0U;
            ff    = 0U;
            break;
        }
    }

    return 0xFFU; /* timeout */
}

/* ==========================================================================
 * Public API — idêntico ao original em tudo que não envolve TX/RX direto.
 * ========================================================================== */

extern "C" void dxl_init(void)
{
    DXL_PORT.begin(1000000U);
    drv_dxl_tx_enable(0U);   /* começa em RX (escutando) */
}

extern "C" void dxl_set_baudrate(uint32_t baud)
{
    DXL_PORT.begin(baud);
    dxl_flush_rx();
}

/* OPENCR: mecanismo elétrico diferente do HDSEL original (ver
 * cabeçalho do arquivo) — este self-test assume que, com
 * drv_dxl_tx_enable(1), o byte transmitido tambem aparece no RX (bus
 * compartilhado). NÃO VERIFICADO em bancada com o schematic da OpenCR;
 * confirme com um osciloscópio ou logic analyzer antes de confiar 100%
 * no resultado — se o transistor de direção isolar TX de RX durante a
 * escrita (diferente do HDSEL, que reaproveita o mesmo pino), o eco
 * pode nunca aparecer mesmo com o barramento saudável, dando falso
 * negativo. */
extern "C" uint8_t dxl_hdsel_selftest(void)
{
    dxl_flush_rx();
    drv_dxl_tx_enable(1U);
    DXL_PORT.write((uint8_t)0xA5U);
    DXL_PORT.flush();
    drv_dxl_tx_enable(0U);

    uint8_t result = 1U;
    uint32_t start = millis();
    while ((millis() - start) < 5U) {
        if (DXL_PORT.available() > 0) {
            uint8_t b = (uint8_t)DXL_PORT.read();
            if (b == 0xA5U) { result = 0U; break; }
        }
    }
    return result;
}

extern "C" uint8_t dxl_ping(uint8_t id)
{
    return dxl_ping_timeout(id, 5U);
}

extern "C" uint8_t dxl_ping_timeout(uint8_t id, uint32_t timeout_ms)
{
    dxl_status_t status;
    dxl_send_packet(id, DXL_PING, NULL, 0U);
    uint8_t res = dxl_receive_packet(&status, id, timeout_ms);
    if (res == 0xFFU) { return 0xFFU; }
    if (res != 0U)    { return 0xFEU; }
    return status.error;
}

static uint8_t dxl_read_reply(uint8_t id, uint8_t addr, uint8_t len,
                              uint8_t *data, uint8_t *status_error)
{
    uint8_t params[2] = {addr, len};
    dxl_send_packet(id, DXL_READ, params, 2U);
    dxl_status_t status;
    uint8_t res = dxl_receive_packet(&status, id, 5U);
    if (res != 0U) { return res; }
    if (status.data_len < len) { return 0xFEU; }
    for (uint8_t i = 0U; i < len; i++) { data[i] = status.data[i]; }
    if (status_error != NULL) {
        *status_error = status.error;
    }
    return 0U;
}

extern "C" uint8_t dxl_read(uint8_t id, uint8_t addr, uint8_t len, uint8_t *data)
{
    uint8_t status_error = 0U;
    uint8_t res = dxl_read_reply(id, addr, len, data, &status_error);
    if (res != 0U) { return res; }
    return status_error;
}

extern "C" uint8_t dxl_write(uint8_t id, uint8_t addr, uint8_t *data, uint8_t len)
{
    uint8_t params[10];
    if (len > 9U) { return 0xFEU; }
    params[0] = addr;
    for (uint8_t i = 0U; i < len; i++) { params[i + 1U] = data[i]; }
    dxl_send_packet(id, DXL_WRITE, params, (uint8_t)(len + 1U));
    if (id == DXL_BROADCAST_ID) { return 0U; }
    dxl_status_t status;
    uint8_t res = dxl_receive_packet(&status, id, 5U);
    if (res != 0U) { return res; }
    return status.error;
}

extern "C" uint8_t dxl_write16(uint8_t id, uint8_t addr, uint16_t val)
{
    uint8_t d[2] = {(uint8_t)(val & 0xFFU), (uint8_t)(val >> 8U)};
    return dxl_write(id, addr, d, 2U);
}

extern "C" uint16_t dxl_read16(uint8_t id, uint8_t addr)
{
    uint8_t d[2] = {0U, 0U};
    dxl_read(id, addr, 2U, d);
    return (uint16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8U));
}

extern "C" void dxl_set_return_delay(uint8_t id, uint8_t val)
{
    dxl_write(id, DXL_REG_RETURN_DELAY, &val, 1U);
}

extern "C" void dxl_torque_enable(uint8_t id, uint8_t enable)
{
    uint8_t d = enable ? 1U : 0U;
    dxl_write(id, DXL_REG_TORQUE_ENABLE, &d, 1U);
}

extern "C" void dxl_set_goal(uint8_t id, uint16_t position)
{
    dxl_write16(id, DXL_REG_GOAL_POSITION, position);
}

extern "C" uint16_t dxl_get_position(uint8_t id)
{
    return dxl_read16(id, DXL_REG_PRESENT_POS);
}

extern "C" uint8_t dxl_get_temperature(uint8_t id)
{
    uint8_t d = 0U;
    dxl_read(id, DXL_REG_PRESENT_TEMP, 1U, &d);
    return d;
}

extern "C" void dxl_reg_write(uint8_t id, uint8_t addr, uint16_t val)
{
    uint8_t params[3] = {addr, (uint8_t)(val & 0xFFU), (uint8_t)(val >> 8U)};
    dxl_send_packet(id, DXL_REG_WRITE, params, 3U);
    if (id == DXL_BROADCAST_ID) { return; }
    dxl_status_t status;
    (void)dxl_receive_packet(&status, id, 5U);
}

extern "C" void dxl_action(void)
{
    dxl_send_packet(DXL_BROADCAST_ID, DXL_ACTION, NULL, 0U);
}

/* ==========================================================================
 * Multi-servo API — idêntico ao original.
 * ========================================================================== */

extern "C" void dxl_sync_write(uint8_t start_addr, uint8_t data_len,
                    uint8_t *ids, uint16_t *values, uint8_t count)
{
    uint8_t param_len = (uint8_t)(2U + (uint8_t)((uint8_t)(data_len + 1U) * count));
    uint8_t length    = (uint8_t)(param_len + 2U);

    uint8_t s = (uint8_t)(DXL_BROADCAST_ID + length + DXL_SYNC_WRITE
                          + start_addr + data_len);
    for (uint8_t i = 0U; i < count; i++) {
        s = (uint8_t)(s + ids[i]);
        s = (uint8_t)(s + (uint8_t)(values[i] & 0xFFU));
        if (data_len >= 2U) {
            s = (uint8_t)(s + (uint8_t)(values[i] >> 8U));
        }
    }
    uint8_t checksum = (uint8_t)(~s & 0xFFU);

    dxl_flush_rx();
    drv_dxl_tx_enable(1U);

    DXL_PORT.write((uint8_t)0xFFU);
    DXL_PORT.write((uint8_t)0xFFU);
    DXL_PORT.write(DXL_BROADCAST_ID);
    DXL_PORT.write(length);
    DXL_PORT.write(DXL_SYNC_WRITE);
    DXL_PORT.write(start_addr);
    DXL_PORT.write(data_len);
    for (uint8_t i = 0U; i < count; i++) {
        DXL_PORT.write(ids[i]);
        DXL_PORT.write((uint8_t)(values[i] & 0xFFU));
        if (data_len >= 2U) {
            DXL_PORT.write((uint8_t)(values[i] >> 8U));
        }
    }
    DXL_PORT.write(checksum);
    DXL_PORT.flush();

    drv_dxl_tx_enable(0U);
}

extern "C" void dxl_sync_write_positions(uint16_t positions[18])
{
    uint8_t ids[DXL_NUM_SERVOS];
    for (uint8_t i = 0U; i < DXL_NUM_SERVOS; i++) {
        ids[i] = (uint8_t)(i + 1U);
    }
    dxl_sync_write(DXL_REG_GOAL_POSITION, 2U, ids, positions, DXL_NUM_SERVOS);
}

extern "C" void dxl_read_all_positions(uint16_t positions[18], const uint8_t presence[18])
{
    for (uint8_t i = 0U; i < DXL_NUM_SERVOS; i++) {
        if (presence != NULL && presence[i] == 0U) {
            positions[i] = 0xFFFFU;
            continue;
        }
        uint8_t id   = (uint8_t)(i + 1U);
        uint8_t d[2] = {0U, 0U};
        uint8_t res  = dxl_read(id, DXL_REG_PRESENT_POS, 2U, d);
        positions[i] = (res != 0U)
                       ? 0xFFFFU
                       : (uint16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8U));
    }
}

extern "C" void dxl_read_all_loads(uint16_t loads[18], const uint8_t presence[18])
{
    for (uint8_t i = 0U; i < DXL_NUM_SERVOS; i++) {
        if (presence != NULL && presence[i] == 0U) {
            loads[i] = 0xFFFFU;
            continue;
        }
        uint8_t id   = (uint8_t)(i + 1U);
        uint8_t d[2] = {0U, 0U};
        uint8_t res  = dxl_read(id, DXL_REG_PRESENT_LOAD, 2U, d);
        loads[i] = (res != 0U)
                   ? 0xFFFFU
                   : (uint16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8U));
    }
}

extern "C" void dxl_read_all(uint16_t positions[18], uint16_t loads[18],
                  uint8_t voltages[18], uint8_t temperatures[18],
                  uint8_t error_flags[18],
                  const uint8_t presence[18])
{
    for (uint8_t i = 0U; i < DXL_NUM_SERVOS; i++) {
        if (presence != NULL && presence[i] == 0U) {
            positions[i] = 0xFFFFU;
            loads[i]     = 0xFFFFU;
            voltages[i]  = 0U;
            temperatures[i] = 0U;
            error_flags[i]  = 0U;
            continue;
        }
        uint8_t id   = (uint8_t)(i + 1U);
        uint8_t d[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
        uint8_t err  = 0U;
        uint8_t res  = dxl_read_reply(id, DXL_REG_PRESENT_POS, 8U, d, &err);
        if (res != 0U) {
            positions[i] = 0xFFFFU;
            loads[i]     = 0xFFFFU;
            voltages[i]  = 0U;
            temperatures[i] = 0U;
            error_flags[i]  = 0U;
        } else {
            positions[i] = (uint16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8U));
            loads[i]     = (uint16_t)((uint16_t)d[4] | ((uint16_t)d[5] << 8U));
            voltages[i]  = d[6];
            temperatures[i] = d[7];
            error_flags[i]  = err;
        }
    }
}

extern "C" void dxl_torque_enable_all(uint8_t enable)
{
    uint8_t  ids[DXL_NUM_SERVOS];
    uint16_t vals[DXL_NUM_SERVOS];
    uint16_t v = enable ? 1U : 0U;
    for (uint8_t i = 0U; i < DXL_NUM_SERVOS; i++) {
        ids[i]  = (uint8_t)(i + 1U);
        vals[i] = v;
    }
    dxl_sync_write(DXL_REG_TORQUE_ENABLE, 1U, ids, vals, DXL_NUM_SERVOS);
}

extern "C" void dxl_init_all(void)
{
    for (uint8_t i = 0U; i < DXL_NUM_SERVOS; i++) {
        uint8_t id  = (uint8_t)(i + 1U);
        uint8_t res = dxl_ping(id);
        if (res != 0xFFU) {
            dxl_set_return_delay(id, 0U);
            dxl_torque_enable(id, 1U);
        }
    }
}

extern "C" uint8_t dxl_scan(uint8_t *found_ids, uint8_t max_count)
{
    uint8_t found = 0U;
    for (uint16_t id = 0U; id <= 253U; id++) {
        if (found >= max_count) { break; }
        if (dxl_ping((uint8_t)id) != 0xFFU) {
            found_ids[found++] = (uint8_t)id;
        }
    }
    return found;
}
