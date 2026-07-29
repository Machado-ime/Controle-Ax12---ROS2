/*
 * opencr_hurocup — porte do firmware HuroCup (STM32F103C6/Bluepill,
 * libopencm3) para a ROBOTIS OpenCR (STM32F746ZGT6, core Arduino).
 *
 * Estrutura do boot e do loop de 100 Hz mantida O MAIS PARECIDA
 * POSSÍVEL com o main.c original — a ordem das etapas, os nomes de
 * variável/função e os comentários de intenção foram preservados; só
 * o que é fisicamente diferente na placa foi trocado, sempre marcado
 * "OPENCR:" no comentário mais próximo. Ver os .h/.c de cada módulo
 * pra detalhe por módulo.
 *
 * NÃO PORTADO (órfãos no main.c original, nada os chama):
 *   ax12a.c/h (driver antigo, substituído por dynamixel.c) e
 *   movimento.h (tabela de keyframes de teste, não referenciada).
 *
 * PENDENTE DE VERIFICAÇÃO EM BANCADA (documentado em cada arquivo):
 *   - eixo/sinal do IMU (imu_fusion.h) — montagem física é outra.
 *   - dxl_hdsel_selftest() (dynamixel.cpp) — mecanismo elétrico novo.
 *   - bootflag.cpp (registrador de backup do RTC) — API não exercitada
 *     em hardware ainda; nada depende dela funcionar hoje.
 *   - mapeamento de pinos do link do host em Serial1 (config.h) —
 *     confira qual conector físico da OpenCR corresponde antes de ligar
 *     o Jetson/PC nele.
 */

#include <HardwareTimer.h>

#include "config.h"
#include "dynamixel.h"
#include "imu.h"
#include "ring_buffer.h"
#include "systick.h"
#include "serial_comm.h"
#include "command_handler.h"
#include "stabilizer.h"
#include "joint_exec.h"
#include "soft_start.h"
#include "safety.h"
#include "battery.h"
#include "battery_hw.h"
#include "syscmd.h"
#include "bootflag.h"

#define HOST_PORT   Serial1   /* link com o Jetson/host — ver config.h */

/* ===== Globais compartilhados (equivalente a handlers.c do original) === */
volatile bool  tick_ready       = false;
volatile bool  rx_frame_pending = false;   /* mantido por fidelidade;
                                             * o original também não
                                             * chegava a checar este
                                             * flag no loop principal. */
ring_buf_t     rx_buf;

static imu_state_t imu_state;

static current_commands_t current_cmds;
static joint_exec_state_t joint_exec;
static safety_state_t safety_state;

static const uint8_t SERVO_IDS[SERVO_COUNT] = {
    1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U,
    10U, 11U, 12U, 13U, 14U, 15U, 16U, 17U, 18U
};

static uint8_t servo_present[SERVO_COUNT];
static uint8_t servo_count_found = 0U;

#define HOME_POSITION  512U
static const uint16_t SAFE_POSE[18] = {
    512U, 512U, 512U, 512U, 512U, 512U, 512U, 512U, 512U,
    413U, 858U, 265U, 512U, 512U, 611U, 166U, 759U, 512U
};
static uint16_t joint_goals[18];
static uint16_t joint_positions[18];
static uint16_t joint_loads[18];
static uint8_t  joint_voltages[18];
static uint8_t  joint_temperatures[18];
static uint8_t  joint_errors[18];
static uint32_t dxl_read_ms = 0U;

static uint8_t speed_write_cb(uint8_t id, uint16_t spd)
{
    return dxl_write16(id, DXL_REG_MOVING_SPEED, spd);
}

static stabilizer_params_t stab_params;
static stabilizer_state_t  stab_state;
static stabilizer_output_t stab_output;

static battery_filter_t battery_filter;
static uint16_t         battery_mv = 0U;

/* Estado do loop (equivalente às variáveis locais do while(1) do
 * original; aqui são globais porque loop() é chamada repetidamente
 * pelo framework Arduino, não é um laço único como lá). */
static uint32_t g_tick_count = 0U;
static uint32_t g_last_tick_ms = 0U;

/* ===== IWDG — OPENCR: via HAL (o F7 tem o mesmo periférico IWDG,
 * API diferente do libopencm3). Período nominal idêntico ao original
 * (2000 ms); mesma ressalva de tolerância do LSI (30-60 kHz) do
 * comentário original em config.h. ===== */
extern "C" {
#include "stm32f7xx_hal.h"
}
static IWDG_HandleTypeDef hiwdg;

static void iwdg_setup(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    hiwdg.Init.Reload    = 249U;   /* ~2000 ms nominal @ LSI ~32 kHz */
    HAL_IWDG_Init(&hiwdg);
}

static void iwdg_reset(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}

/* ===== Timer de 100 Hz — OPENCR: HardwareTimer em vez de TIM2 cru.
 * tick_isr() só seta a flag, igual ao tim2_isr() original. ===== */
static HardwareTimer *g_tick_timer = nullptr;

static void tick_isr(void)
{
    tick_ready = true;
}

static void tick_timer_setup(void)
{
    g_tick_timer = new HardwareTimer(TIM2);
    g_tick_timer->setOverflow(TIM2_FREQ_HZ, HERTZ_FORMAT);
    g_tick_timer->attachInterrupt(tick_isr);
    g_tick_timer->resume();
}

/* ===== Drena o buffer da HardwareSerial pro ring buffer — OPENCR:
 * substitui o usart2_isr() do original (que empilhava byte a byte
 * dentro de uma ISR de verdade). A HardwareSerial já tem seu próprio
 * buffer de RX servido por interrupção internamente; aqui só
 * transferimos pro ring_buf_t que o resto do firmware espera, mantendo
 * serial_comm.c/command_handler.c inalterados. Chamado no topo do
 * loop(), então o atraso é no máximo uma volta do loop (< 1 ms na
 * prática). ===== */
static void drain_host_serial(void)
{
    while (HOST_PORT.available() > 0) {
        uint8_t byte = (uint8_t)HOST_PORT.read();
        ring_buf_push(&rx_buf, byte);
        if (byte == 0x00U) {
            rx_frame_pending = true;
        }
    }
}

/* ===== Debug boot messages — OPENCR: impressas em HOST_PORT, igual ao
 * original (que usava jetson_str sobre a própria USART2/Jetson). ===== */
static void jetson_str(const char *s)
{
    while (*s) { HOST_PORT.write((uint8_t)*s++); }
}

static void jetson_hex(uint8_t v)
{
    static const char h[] = "0123456789ABCDEF";
    HOST_PORT.write((uint8_t)h[v >> 4]);
    HOST_PORT.write((uint8_t)h[v & 0x0FU]);
}

static void jetson_uint(uint32_t v)
{
    char    buf[10];
    uint8_t n = 0U;
    if (v == 0U) { HOST_PORT.write((uint8_t)'0'); return; }
    while (v > 0U && n < (uint8_t)sizeof(buf)) {
        buf[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    while (n > 0U) { HOST_PORT.write((uint8_t)buf[--n]); }
}

/* ===== Scan de servos — IDÊNTICO ao original (lógica 100% protocolo,
 * sem acesso a hardware fora das funções dxl_*). ===== */

static const uint32_t DXL_ALT_BAUDS[] = {
    500000U, 400000U, 117647U, 57600U, 19200U, 9600U
};
#define DXL_ALT_BAUD_COUNT ((uint8_t)(sizeof(DXL_ALT_BAUDS)/sizeof(DXL_ALT_BAUDS[0])))

static void print_ping_result(uint16_t id, uint8_t res)
{
    jetson_str("  ID=");
    jetson_uint(id);
    if (res == 0U) {
        jetson_str(" [OK]\r\n");
    } else if (res == 0xFFU) {
        jetson_str(" [--]\r\n");
    } else {
        jetson_str(" [ERR=0x");
        jetson_hex(res);
        jetson_str("]\r\n");
    }
}

static uint32_t scan_timeout_for_baud(uint32_t baud)
{
    uint32_t byte_us = (10000000U + baud - 1U) / baud;
    uint32_t t_us    = byte_us * 20U;
    uint32_t t_ms    = (t_us + 999U) / 1000U;
    if (t_ms < 5U)  { t_ms = 5U;  }
    if (t_ms > 50U) { t_ms = 50U; }
    return t_ms;
}

static uint8_t servo_scan_range(uint16_t lo, uint16_t hi, uint32_t timeout_ms,
                                uint8_t mark_presence, uint8_t verbose)
{
    uint8_t hits = 0U;
    for (uint16_t id = lo; id <= hi; id++) {
        if (id == DXL_BROADCAST_ID) { continue; }
        uint8_t res = dxl_ping_timeout((uint8_t)id, timeout_ms);
        if (res == 0xFFU) { continue; }

        hits++;
        if (verbose) { print_ping_result(id, res); }

        if (mark_presence && id >= 1U && id <= SERVO_COUNT) {
            uint8_t idx = (uint8_t)(id - 1U);
            if (!servo_present[idx]) {
                servo_present[idx] = 1U;
                servo_count_found++;
            }
            if (res == 0U) {
                dxl_set_return_delay((uint8_t)id, 0U);
            }
            dxl_torque_enable((uint8_t)id, 1U);
        }
    }
    return hits;
}

static void servo_scan(void)
{
    for (uint8_t i = 0U; i < SERVO_COUNT; i++) { servo_present[i] = 0U; }
    servo_count_found = 0U;

    jetson_str("HDSEL echo probe: ");
    uint8_t echo_fail = dxl_hdsel_selftest();
    jetson_str(echo_fail ? "no echo (bus may be unterminated)\r\n"
                         : "echo present (bus is driving back)\r\n");

    jetson_str("Servo scan IDs 1-18 @ 1 Mbps (5 ms/ID):\r\n");
    for (uint8_t i = 0U; i < SERVO_COUNT; i++) {
        uint8_t id  = SERVO_IDS[i];
        uint8_t res = dxl_ping(id);
        print_ping_result(id, res);
        if (res != 0xFFU) {
            servo_present[i] = 1U;
            servo_count_found++;
            if (res == 0U) { dxl_set_return_delay(id, 0U); }
            dxl_torque_enable(id, 1U);
        }
    }
    jetson_str("Layer 2: ");
    jetson_uint(servo_count_found);
    jetson_str("/18 servos online.\r\n");

    if (servo_count_found > 0U) { return; }

    jetson_str("Layer 3: extended scan IDs 0-253 @ 1 Mbps...\r\n");
    uint8_t ext_hits = servo_scan_range(0U, 253U,
                                        scan_timeout_for_baud(1000000U),
                                        1U, 1U);
    jetson_str("Layer 3: ");
    jetson_uint(ext_hits);
    jetson_str(" servo(s) answered.\r\n");

    if (servo_count_found > 0U || ext_hits > 0U) { return; }

    (void)echo_fail;

    for (uint8_t b = 0U; b < DXL_ALT_BAUD_COUNT; b++) {
        uint32_t baud = DXL_ALT_BAUDS[b];
        uint32_t tmo  = scan_timeout_for_baud(baud);
        jetson_str("Layer 4: scan @ ");
        jetson_uint(baud);
        jetson_str(" bps (tmo=");
        jetson_uint(tmo);
        jetson_str(" ms)...\r\n");
        dxl_set_baudrate(baud);
        uint8_t hits = servo_scan_range(0U, 253U, tmo, 0U, 1U);
        if (hits > 0U) {
            jetson_str("!!! ");
            jetson_uint(hits);
            jetson_str(" servo(s) at ");
            jetson_uint(DXL_ALT_BAUDS[b]);
            jetson_str(" bps -- rewrite EEPROM Baud_Rate=1 to restore 1 Mbps.\r\n");
            return;
        }
    }

    dxl_set_baudrate(1000000U);
    jetson_str("No servos detected at any tested baud rate.\r\n");
}

/* ========================================================================
 * setup() — espelha a sequência do main() original (ver comentário no
 * topo do arquivo pra o que mudou em cada etapa).
 * ======================================================================== */
void setup(void)
{
    /* OPENCR: clock/PLL já configurados pelo core antes do setup()
     * rodar (equivalente a clock_setup()+setup_systick() do original —
     * não há nada a fazer aqui). */

    bootflag_clear();

    /* OPENCR: liga o rail de energia dos motores (não existe no F103
     * original — lá os AX-12A eram sempre alimentados pela bateria
     * principal sem uma chave de software). */
    pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
    digitalWrite(BDPIN_DXL_PWR_EN, HIGH);

    pinMode(BDPIN_LED_USER_1, OUTPUT);

    ring_buf_init(&rx_buf);

    HOST_PORT.begin(USART_JETSON_BAUD);

    /* Pisca o LED por ~3 s (igual ao original: tempo pra abrir o
     * monitor serial antes do resto do boot imprimir). */
    for (uint8_t b = 0U; b < 20U; b++) {
        digitalWrite(BDPIN_LED_USER_1, !digitalRead(BDPIN_LED_USER_1));
        delay(150U);
    }
    digitalWrite(BDPIN_LED_USER_1, LOW);

    jetson_str("\r\n=== HuroCup OpenCR BOOT (porte) ===\r\n");
    jetson_str("HOST_PORT 460800 OK\r\n");

    jetson_str("imu_bus_recover...\r\n");
    imu_bus_recover();
    jetson_str("imu_init...\r\n");
    imu_init();
    jetson_str("imu_who_am_i...\r\n");

    uint8_t who = imu_who_am_i();
    jetson_str("IMU smoke-test=0x");
    jetson_hex(who);
    jetson_str(who == 0xD1U ? " [OK]\r\n" : " [WARN]\r\n");

    if (who == 0xD1U) {
        jetson_str("IMU calibrating (hold still)...\r\n");
        imu_calibrate(&imu_state, 100U);
        jetson_str("IMU CAL OK\r\n");
    } else {
        jetson_str("IMU implausible reading -- skipping calibration.\r\n");
    }

    dxl_init();
    servo_scan();

    for (uint8_t i = 0U; i < 18U; i++) {
        joint_goals[i]     = HOME_POSITION;
        joint_positions[i] = 0xFFFFU;
        joint_loads[i]     = 0xFFFFU;
        joint_voltages[i]  = 0U;
        joint_temperatures[i] = 0U;
        joint_errors[i]    = 0U;
    }

    for (uint8_t i = 0U; i < SERVO_COUNT; i++) {
        if (servo_present[i]) {
            dxl_write16((uint8_t)(i + 1U), DXL_REG_MOVING_SPEED,
                        BOOT_MOVING_SPEED);
        }
    }
    soft_start_init(soft_start_instance(), BOOT_MOVING_SPEED, millis());
    jetson_str("Boot soft-start: Moving Speed=");
    jetson_uint(BOOT_MOVING_SPEED);
    jetson_str(" (present servos)\r\n");

    current_commands_init(&current_cmds);
    joint_exec_init(&joint_exec);
    safety_init(&safety_state);

    stabilizer_set_defaults(&stab_params);
    stabilizer_init(&stab_state);
    jetson_str("Stabilizer init OK\r\n");

    battery_filter_init(&battery_filter);
    battery_hw_init();
    jetson_str("Battery init OK\r\n");

    serial_comm_init();

    tick_timer_setup();

    g_tick_count   = 0U;
    g_last_tick_ms = millis();

    jetson_str("Loop started.\r\n");

    iwdg_setup();
}

/* ========================================================================
 * loop() — corpo do while(1) original, gated em tick_ready (100 Hz).
 * ======================================================================== */
void loop(void)
{
    drain_host_serial();

    if (!tick_ready) { return; }
    tick_ready = false;
    iwdg_reset();
    g_tick_count++;
    uint32_t tick_count = g_tick_count;

    uint32_t now_ms        = millis();
    uint32_t tick_start_ms = now_ms;
    uint32_t dt_ms         = now_ms - g_last_tick_ms;
    g_last_tick_ms         = now_ms;
    if (dt_ms < 1U)  { dt_ms = 1U;  }
    if (dt_ms > 50U) { dt_ms = 50U; }
    float tick_dt = (float)dt_ms * 0.001f;

    /* 1 -- Parse incoming serial frames + handle commands */
    {
        uint8_t msg_type;
        uint8_t payload[SERIAL_MAX_PAYLOAD];
        uint8_t payload_len;
        int res;
        do {
            res = serial_parse_frame(&rx_buf, &msg_type,
                                     payload, &payload_len);
            if (res == 1) {
                handle_command(msg_type, payload, payload_len,
                               &current_cmds);
                if (msg_type == MSG_JOINT_CMD
                    && current_cmds.mode == (uint8_t)MODE_JOINT_STREAM) {
                    uint16_t stream_positions[18];
                    uint32_t mask = current_cmds.kf_torque_mask;
                    for (uint8_t i = 0U; i < 18U; i++) {
                        stream_positions[i] = joint_goals[i];
                        if ((mask & (1U << i)) != 0U) {
                            stream_positions[i] = current_cmds.kf_goals[i];
                        }
                    }
                    joint_exec_on_joint_cmd(&joint_exec, stream_positions, tick_count);
                }
                if (current_cmds.heartbeat_pending) {
                    current_cmds.heartbeat_pending = false;
                    serial_send_heartbeat_ack(
                        current_cmds.last_heartbeat_ms,
                        current_cmds.heartbeat_seq,
                        current_cmds.mode,
                        0U);
                }
            }
        } while (res == 1);
    }

    /* 1.5 -- SystemCommand pendente */
    if (current_cmds.syscmd_pending == (int8_t)SYSCMD_SOFT_RESET) {
        NVIC_SystemReset();
    }
    if (current_cmds.syscmd_pending == (int8_t)SYSCMD_ENTER_BOOTLOADER) {
        /* OPENCR: sem mini-boot IAP portado (ver bootflag.h) -- isto
         * hoje só reseta normalmente. Reflash real é por USB. */
        bootflag_request_bootloader();
        NVIC_SystemReset();
    }

    /* 2 -- IMU @ 100 Hz */
    imu_read_and_filter(&imu_state, tick_dt);

    /* 2.5 -- Stabilizer + joint executor @ 100 Hz */
    stabilizer_compute(&stab_params, &stab_state,
                       imu_state.pitch, imu_state.roll,
                       -imu_state.gyro_y, imu_state.gyro_x,
                       tick_dt, &stab_output);
    (void)stab_output;

    joint_exec_tick(&joint_exec, current_cmds.mode, tick_count,
                    joint_goals, SAFE_POSE);
    if (current_cmds.mode != (uint8_t)MODE_JOINT_STREAM
        && current_cmds.keyframe_active) {
        uint32_t mask = current_cmds.kf_torque_mask;
        for (uint8_t i = 0U; i < 18U; i++) {
            if ((mask & (1U << i)) != 0U) {
                joint_goals[i] = current_cmds.kf_goals[i];
            }
        }
    }

    /* 3 -- Servo update @ 50 Hz */
    if ((tick_count % 2U) == 0U && servo_count_found > 0U) {
        dxl_sync_write_positions(joint_goals);

        uint32_t t0 = millis();
        dxl_read_all(joint_positions, joint_loads,
                     joint_voltages, joint_temperatures, joint_errors,
                     servo_present);
        dxl_read_ms  = millis() - t0;
    }

    /* 3b -- Soft-start supervisor @ 50 Hz */
    if ((tick_count % 2U) == 1U && servo_count_found > 0U) {
        soft_start_tick(soft_start_instance(), joint_goals,
                        joint_positions, servo_present, millis());
        (void)soft_start_flush(soft_start_instance(), servo_present,
                               4U, speed_write_cb);
    }

    /* 4 -- LED heartbeat @ 1 Hz */
    if ((tick_count % 50U) == 0U) {
        digitalWrite(BDPIN_LED_USER_1, !digitalRead(BDPIN_LED_USER_1));
    }

    /* 5 -- Bateria (getPowerInVoltage(), nao-bloqueante) */
    {
        uint16_t millivolts;
        if (battery_hw_sample(&millivolts)) {
            battery_mv = battery_filter_apply(&battery_filter, millivolts);
        }
    }

    /* 7 -- Binary telemetry uplink */
    serial_send_imu(&imu_state);

    if ((tick_count % 2U) == 0U) {
        serial_send_joint_state(joint_positions, joint_loads,
                                joint_voltages, joint_temperatures,
                                joint_errors);
    }

    if ((tick_count % 10U) == 0U) {
        uint32_t loop_us = (millis() - tick_start_ms) * 1000U;
        uint8_t stream_watchdog_tripped =
            (current_cmds.mode == (uint8_t)MODE_JOINT_STREAM
             && joint_exec.stream_active
             && ((uint32_t)(tick_count - joint_exec.last_stream_tick) > 10U))
                ? 1U : 0U;
        safety_update(&safety_state, joint_temperatures, servo_present,
                      stream_watchdog_tripped);
        serial_send_system_status(
            current_cmds.mode,
            safety_state.error_flags,
            tick_count * 10U,
            battery_mv,
            loop_us);
    }
}
