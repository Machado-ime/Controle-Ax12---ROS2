#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/* =====================================================================
 * config.h — porte do firmware HuroCup (STM32F103C6/Bluepill) para a
 * ROBOTIS OpenCR (STM32F746ZGT6). Mantido o mais parecido possível com
 * o original (mesmas constantes de protocolo/timing/AX-12A) — só o que
 * é fisicamente diferente na placa foi alterado, e está marcado com
 * "OPENCR:" nos comentários.
 * ===================================================================== */

/* ===== Servos ===== */
#define SERVO_COUNT       18
#define DXL_BUF_SIZE      32

/* ===== Portas seriais — OPENCR: mapeamento novo =====================
 * Original (Bluepill): USART1/PA9 HDSEL (motores) + USART2/PA2-PA3 (Jetson).
 * OpenCR não tem HDSEL disponível do mesmo jeito nem a mesma pinagem;
 * usa-se o padrão já empregado no firmware oficial da ROBOTIS
 * (usb_to_dxl / opencr_op3):
 *   Serial3 (DXL_PORT) -> barramento Dynamixel, half-duplex via
 *     drv_dxl_tx_enable() (pino dedicado + transistor na própria placa,
 *     não um bit de periférico) em vez do HDSEL do F103.
 *   Serial1 -> link com o host (Jetson/PC), EQUIVALENTE ao USART2
 *     original. Serial1 é uma UART de verdade (não a USB/CDC) —
 *     confirme no seu conector qual header físico é Serial1 antes de
 *     ligar o host nele; se preferir usar a própria USB (Serial) como
 *     link do host, troque HOST_PORT abaixo.
 * ===================================================================== */
#define USART_JETSON_BAUD  460800UL
#define USART_DXL_BAUD     1000000UL

/* ===== IMU — OPENCR: chip e barramento diferentes ====================
 * Original: BMI160 por I2C1 bit-banged manualmente (endereço 0x69).
 * OpenCR: MPU9250 (placas até ~2020) ou ICM-20648 (2020+), acessados
 * pela classe oficial cIMU (<IMU.h>) — sem I2C manual, sem endereço
 * fixo aqui. Mantidos só os fatores de conversão (ver imu.c) que a
 * classe cIMU expõe em unidades cruas equivalentes.
 * ===================================================================== */

/* ===== Timer ===== */
#define TIM2_FREQ_HZ      100U

/* ===== Ring buffer ===== */
#define RING_BUF_SIZE     256U

/* ===== FSM states (idêntico ao original) ===== */
typedef enum {
    MODE_INIT         = 0,
    MODE_IDLE         = 1,
    MODE_ACTIVE       = 2,
    MODE_SAFE_STOP    = 3,
    MODE_EMERGENCY    = 4,
    MODE_JOINT_STREAM = 5
} fsm_state_t;

/* ===== AX-12A conversion constants (idêntico ao original) ============
 * Full range 300° = 5.236 rad over 0–1023 ticks. Centre = 512.
 * ================================================================ */
#define AX12A_TICKS_PER_RAD  195.38f   /* 1023.0f / (300° × π/180°) */
#define AX12A_CENTER         512U
#define AX12A_MIN            0U
#define AX12A_MAX            1023U

/* ===== Boot soft-start (idêntico ao original — ver rationale lá) ===== */
#define BOOT_MOVING_SPEED    90U

/* ===== Soft-start supervisor (idêntico ao original) ================= */
#define SOFT_START_JUMP_TICKS   150U
#define SOFT_START_DONE_TICKS   25U
#define SOFT_START_TIMEOUT_MS   4000UL

/* ===== Bateria — OPENCR: ADC próprio descartado ======================
 * Original lia um divisor resistivo via ADC1 (desligado por padrão,
 * nunca fiado no hardware). A OpenCR já tem leitura de tensão de
 * alimentação própria e testada (getPowerInVoltage()) — battery_hw.c
 * agora é um wrapper fino em cima dela, sem ADC manual. As constantes
 * de divisor/canal do original NÃO se aplicam mais e foram removidas;
 * battery.c (filtro EMA) continua idêntico, só troca a fonte da amostra.
 * ================================================================ */
#ifndef BATTERY_ADC_ENABLE
#define BATTERY_ADC_ENABLE   1   /* OPENCR: sempre disponível via getPowerInVoltage() */
#endif

/* Vestigiais: battery.c mantém battery_counts_to_mv()/_ex() idênticas ao
 * original (módulo puro, sem motivo pra mudar) e essas macros existem só
 * pra ela compilar. O caminho de dados real da OpenCR NÃO passa por elas
 * — battery_hw.c já devolve milivolts direto de getPowerInVoltage(),
 * sem contagem de ADC/divisor pra converter. */
#ifndef BATTERY_ADC_VREF_MV
#define BATTERY_ADC_VREF_MV  3300U
#endif
#ifndef BATTERY_DIVIDER_NUM
#define BATTERY_DIVIDER_NUM  1U
#endif
#ifndef BATTERY_DIVIDER_DEN
#define BATTERY_DIVIDER_DEN  1U
#endif

/* ===== SystemCommand / IWDG (idêntico em espírito; ver bootflag.h) ===
 * OPENCR: o watchdog independente (IWDG) existe igual no F7; o período
 * nominal abaixo é o mesmo valor do original — reavalie se a fonte de
 * clock do IWDG (LSI) tiver tolerância diferente documentada pra F7.
 * ================================================================ */
#define IWDG_PERIOD_MS    2000UL

/* ===== bootflag — OPENCR: mecanismo trocado, mesma finalidade =========
 * O mini-boot IAP original (ADR-0032) existia porque o bootloader ROM
 * do F103 ficou preso no barramento de motores (USART1) e o reflash
 * remoto precisava de uma rota alternativa por UART. Na OpenCR você
 * grava por USB (Arduino IDE) diretamente — esse problema não existe.
 * bootflag.c foi PORTADO MESMO ASSIM (registrador de backup do RTC no
 * F7, no lugar do BKP_DR1 do F1) porque a API "grava/lê uma flag que
 * sobrevive a reset" é utilidade genérica (ex.: detectar reset após
 * EMERGENCY); o consumidor real do mini-boot (boot/an3155.c,
 * boot/boot_main.c, boot/bootdecide.c) NÃO foi portado — não tem
 * função na OpenCR. Veja bootflag.h.
 * ================================================================ */
#define BOOTFLAG_MAGIC    0xB007U

#endif /* CONFIG_H */
