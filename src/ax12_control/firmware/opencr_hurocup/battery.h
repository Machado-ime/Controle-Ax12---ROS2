#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>

/* Módulo PURO — porte 1:1 do original: conversão da contagem do ADC
 * (12 bits) para tensão do pack em mV e suavização EMA leve. Nada aqui
 * toca registrador — o acesso físico (agora getPowerInVoltage() da
 * OpenCR em vez de ADC1) vive em battery_hw.c. */

uint16_t battery_counts_to_mv_ex(uint16_t counts, uint16_t vref_mv,
                                 uint16_t num, uint16_t den);

uint16_t battery_counts_to_mv(uint16_t counts);

/* EMA inteira com alpha = 1/8 (acumulador em Q3): suavização LEVE de
 * propósito — afundamento sob carga deve continuar visível na
 * telemetria. Regime permanente é exato (saída cravada na amostra). */
typedef struct {
    uint32_t acc;     /* ~ 8 x média corrente (Q3) */
    uint8_t  primed;  /* 0 até a primeira amostra */
} battery_filter_t;

void     battery_filter_init(battery_filter_t *f);
uint16_t battery_filter_apply(battery_filter_t *f, uint16_t mv_sample);

#endif /* BATTERY_H */
