#ifndef BATTERY_HW_H
#define BATTERY_HW_H

#include <stdbool.h>
#include <stdint.h>

/* OPENCR: reescrito. O original lia um divisor resistivo via ADC1
 * (nunca fiado no hardware, ficava atrás de BATTERY_ADC_ENABLE=0). A
 * OpenCR tem leitura de tensão de alimentação própria e testada
 * (getPowerInVoltage(), da lib oficial) — battery_hw_sample() agora
 * devolve MILIVOLTS diretamente, não uma contagem de ADC de 12 bits.
 * Por isso o main loop não passa mais pelo battery_counts_to_mv() (ver
 * nota em battery.h) — chama battery_filter_apply() direto com o valor
 * em mV. Interface (nome/assinatura) mantida igual ao original por
 * fidelidade, só o significado do parâmetro `counts` mudou (é mV, não
 * contagem crua) — documentado aqui para não confundir quem só olhar
 * o .h. */

void battery_hw_init(void);

/* Não-bloqueante; chamar 1x por tick. Retorna true quando *millivolts
 * recebeu uma amostra nova (tensão de alimentação em mV). */
bool battery_hw_sample(uint16_t *millivolts);

#endif /* BATTERY_HW_H */
