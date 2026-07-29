/* OPENCR: reescrito (ver battery_hw.h). Arquivo .cpp (em vez de .c) só
 * porque getPowerInVoltage() é da API do core Arduino da OpenCR — o
 * wrapper extern "C" abaixo deixa o resto do firmware (100% C) chamar
 * battery_hw_init()/battery_hw_sample() exatamente como no original. */
#include <Arduino.h>
#include "battery_hw.h"
#include "config.h"

#if BATTERY_ADC_ENABLE

extern "C" void battery_hw_init(void)
{
    /* getPowerInVoltage() não precisa de setup — nada a fazer aqui. */
}

extern "C" bool battery_hw_sample(uint16_t *millivolts)
{
    /* getPowerInVoltage() é síncrona (sem o start/poll-EOC do ADC1
     * original), então cada chamada já entrega uma amostra pronta —
     * ao contrário do original não há estado "in_flight" pra manter. */
    float volts = getPowerInVoltage();
    if (volts < 0.0f) { volts = 0.0f; }
    float mv = volts * 1000.0f;
    if (mv > 65535.0f) { mv = 65535.0f; }
    *millivolts = (uint16_t)mv;
    return true;
}

#else /* !BATTERY_ADC_ENABLE */

extern "C" void battery_hw_init(void)
{
}

extern "C" bool battery_hw_sample(uint16_t *millivolts)
{
    (void)millivolts;
    return false;
}

#endif /* BATTERY_ADC_ENABLE */
