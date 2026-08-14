/* Porte 1:1 do original — lógica pura, nenhuma mudança.
 * (No caminho de dados da OpenCR, battery_counts_to_mv* não é chamada —
 * ver nota em battery.h/config.h. Mantida para fidelidade ao módulo.) */
#include "battery.h"
#include "config.h"

uint16_t battery_counts_to_mv_ex(uint16_t counts, uint16_t vref_mv,
                                 uint16_t num, uint16_t den)
{
    if (den == 0U) { return 0U; }
    if (counts > 4095U) { counts = 4095U; }

    uint64_t numer = (uint64_t)counts * (uint64_t)vref_mv * (uint64_t)num;
    uint64_t denom = 4095ULL * (uint64_t)den;
    uint64_t mv    = (numer + denom / 2ULL) / denom;

    if (mv > 65535ULL) { mv = 65535ULL; }
    return (uint16_t)mv;
}

uint16_t battery_counts_to_mv(uint16_t counts)
{
    return battery_counts_to_mv_ex(counts, BATTERY_ADC_VREF_MV,
                                   BATTERY_DIVIDER_NUM, BATTERY_DIVIDER_DEN);
}

void battery_filter_init(battery_filter_t *f)
{
    f->acc    = 0U;
    f->primed = 0U;
}

uint16_t battery_filter_apply(battery_filter_t *f, uint16_t mv_sample)
{
    if (!f->primed) {
        f->acc    = (uint32_t)mv_sample << 3;
        f->primed = 1U;
    } else {
        f->acc = f->acc - (f->acc >> 3) + mv_sample;
    }
    return (uint16_t)(f->acc >> 3);
}
