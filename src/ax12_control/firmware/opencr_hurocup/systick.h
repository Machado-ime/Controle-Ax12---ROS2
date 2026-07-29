#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

/* OPENCR: reescrito. O original configurava o SysTick do Cortex-M
 * manualmente (systick_set_frequency + ISR incrementando systick_ms a
 * 1 kHz) porque não tinha um core/framework por baixo. Na OpenCR, o
 * core Arduino já mantém millis() rodando desde o boot — get_ticks()
 * agora é só um wrapper.
 *
 * systick_ms (a variável, não a função) é referenciada diretamente por
 * command_handler.c original — mantida aqui como um espelho de
 * millis() atualizado a cada iteração do loop principal (ver .ino).
 * Não é mais um contador de ISR de 1 ms "ao vivo" em todo instante,
 * mas para o uso real (timestamp de heartbeat) a diferença é
 * irrelevante — o loop principal roda bem mais rápido que 1 ms.
 */
#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t systick_ms;

uint32_t get_ticks(void);
void     delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* SYSTICK_H */
