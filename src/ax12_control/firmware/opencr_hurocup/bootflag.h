#ifndef BOOTFLAG_H
#define BOOTFLAG_H

/* OPENCR: reescrito. O original gravava um "magic" em BKP_DR1 (registrador
 * de backup do F103) que sobrevive a reset por software; o mini-boot IAP
 * (ADR-0032 do projeto original) lia esse registrador no próximo boot pra
 * decidir ficar no servidor de gravação da USART2 — mecanismo que existia
 * porque o bootloader ROM do F103 ficou preso no barramento de motores.
 *
 * Na OpenCR você grava por USB (Arduino IDE) — esse problema não existe,
 * e o CONSUMIDOR do mini-boot (a leitura da flag no próximo boot decidindo
 * ficar num modo de gravação especial) NÃO foi portado, não tem função
 * aqui. Esta API foi mantida só por completude/fidelidade de interface —
 * ela grava/lê um registrador de backup do RTC do F7 (sobrevive a reset,
 * mesma garantia do BKP_DR1 original), mas nada no firmware lê essa flag
 * no boot hoje. NÃO VERIFICADO em hardware: confirme que o core da OpenCR
 * permite inicializar a RTC deste jeito antes de confiar nisto.
 */
#ifdef __cplusplus
extern "C" {
#endif

void bootflag_request_bootloader(void);
void bootflag_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* BOOTFLAG_H */
