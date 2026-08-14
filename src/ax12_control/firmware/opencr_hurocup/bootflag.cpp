/* OPENCR: reescrito — ver aviso em bootflag.h. Usa o registrador de
 * backup 0 do RTC (HAL_RTCEx_BKUPWrite/Read), equivalente funcional do
 * BKP_DR1 do F103 original, mas em outra família de periférico.
 *
 * NÃO VERIFICADO EM HARDWARE — o core da OpenCR pode já inicializar uma
 * RTC própria em outro lugar; inicializar de novo aqui com parâmetros
 * "razoáveis" pode conflitar. Se o build ou o boot falhar por causa
 * deste arquivo, o mais simples é isolar esta função: nada no firmware
 * hoje DEPENDE dela funcionar (ver bootflag.h) — pode virar um no-op
 * temporário sem perder nenhuma funcionalidade real.
 */
#include "bootflag.h"
#include <stdint.h>

extern "C" {
#include "stm32f7xx_hal.h"
}

#define BOOTFLAG_MAGIC   0xB007U
#define BOOTFLAG_BKP_REG RTC_BKP_DR0

static bool bootflag_rtc_ready(RTC_HandleTypeDef *hrtc)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_RTC_ENABLE();

    hrtc->Instance = RTC;
    hrtc->Init.HourFormat     = RTC_HOURFORMAT_24;
    hrtc->Init.AsynchPrediv   = 127;
    hrtc->Init.SynchPrediv    = 255;
    hrtc->Init.OutPut         = RTC_OUTPUT_DISABLE;
    hrtc->Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc->Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
    /* HAL_RTC_Init reconfigura só se a RTC ainda não estiver rodando com
     * esses parâmetros; backup registers sobrevivem independentemente. */
    return HAL_RTC_Init(hrtc) == HAL_OK;
}

static void bootflag_write(uint32_t value)
{
    RTC_HandleTypeDef hrtc = {0};
    if (!bootflag_rtc_ready(&hrtc)) { return; }
    HAL_RTCEx_BKUPWrite(&hrtc, BOOTFLAG_BKP_REG, value);
}

extern "C" void bootflag_request_bootloader(void)
{
    bootflag_write(BOOTFLAG_MAGIC);
}

extern "C" void bootflag_clear(void)
{
    bootflag_write(0U);
}
