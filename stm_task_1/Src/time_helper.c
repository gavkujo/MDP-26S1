#include "time_helper.h"
#include "stm32f4xx.h"

static uint32_t th_last_us = 0;

void time_helper_init(void)
{
    // enable DWT cycle counter for high-res timing
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    th_last_us = 0;
}

uint32_t time_micros(void)
{
    // CPU freq in MHz -> cycles to µs
    return DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000U);
}

float dt_seconds(void)
{
    uint32_t now_us = time_micros();
    uint32_t diff_us = now_us - th_last_us; // unsigned wrap-safe
    th_last_us = now_us;
    if (diff_us == 0u) diff_us = 1u;
    return diff_us * 1e-6f;
}
