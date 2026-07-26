#include "control_scheduler.h"

#include "ti_msp_dl_config.h"

static volatile bool gTickReady;
static volatile uint32_t gSystemMs;

void control_scheduler_init(void)
{
    gTickReady = false;
    gSystemMs = 0U;
    (void)DL_SYSTICK_config(CPUCLK_FREQ / 1000U);
}

bool control_scheduler_take_1ms(void)
{
    bool ready;

    __disable_irq();
    ready = gTickReady;
    gTickReady = false;
    __enable_irq();
    return ready;
}

uint32_t control_scheduler_now_ms(void)
{
    return gSystemMs;
}

void SysTick_Handler(void)
{
    gSystemMs++;
    gTickReady = true;
}
