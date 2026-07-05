#include "app_io.h"

#define BEEP_ENABLE 1U
#define BEEP_OFF_LEVEL true
#define BEEP_HALF_PERIOD_CYCLES (CPUCLK_FREQ / 4000U)

void app_delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_cycles(CPUCLK_FREQ / 1000U);
    }
}

void app_gpio_write(GPIO_Regs *port, uint32_t pin, bool high)
{
    if (high) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

bool app_gpio_read(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0U);
}

bool app_start_key_pressed(void)
{
    return !app_gpio_read(KEY_START_PORT, KEY_START_PIN);
}

void app_wait_start_key(void)
{
    uint16_t pressedMs = 0;

    while (pressedMs < 300U) {
        if (app_start_key_pressed()) {
            pressedMs += 10U;
        } else {
            pressedMs = 0U;
        }
        app_delay_ms(10U);
    }

    while (app_start_key_pressed()) {
        app_delay_ms(10U);
    }
    app_delay_ms(250U);
}

void app_beep_short(void)
{
#if BEEP_ENABLE
    for (uint32_t i = 0; i < 180U; i++) {
        app_gpio_write(BEEP_PORT, BEEP_PIN, true);
        delay_cycles(BEEP_HALF_PERIOD_CYCLES);
        app_gpio_write(BEEP_PORT, BEEP_PIN, false);
        delay_cycles(BEEP_HALF_PERIOD_CYCLES);
    }
#endif
    app_gpio_write(BEEP_PORT, BEEP_PIN, BEEP_OFF_LEVEL);
}

void app_led_blink(uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        app_gpio_write(LED_PORT, LED_PIN, true);
        app_delay_ms(180U);
        app_gpio_write(LED_PORT, LED_PIN, false);
        app_delay_ms(180U);
    }
}

void app_outputs_idle(void)
{
    app_gpio_write(LED_PORT, LED_PIN, false);
    app_gpio_write(BEEP_PORT, BEEP_PIN, BEEP_OFF_LEVEL);
}
