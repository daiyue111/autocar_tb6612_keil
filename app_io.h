#ifndef APP_IO_H
#define APP_IO_H

#include "ti_msp_dl_config.h"

void app_delay_ms(uint32_t ms);
void app_gpio_write(GPIO_Regs *port, uint32_t pin, bool high);
bool app_gpio_read(GPIO_Regs *port, uint32_t pin);
bool app_start_key_pressed(void);
void app_wait_start_key(void);
void app_beep_short(void);
void app_led_blink(uint8_t count);
void app_outputs_idle(void);

#endif
