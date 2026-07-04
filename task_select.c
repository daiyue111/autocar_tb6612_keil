#include "task_select.h"

#include "app_io.h"
#include "task1.h"
#include "task2.h"
#include "task3.h"
#include "task4.h"

#define SELECT_MAX_TASK 4U
#define SELECT_DEBOUNCE_MS 40U
#define SELECT_RELEASE_DEBOUNCE_MS 40U
#define SELECT_LOCK_IDLE_MS 1200U
#define SELECT_LED_CONFIRM_MS 80U

static void led_confirm_press(void)
{
    app_gpio_write(LED_PORT, LED_PIN, true);
    app_delay_ms(SELECT_LED_CONFIRM_MS);
    app_gpio_write(LED_PORT, LED_PIN, false);
}

static void wait_key_release(void)
{
    uint16_t releaseMs = 0U;

    while (releaseMs < SELECT_RELEASE_DEBOUNCE_MS) {
        if (!app_start_key_pressed()) {
            releaseMs += 10U;
        } else {
            releaseMs = 0U;
        }
        app_delay_ms(10U);
    }
}

static uint8_t wait_task_number(void)
{
    uint8_t count = 0U;
    uint16_t idleMs = 0U;

    while ((count == 0U) || (idleMs < SELECT_LOCK_IDLE_MS)) {
        if (app_start_key_pressed()) {
            uint16_t pressedMs = 0U;

            while (pressedMs < SELECT_DEBOUNCE_MS) {
                if (app_start_key_pressed()) {
                    pressedMs += 10U;
                } else {
                    pressedMs = 0U;
                }
                app_delay_ms(10U);
            }

            if (count < SELECT_MAX_TASK) {
                count++;
                led_confirm_press();
            }
            wait_key_release();
            idleMs = 0U;
        } else {
            if (count > 0U) {
                idleMs += 10U;
            }
            app_delay_ms(10U);
        }
    }

    return count;
}

void task_select_run(void)
{
    uint8_t task = wait_task_number();

    app_led_blink(task);
    app_delay_ms(250U);

    switch (task) {
    case 1U:
        task1_run();
        break;
    case 2U:
        task2_run();
        break;
    case 3U:
        task3_run();
        break;
    case 4U:
        task4_run();
        break;
    default:
        app_led_blink(4U);
        break;
    }
}
