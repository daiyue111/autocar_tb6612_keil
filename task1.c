#include "task1.h"

#include "app_io.h"
#include "motor.h"
#include "track.h"

#define TASK1_LEFT_DUTY 25U
#define TASK1_RIGHT_DUTY 21U
#define TASK1_START_IGNORE_MS 1000U
#define TASK1_LINE_CONFIRM_MS 1U
#define TASK1_MAX_MS 15000U

void task1_run(void)
{
    uint16_t lineMs = 0U;

    for (uint32_t t = 0U; t < TASK1_MAX_MS; t++) {
        uint8_t mask = track_read_mask();

        if ((t >= TASK1_START_IGNORE_MS) && (mask != 0U)) {
            if (lineMs < TASK1_LINE_CONFIRM_MS) {
                lineMs++;
            }
        } else {
            lineMs = 0U;
        }

        if (lineMs >= TASK1_LINE_CONFIRM_MS) {
            motor_short_brake_ms(120U);
            app_beep_short();
            app_led_blink(1U);
            return;
        }

        motor_forward_pwm_1ms(TASK1_LEFT_DUTY, TASK1_RIGHT_DUTY);
    }

    motor_active_brake_then_stop();
    app_led_blink(3U);
}
