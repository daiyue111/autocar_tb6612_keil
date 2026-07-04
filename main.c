#include "app_io.h"
#include "motor.h"
#include "task_select.h"

int main(void)
{
    SYSCFG_DL_init();
    app_gpio_write(MOTOR_STBY_PORT, MOTOR_STBY_PIN, true);
    app_outputs_idle();
    motor_safe_stop();

    task_select_run();

    while (1) {
        motor_safe_stop();
        app_delay_ms(20U);
    }
}
