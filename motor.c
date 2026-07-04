#include "motor.h"
#include "app_io.h"

#define PWM_PERIOD_TICKS 200U
#define PWM_STEP_DELAY_CYCLES (CPUCLK_FREQ / 200000U)

void motor_safe_stop(void)
{
    app_gpio_write(MOTOR_PWMA_PORT, MOTOR_PWMA_PIN, false);
    app_gpio_write(MOTOR_PWMB_PORT, MOTOR_PWMB_PIN, false);
    app_gpio_write(MOTOR_PWMC_PORT, MOTOR_PWMC_PIN, false);
    app_gpio_write(MOTOR_PWMD_PORT, MOTOR_PWMD_PIN, false);

    app_gpio_write(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, false);
    app_gpio_write(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, false);
    app_gpio_write(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, false);
    app_gpio_write(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, false);
    app_gpio_write(MOTOR_CIN1_PORT, MOTOR_CIN1_PIN, false);
    app_gpio_write(MOTOR_CIN2_PORT, MOTOR_CIN2_PIN, false);
    app_gpio_write(MOTOR_DIN1_PORT, MOTOR_DIN1_PIN, false);
    app_gpio_write(MOTOR_DIN2_PORT, MOTOR_DIN2_PIN, false);
}

void motor_forward_dir(void)
{
    app_gpio_write(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, true);
    app_gpio_write(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, false);
    app_gpio_write(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, true);
    app_gpio_write(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, false);
    app_gpio_write(MOTOR_CIN1_PORT, MOTOR_CIN1_PIN, false);
    app_gpio_write(MOTOR_CIN2_PORT, MOTOR_CIN2_PIN, true);
    app_gpio_write(MOTOR_DIN1_PORT, MOTOR_DIN1_PIN, false);
    app_gpio_write(MOTOR_DIN2_PORT, MOTOR_DIN2_PIN, true);
}

void motor_pwm_off(void)
{
    app_gpio_write(MOTOR_PWMA_PORT, MOTOR_PWMA_PIN, false);
    app_gpio_write(MOTOR_PWMB_PORT, MOTOR_PWMB_PIN, false);
    app_gpio_write(MOTOR_PWMC_PORT, MOTOR_PWMC_PIN, false);
    app_gpio_write(MOTOR_PWMD_PORT, MOTOR_PWMD_PIN, false);
}

void motor_pwm_run_1ms(uint8_t leftDuty, uint8_t rightDuty)
{
    uint32_t leftOn = ((uint32_t)leftDuty * PWM_PERIOD_TICKS) / 100U;
    uint32_t rightOn = ((uint32_t)rightDuty * PWM_PERIOD_TICKS) / 100U;

    for (uint32_t t = 0; t < PWM_PERIOD_TICKS; t++) {
        bool leftHigh = (t < leftOn);
        bool rightHigh = (t < rightOn);

        app_gpio_write(MOTOR_PWMA_PORT, MOTOR_PWMA_PIN, leftHigh);
        app_gpio_write(MOTOR_PWMD_PORT, MOTOR_PWMD_PIN, leftHigh);
        app_gpio_write(MOTOR_PWMB_PORT, MOTOR_PWMB_PIN, rightHigh);
        app_gpio_write(MOTOR_PWMC_PORT, MOTOR_PWMC_PIN, rightHigh);
        delay_cycles(PWM_STEP_DELAY_CYCLES);
    }
}

void motor_active_brake_then_stop(void)
{
    motor_pwm_off();
    motor_safe_stop();
    app_delay_ms(80U);
}

void motor_short_brake_ms(uint32_t ms)
{
    app_gpio_write(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, true);
    app_gpio_write(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, true);
    app_gpio_write(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, true);
    app_gpio_write(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, true);
    app_gpio_write(MOTOR_CIN1_PORT, MOTOR_CIN1_PIN, true);
    app_gpio_write(MOTOR_CIN2_PORT, MOTOR_CIN2_PIN, true);
    app_gpio_write(MOTOR_DIN1_PORT, MOTOR_DIN1_PIN, true);
    app_gpio_write(MOTOR_DIN2_PORT, MOTOR_DIN2_PIN, true);

    app_gpio_write(MOTOR_PWMA_PORT, MOTOR_PWMA_PIN, true);
    app_gpio_write(MOTOR_PWMB_PORT, MOTOR_PWMB_PIN, true);
    app_gpio_write(MOTOR_PWMC_PORT, MOTOR_PWMC_PIN, true);
    app_gpio_write(MOTOR_PWMD_PORT, MOTOR_PWMD_PIN, true);

    app_delay_ms(ms);
    motor_safe_stop();
}

void motor_forward_test(void)
{
    motor_forward_dir();
    for (uint16_t i = 0; i < 1000U; i++) {
        motor_pwm_run_1ms(35U, 35U);
    }
    motor_active_brake_then_stop();
}

void motor_forward_pwm_1ms(uint8_t leftDuty, uint8_t rightDuty)
{
    motor_forward_dir();
    motor_pwm_run_1ms(leftDuty, rightDuty);
}

void motor_spin_right_pwm_1ms(uint8_t duty)
{
    app_gpio_write(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, true);
    app_gpio_write(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, false);
    app_gpio_write(MOTOR_DIN1_PORT, MOTOR_DIN1_PIN, false);
    app_gpio_write(MOTOR_DIN2_PORT, MOTOR_DIN2_PIN, true);

    app_gpio_write(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, false);
    app_gpio_write(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, true);
    app_gpio_write(MOTOR_CIN1_PORT, MOTOR_CIN1_PIN, true);
    app_gpio_write(MOTOR_CIN2_PORT, MOTOR_CIN2_PIN, false);

    motor_pwm_run_1ms(duty, duty);
}

void motor_spin_left_pwm_1ms(uint8_t duty)
{
    app_gpio_write(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, false);
    app_gpio_write(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, true);
    app_gpio_write(MOTOR_DIN1_PORT, MOTOR_DIN1_PIN, true);
    app_gpio_write(MOTOR_DIN2_PORT, MOTOR_DIN2_PIN, false);

    app_gpio_write(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, true);
    app_gpio_write(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, false);
    app_gpio_write(MOTOR_CIN1_PORT, MOTOR_CIN1_PIN, false);
    app_gpio_write(MOTOR_CIN2_PORT, MOTOR_CIN2_PIN, true);

    motor_pwm_run_1ms(duty, duty);
}
