#include "ti_msp_dl_config.h"

#define TRACK_BLACK_IS_HIGH 1U

#define STRAIGHT_LEFT_DUTY 56U
#define STRAIGHT_RIGHT_DUTY 50U
#define START_IGNORE_MS 1200U
#define STOP_CENTER_MASK 0x3CU
#define STOP_MIN_CENTER_COUNT 4U
#define STOP_CONFIRM_MS 80U
#define BRAKE_DUTY 80U
#define BRAKE_TIME_MS 120U
#define DIAG_PULSE_DUTY 60U
#define DIAG_PULSE_MS 120U

#define PWM_PERIOD_TICKS 200U
#define PWM_STEP_DELAY_CYCLES (CPUCLK_FREQ / 200000U)

static void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_cycles(CPUCLK_FREQ / 1000U);
    }
}

static void gpio_write(GPIO_Regs *port, uint32_t pin, bool high)
{
    if (high) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

static bool gpio_read(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0U);
}

static bool key_pressed(void)
{
    return !gpio_read(KEY_START_PORT, KEY_START_PIN);
}

static void wait_start_key(void)
{
    uint16_t pressedMs = 0;

    while (pressedMs < 300U) {
        if (key_pressed()) {
            pressedMs += 10U;
        } else {
            pressedMs = 0U;
        }
        delay_ms(10U);
    }

    while (key_pressed()) {
        delay_ms(10U);
    }
    delay_ms(300U);
}

static void motors_safe_stop(void)
{
    gpio_write(MOTOR_PWMA_PORT, MOTOR_PWMA_PIN, false);
    gpio_write(MOTOR_PWMB_PORT, MOTOR_PWMB_PIN, false);
    gpio_write(MOTOR_PWMC_PORT, MOTOR_PWMC_PIN, false);
    gpio_write(MOTOR_PWMD_PORT, MOTOR_PWMD_PIN, false);
    gpio_write(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, false);
    gpio_write(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, false);
    gpio_write(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, false);
    gpio_write(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, false);
    gpio_write(MOTOR_CIN1_PORT, MOTOR_CIN1_PIN, false);
    gpio_write(MOTOR_CIN2_PORT, MOTOR_CIN2_PIN, false);
    gpio_write(MOTOR_DIN1_PORT, MOTOR_DIN1_PIN, false);
    gpio_write(MOTOR_DIN2_PORT, MOTOR_DIN2_PIN, false);
}

static void motors_forward_dir(void)
{
    gpio_write(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, true);
    gpio_write(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, false);
    gpio_write(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, true);
    gpio_write(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, false);
    gpio_write(MOTOR_CIN1_PORT, MOTOR_CIN1_PIN, false);
    gpio_write(MOTOR_CIN2_PORT, MOTOR_CIN2_PIN, true);
    gpio_write(MOTOR_DIN1_PORT, MOTOR_DIN1_PIN, false);
    gpio_write(MOTOR_DIN2_PORT, MOTOR_DIN2_PIN, true);
}

static void motors_backward_dir(void)
{
    gpio_write(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, false);
    gpio_write(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, true);
    gpio_write(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, false);
    gpio_write(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, true);
    gpio_write(MOTOR_CIN1_PORT, MOTOR_CIN1_PIN, true);
    gpio_write(MOTOR_CIN2_PORT, MOTOR_CIN2_PIN, false);
    gpio_write(MOTOR_DIN1_PORT, MOTOR_DIN1_PIN, true);
    gpio_write(MOTOR_DIN2_PORT, MOTOR_DIN2_PIN, false);
}

static void pwm_run_1ms(uint8_t leftDuty, uint8_t rightDuty)
{
    for (uint32_t tick = 0; tick < PWM_PERIOD_TICKS; tick++) {
        bool leftOn = (tick < leftDuty);
        bool rightOn = (tick < rightDuty);

        gpio_write(MOTOR_PWMB_PORT, MOTOR_PWMB_PIN, leftOn);
        gpio_write(MOTOR_PWMC_PORT, MOTOR_PWMC_PIN, leftOn);
        gpio_write(MOTOR_PWMA_PORT, MOTOR_PWMA_PIN, rightOn);
        gpio_write(MOTOR_PWMD_PORT, MOTOR_PWMD_PIN, rightOn);
        delay_cycles(PWM_STEP_DELAY_CYCLES);
    }
}

static uint8_t track_read_mask(void)
{
    uint8_t mask = 0;
    bool x1 = gpio_read(TRACK_X1_PORT, TRACK_X1_PIN);
    bool x2 = gpio_read(TRACK_X2_PORT, TRACK_X2_PIN);
    bool x3 = gpio_read(TRACK_X3_PORT, TRACK_X3_PIN);
    bool x4 = gpio_read(TRACK_X4_PORT, TRACK_X4_PIN);
    bool x5 = gpio_read(TRACK_X5_PORT, TRACK_X5_PIN);
    bool x6 = gpio_read(TRACK_X6_PORT, TRACK_X6_PIN);
    bool x7 = gpio_read(TRACK_X7_PORT, TRACK_X7_PIN);
    bool x8 = gpio_read(TRACK_X8_PORT, TRACK_X8_PIN);

#if !TRACK_BLACK_IS_HIGH
    x1 = !x1;
    x2 = !x2;
    x3 = !x3;
    x4 = !x4;
    x5 = !x5;
    x6 = !x6;
    x7 = !x7;
    x8 = !x8;
#endif

    if (x1) { mask |= 0x01U; }
    if (x2) { mask |= 0x02U; }
    if (x3) { mask |= 0x04U; }
    if (x4) { mask |= 0x08U; }
    if (x5) { mask |= 0x10U; }
    if (x6) { mask |= 0x20U; }
    if (x7) { mask |= 0x40U; }
    if (x8) { mask |= 0x80U; }

    return mask;
}

static uint8_t bit_count(uint8_t mask)
{
    uint8_t count = 0;

    for (uint8_t i = 0; i < 8U; i++) {
        if ((mask & (uint8_t)(1U << i)) != 0U) {
            count++;
        }
    }

    return count;
}

static bool b_line_detected(void)
{
    uint8_t centerMask = track_read_mask() & STOP_CENTER_MASK;

    return bit_count(centerMask) >= STOP_MIN_CENTER_COUNT;
}

static void motor_a_forward(void)
{
    gpio_write(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, true);
    gpio_write(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, false);
}

static void pulse_a(uint32_t ms)
{
    motor_a_forward();

    for (uint32_t i = 0; i < ms; i++) {
        for (uint32_t tick = 0; tick < PWM_PERIOD_TICKS; tick++) {
            gpio_write(MOTOR_PWMA_PORT, MOTOR_PWMA_PIN,
                tick < DIAG_PULSE_DUTY);
            delay_cycles(PWM_STEP_DELAY_CYCLES);
        }
    }

    motors_safe_stop();
}

static void report_stop_mask(uint8_t stopMask)
{
    uint8_t centerCount = bit_count(stopMask & STOP_CENTER_MASK);

    delay_ms(800U);

    if (centerCount == 0U) {
        delay_ms(1200U);
        return;
    }

    for (uint8_t i = 0; i < centerCount; i++) {
        pulse_a(DIAG_PULSE_MS);
        delay_ms(350U);
    }
}

static void active_brake(void)
{
    motors_backward_dir();

    for (uint32_t i = 0; i < BRAKE_TIME_MS; i++) {
        pwm_run_1ms(BRAKE_DUTY, BRAKE_DUTY);
    }

    motors_safe_stop();
}

int main(void)
{
    uint32_t stopConfirmMs = 0;
    uint8_t stopMask = 0;

    SYSCFG_DL_init();
    gpio_write(MOTOR_STBY_PORT, MOTOR_STBY_PIN, true);
    motors_safe_stop();

    wait_start_key();
    motors_forward_dir();

    for (uint32_t t = 0; ; t++) {
        uint8_t currentMask = track_read_mask();
        uint8_t centerMask = currentMask & STOP_CENTER_MASK;

        if ((t > START_IGNORE_MS) &&
            (bit_count(centerMask) >= STOP_MIN_CENTER_COUNT)) {
            stopConfirmMs++;
            stopMask = currentMask;
        } else {
            stopConfirmMs = 0U;
        }

        if (stopConfirmMs >= STOP_CONFIRM_MS) {
            break;
        }

        pwm_run_1ms(STRAIGHT_LEFT_DUTY, STRAIGHT_RIGHT_DUTY);
    }

    active_brake();
    report_stop_mask(stopMask);

    while (1) {
        motors_safe_stop();
        delay_ms(20U);
    }
}
