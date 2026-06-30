#include "ti_msp_dl_config.h"

#define TRACK_BLACK_IS_HIGH 0U

#define TASK_MODE 2U

#define STRAIGHT_LEFT_DUTY 53U
#define STRAIGHT_RIGHT_DUTY 60U
#define CD_STRAIGHT_LEFT_DUTY 54U
#define CD_STRAIGHT_RIGHT_DUTY 60U
#define CD_ALIGN_LEFT_DUTY 35U
#define CD_ALIGN_RIGHT_DUTY 72U
#define CD_ALIGN_MS 820U
#define LINE_BASE_LEFT_DUTY 34U
#define LINE_BASE_RIGHT_DUTY 40U
#define LINE_KP 6
#define START_IGNORE_MS 1200U
#define LINE_DETECT_MASK 0xFFU
#define LINE_DETECT_MIN_COUNT 1U
#define CD_LINE_DETECT_MASK 0xFFU
#define CD_LINE_DETECT_MIN_COUNT 1U
#define CD_LINE_IGNORE_MS 120U
#define LINE_DETECT_CONFIRM_MS 1U
#define STRAIGHT_LEAVE_LINE_IGNORE_MS 500U
#define ARC_MIN_RUN_MS 800U
#define ARC_LOST_CONFIRM_MS 120U
#define ARC_LOST_MASK 0xFFU
#define ARC_SETTLE_MS 350U
#define D_ARC_CAPTURE_LEFT_DUTY 32U
#define D_ARC_CAPTURE_RIGHT_DUTY 62U
#define D_ARC_CAPTURE_MS 340U
#define BRAKE_MS 80U
#define BEEP_HALF_PERIOD_CYCLES (CPUCLK_FREQ / 4000U)

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

static void notice_arrived(void)
{
    for (uint32_t i = 0; i < 1000U; i++) {
        gpio_write(BEEP_PORT, BEEP_PIN, true);
        delay_cycles(BEEP_HALF_PERIOD_CYCLES);
        gpio_write(BEEP_PORT, BEEP_PIN, false);
        delay_cycles(BEEP_HALF_PERIOD_CYCLES);
    }

    for (uint8_t i = 0; i < 3U; i++) {
        gpio_write(LED_PORT, LED_PIN, true);
        delay_ms(180U);
        gpio_write(LED_PORT, LED_PIN, false);
        delay_ms(180U);
    }
}

static void notice_pass_point(void)
{
    for (uint32_t i = 0; i < 60U; i++) {
        gpio_write(BEEP_PORT, BEEP_PIN, true);
        delay_cycles(BEEP_HALF_PERIOD_CYCLES);
        gpio_write(BEEP_PORT, BEEP_PIN, false);
        delay_cycles(BEEP_HALF_PERIOD_CYCLES);
    }

    gpio_write(LED_PORT, LED_PIN, true);
    delay_ms(80U);
    gpio_write(LED_PORT, LED_PIN, false);
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

static void motors_brake(void)
{
    gpio_write(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, true);
    gpio_write(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, true);
    gpio_write(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, true);
    gpio_write(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, true);
    gpio_write(MOTOR_CIN1_PORT, MOTOR_CIN1_PIN, true);
    gpio_write(MOTOR_CIN2_PORT, MOTOR_CIN2_PIN, true);
    gpio_write(MOTOR_DIN1_PORT, MOTOR_DIN1_PIN, true);
    gpio_write(MOTOR_DIN2_PORT, MOTOR_DIN2_PIN, true);
    gpio_write(MOTOR_PWMA_PORT, MOTOR_PWMA_PIN, true);
    gpio_write(MOTOR_PWMB_PORT, MOTOR_PWMB_PIN, true);
    gpio_write(MOTOR_PWMC_PORT, MOTOR_PWMC_PIN, true);
    gpio_write(MOTOR_PWMD_PORT, MOTOR_PWMD_PIN, true);
}

static uint8_t clamp_duty(int32_t duty)
{
    if (duty < 0) {
        return 0U;
    }
    if (duty > (int32_t)PWM_PERIOD_TICKS) {
        return PWM_PERIOD_TICKS;
    }
    return (uint8_t)duty;
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

static int16_t track_error(uint8_t mask)
{
    static const int8_t weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
    int16_t sum = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < 8U; i++) {
        if ((mask & (uint8_t)(1U << i)) != 0U) {
            sum += weights[i];
            count++;
        }
    }

    if (count == 0U) {
        return 0;
    }

    return (int16_t)(sum / count);
}

static bool line_detected_with_mask(uint8_t mask, uint8_t detectMask, uint8_t minCount)
{
    return bit_count(mask & detectMask) >= minCount;
}

static void active_brake_then_stop(void)
{
    motors_brake();
    delay_ms(BRAKE_MS);
    motors_safe_stop();
}

static void run_straight_to_line_with_ignore(uint8_t leftDuty, uint8_t rightDuty,
    uint8_t detectMask, uint8_t minCount, uint32_t ignoreMs, bool brakeAtEnd)
{
    uint32_t confirmMs = 0;

    motors_forward_dir();

    for (uint32_t t = 0; ; t++) {
        uint8_t mask = track_read_mask();

        if ((t > ignoreMs) && line_detected_with_mask(mask, detectMask, minCount)) {
            confirmMs++;
        } else {
            confirmMs = 0U;
        }

        if (confirmMs >= LINE_DETECT_CONFIRM_MS) {
            break;
        }

        pwm_run_1ms(leftDuty, rightDuty);
    }

    if (brakeAtEnd) {
        active_brake_then_stop();
    }
}

static void run_straight_to_line(void)
{
    run_straight_to_line_with_ignore(STRAIGHT_LEFT_DUTY, STRAIGHT_RIGHT_DUTY,
        LINE_DETECT_MASK, LINE_DETECT_MIN_COUNT, START_IGNORE_MS, true);
}

static void run_cd_straight_to_line(void)
{
    run_straight_to_line_with_ignore(CD_STRAIGHT_LEFT_DUTY, CD_STRAIGHT_RIGHT_DUTY,
        CD_LINE_DETECT_MASK, CD_LINE_DETECT_MIN_COUNT, CD_LINE_IGNORE_MS, true);
}

static void cd_exit_align_right(void)
{
    motors_forward_dir();

    for (uint32_t t = 0; t < CD_ALIGN_MS; t++) {
        pwm_run_1ms(CD_ALIGN_LEFT_DUTY, CD_ALIGN_RIGHT_DUTY);
    }
}

static void line_follow_step(uint8_t mask, int16_t *lastError)
{
    uint8_t leftDuty;
    uint8_t rightDuty;

    if (mask != 0U) {
        int16_t error = track_error(mask);
        int32_t correction = (int32_t)error * LINE_KP;

        *lastError = error;
        leftDuty = clamp_duty((int32_t)LINE_BASE_LEFT_DUTY - correction);
        rightDuty = clamp_duty((int32_t)LINE_BASE_RIGHT_DUTY + correction);
    } else if (*lastError < 0) {
        leftDuty = 62U;
        rightDuty = 32U;
    } else if (*lastError > 0) {
        leftDuty = 28U;
        rightDuty = 68U;
    } else {
        leftDuty = LINE_BASE_LEFT_DUTY;
        rightDuty = LINE_BASE_RIGHT_DUTY;
    }

    pwm_run_1ms(leftDuty, rightDuty);
}

static void settle_on_arc(void)
{
    int16_t lastError = 0;

    motors_forward_dir();

    for (uint32_t t = 0; t < ARC_SETTLE_MS; t++) {
        uint8_t mask = track_read_mask();
        line_follow_step(mask, &lastError);
    }
}

static void run_arc_until_lost(void)
{
    uint32_t lostConfirmMs = 0;
    int16_t lastError = 0;

    motors_forward_dir();
    settle_on_arc();

    for (uint32_t t = 0; ; t++) {
        uint8_t mask = track_read_mask();

        if ((t > ARC_MIN_RUN_MS) && ((mask & ARC_LOST_MASK) == 0U)) {
            lostConfirmMs++;
        } else {
            lostConfirmMs = 0U;
        }

        if (lostConfirmMs >= ARC_LOST_CONFIRM_MS) {
            break;
        }

        line_follow_step(mask, &lastError);
    }

    active_brake_then_stop();
}

static void capture_d_arc_right(void)
{
    motors_forward_dir();

    for (uint32_t t = 0; t < D_ARC_CAPTURE_MS; t++) {
        pwm_run_1ms(D_ARC_CAPTURE_LEFT_DUTY, D_ARC_CAPTURE_RIGHT_DUTY);
    }
}

static void run_task_1(void)
{
    run_straight_to_line();
    notice_arrived();
}

static void run_task_2(void)
{
    run_straight_to_line();
    notice_pass_point();
    delay_ms(80U);
    run_arc_until_lost();
    notice_pass_point();
    delay_ms(80U);
    cd_exit_align_right();
    run_cd_straight_to_line();
    notice_pass_point();
    delay_ms(30U);
    capture_d_arc_right();
    run_arc_until_lost();
    notice_arrived();
}

int main(void)
{
    SYSCFG_DL_init();
    gpio_write(MOTOR_STBY_PORT, MOTOR_STBY_PIN, true);
    gpio_write(LED_PORT, LED_PIN, false);
    gpio_write(BEEP_PORT, BEEP_PIN, false);
    motors_safe_stop();

    wait_start_key();
#if TASK_MODE == 2U
    run_task_2();
#else
    run_task_1();
#endif

    while (1) {
        motors_safe_stop();
        delay_ms(20U);
    }
}
