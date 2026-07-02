#include "ti_msp_dl_config.h"

#define TRACK_BLACK_IS_HIGH 0U

#define TASK_MODE 99U

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
#define BEEP_ENABLE 1U
#define BEEP_OFF_LEVEL true
#define BEEP_HALF_PERIOD_CYCLES (CPUCLK_FREQ / 4000U)

#define PWM_PERIOD_TICKS 200U
#define I2C_TIMEOUT_CYCLES 40000U
#define IMU_WHO_AM_I_REG 0x75U
#define IMU_WHO_AM_I_EXPECTED 0x47U
#define IMU_DEVICE_CONFIG 0x11U
#define IMU_REG_BANK_SEL 0x76U
#define IMU_PWR_MGMT0 0x4EU
#define IMU_GYRO_CONFIG0 0x4FU
#define IMU_GYRO_DATA_Z1 0x29U
#define IMU_GYRO_SAMPLE_MS 8U
#define IMU_GYRO_SAMPLE_MAX_RAW 12000
#define IMU_TURN_DRIFT_DEAD_RAW 8
#define IMU_FAST_REBIAS_SETTLE_MS 220U
#define IMU_FAST_REBIAS_SAMPLES 64U
#define IMU_FAST_REBIAS_SAMPLE_MS 4U

#define TASK3_TURN_LEFT 0U
#define TASK3_TURN_RIGHT 1U
#define TASK3_TURN_DUTY 38U
#define TASK3_TURN_KICK_DUTY 50U
#define TASK3_TURN_KICK_MS 30U
#define TASK3_TURN_IGNORE_MS 40U
#define TASK3_TURN_MIN_MS 60U
#define TASK3_TURN_NO_GYRO_STOP_MS 500U
#define TASK3_TURN_MAX_MS 3000U
#define TASK3_SETTLE_MS 120U
#define TASK3_POINT_DELAY_MS 80U
#define TASK3_A_TO_AC_TURN_RAW 510000
#define TASK3_C_TO_CB_TURN_RAW 100000
#define TASK3_B_TO_BD_TURN_RAW 400000
#define TASK3_D_TO_DA_TURN_RAW 70000
#define TASK3_A_TO_AC_OPEN_TURN_MS 480U
#define TASK3_C_TO_CB_OPEN_TURN_MS 360U
#define TASK3_B_TO_BD_OPEN_TURN_MS 360U
#define TASK3_D_TO_DA_OPEN_TURN_MS 360U
#define TASK3_FIND_LINE_MAX_MS 4300U
#define TASK3_FIND_LINE_IGNORE_MS 1400U
#define TASK3_BD_FIND_LINE_IGNORE_MS 650U
#define TASK3_FIND_LINE_CONFIRM_MS 20U
#define TASK3_FIND_LINE_MASK 0x3CU
#define TASK3_STRAIGHT_LEFT_DUTY 45U
#define TASK3_STRAIGHT_RIGHT_DUTY 51U
#define TASK3_HEADING_KP 3
#define TASK3_HEADING_CORR_DIV 900
#define TASK3_CAPTURE_MASK 0x3CU
#define TASK3_CAPTURE_MAX_MS 900U
#define TASK3_CAPTURE_LEFT_DUTY 24U
#define TASK3_CAPTURE_RIGHT_DUTY 28U
#define TASK3_CB_BASE_LEFT_DUTY 44U
#define TASK3_CB_BASE_RIGHT_DUTY 20U
#define TASK3_CB_KP 7
#define TASK3_DA_BASE_LEFT_DUTY 34U
#define TASK3_DA_BASE_RIGHT_DUTY 16U
#define TASK3_DA_KP 9
#define TASK3_CB_ARC_MIN_MS 800U
#define TASK3_DA_ARC_MIN_MS 1200U
#define TASK3_CB_ARC_LOST_CONFIRM_MS 140U
#define TASK3_DA_ARC_LOST_CONFIRM_MS 360U
#define TASK3_DA_CAPTURE_MS 820U


#define PWM_STEP_DELAY_CYCLES (CPUCLK_FREQ / 200000U)

static uint8_t gImuAddr = IMU_I2C_ADDR;
static int32_t gGyroZBias = 0;
static bool gImuReady = false;

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

static void beep_tone_cycles(uint32_t cycles)
{
    for (uint32_t i = 0; i < cycles; i++) {
#if BEEP_ENABLE
        gpio_write(BEEP_PORT, BEEP_PIN, true);
        delay_cycles(BEEP_HALF_PERIOD_CYCLES);
        gpio_write(BEEP_PORT, BEEP_PIN, false);
        delay_cycles(BEEP_HALF_PERIOD_CYCLES);
#else
        delay_cycles(BEEP_HALF_PERIOD_CYCLES * 2U);
#endif
    }
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
    beep_tone_cycles(1000U);

    for (uint8_t i = 0; i < 3U; i++) {
        gpio_write(LED_PORT, LED_PIN, true);
        delay_ms(180U);
        gpio_write(LED_PORT, LED_PIN, false);
        delay_ms(180U);
    }
}

static void notice_pass_point(void)
{
    beep_tone_cycles(60U);

    gpio_write(LED_PORT, LED_PIN, true);
    delay_ms(80U);
    gpio_write(LED_PORT, LED_PIN, false);
}

static void notice_fail_code(uint8_t code)
{
    motors_safe_stop();
    for (uint8_t i = 0; i < code; i++) {
        gpio_write(LED_PORT, LED_PIN, true);
        delay_ms(350U);
        gpio_write(LED_PORT, LED_PIN, false);
        delay_ms(350U);
    }
}

static void notice_task_selected(uint8_t task)
{
    for (uint8_t i = 0; i < task; i++) {
        gpio_write(LED_PORT, LED_PIN, true);
        delay_ms(120U);
        gpio_write(LED_PORT, LED_PIN, false);
        delay_ms(160U);
    }
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

static void motors_spin_left_dir(void)
{
    gpio_write(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, false);
    gpio_write(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, true);
    gpio_write(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, true);
    gpio_write(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, false);
    gpio_write(MOTOR_CIN1_PORT, MOTOR_CIN1_PIN, false);
    gpio_write(MOTOR_CIN2_PORT, MOTOR_CIN2_PIN, true);
    gpio_write(MOTOR_DIN1_PORT, MOTOR_DIN1_PIN, true);
    gpio_write(MOTOR_DIN2_PORT, MOTOR_DIN2_PIN, false);
}

static void motors_spin_right_dir(void)
{
    gpio_write(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, true);
    gpio_write(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, false);
    gpio_write(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, false);
    gpio_write(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, true);
    gpio_write(MOTOR_CIN1_PORT, MOTOR_CIN1_PIN, true);
    gpio_write(MOTOR_CIN2_PORT, MOTOR_CIN2_PIN, false);
    gpio_write(MOTOR_DIN1_PORT, MOTOR_DIN1_PIN, false);
    gpio_write(MOTOR_DIN2_PORT, MOTOR_DIN2_PIN, true);
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

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static void imu_i2c_recover(void)
{
    DL_I2C_disableController(IMU_I2C);
    DL_I2C_resetControllerTransfer(IMU_I2C);
    DL_I2C_flushControllerTXFIFO(IMU_I2C);
    DL_I2C_flushControllerRXFIFO(IMU_I2C);
    DL_I2C_clearInterruptStatus(IMU_I2C, 0xFFFFFFFFU);
    DL_I2C_enableController(IMU_I2C);
}

static bool i2c_wait_idle_status(void)
{
    for (uint32_t t = 0; t < I2C_TIMEOUT_CYCLES; t++) {
        if ((DL_I2C_getControllerStatus(IMU_I2C) &
             DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) {
            return true;
        }
    }

    return false;
}

static bool i2c_wait_bus_done(void)
{
    for (uint32_t t = 0; t < I2C_TIMEOUT_CYCLES; t++) {
        uint32_t status = DL_I2C_getControllerStatus(IMU_I2C);

        if ((status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) == 0U) {
            return ((status & DL_I2C_CONTROLLER_STATUS_ERROR) == 0U);
        }
    }

    return false;
}

static bool imu_read_reg_addr(uint8_t addr, uint8_t reg, uint8_t *value)
{
    imu_i2c_recover();
    if (!i2c_wait_idle_status()) {
        return false;
    }

    DL_I2C_flushControllerTXFIFO(IMU_I2C);
    DL_I2C_flushControllerRXFIFO(IMU_I2C);
    DL_I2C_resetControllerTransfer(IMU_I2C);
    DL_I2C_fillControllerTXFIFO(IMU_I2C, &reg, 1U);
    DL_I2C_startControllerTransfer(IMU_I2C, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U);
    delay_cycles(100U);

    if (!i2c_wait_bus_done() || !i2c_wait_idle_status()) {
        return false;
    }

    DL_I2C_startControllerTransfer(IMU_I2C, addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, 1U);
    delay_cycles(100U);

    for (uint32_t t = 0; t < I2C_TIMEOUT_CYCLES; t++) {
        uint32_t status = DL_I2C_getControllerStatus(IMU_I2C);

        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            return false;
        }
        if (!DL_I2C_isControllerRXFIFOEmpty(IMU_I2C)) {
            *value = DL_I2C_receiveControllerData(IMU_I2C);
            return i2c_wait_bus_done();
        }
    }

    return false;
}

static bool imu_write_reg_addr(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};

    imu_i2c_recover();
    if (!i2c_wait_idle_status()) {
        return false;
    }

    DL_I2C_flushControllerTXFIFO(IMU_I2C);
    DL_I2C_flushControllerRXFIFO(IMU_I2C);
    DL_I2C_resetControllerTransfer(IMU_I2C);
    DL_I2C_fillControllerTXFIFO(IMU_I2C, tx, 2U);
    DL_I2C_startControllerTransfer(IMU_I2C, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2U);
    delay_cycles(100U);

    return i2c_wait_bus_done();
}

static bool imu_read_i16_addr(uint8_t addr, uint8_t regHigh, int16_t *value)
{
    uint8_t high = 0U;
    uint8_t low = 0U;

    if (!imu_read_reg_addr(addr, regHigh, &high)) {
        return false;
    }
    if (!imu_read_reg_addr(addr, (uint8_t)(regHigh + 1U), &low)) {
        return false;
    }

    *value = (int16_t)(((uint16_t)high << 8) | low);
    return true;
}

static bool imu_detect_addr(uint8_t *addr)
{
    uint8_t who = 0U;

    if (imu_read_reg_addr(IMU_I2C_ADDR, IMU_WHO_AM_I_REG, &who) &&
        (who == IMU_WHO_AM_I_EXPECTED)) {
        *addr = IMU_I2C_ADDR;
        return true;
    }

    if (imu_read_reg_addr(IMU_I2C_ADDR_ALT, IMU_WHO_AM_I_REG, &who) &&
        (who == IMU_WHO_AM_I_EXPECTED)) {
        *addr = IMU_I2C_ADDR_ALT;
        return true;
    }

    return false;
}

static bool imu_start_motion_sensors(uint8_t addr)
{
    if (!imu_write_reg_addr(addr, IMU_REG_BANK_SEL, 0x00U)) {
        return false;
    }
    if (!imu_write_reg_addr(addr, IMU_GYRO_CONFIG0, 0x06U)) {
        return false;
    }
    if (!imu_write_reg_addr(addr, IMU_PWR_MGMT0, 0x0FU)) {
        return false;
    }
    delay_ms(200U);
    return true;
}

static bool imu_read_gyro_z(int16_t *gz)
{
    if (!gImuReady) {
        return false;
    }
    return imu_read_i16_addr(gImuAddr, IMU_GYRO_DATA_Z1, gz);
}

static bool imu_rebias_gyro_z_fast(void)
{
    int32_t sum = 0;
    uint16_t count = 0;

    if (!gImuReady) {
        return false;
    }

    motors_safe_stop();
    delay_ms(IMU_FAST_REBIAS_SETTLE_MS);

    for (uint8_t i = 0; i < IMU_FAST_REBIAS_SAMPLES; i++) {
        int16_t gz = 0;

        if (imu_read_gyro_z(&gz)) {
            sum += gz;
            count++;
        }
        delay_ms(IMU_FAST_REBIAS_SAMPLE_MS);
    }

    if (count == 0U) {
        return false;
    }

    gGyroZBias = sum / (int32_t)count;
    return true;
}

static bool imu_init_for_route(void)
{
    uint8_t addr = IMU_I2C_ADDR;
    bool detected = false;

    delay_ms(300U);
    for (uint8_t retry = 0; retry < 5U; retry++) {
        if (imu_detect_addr(&addr)) {
            detected = true;
            break;
        }
        delay_ms(120U);
    }

    if (!detected) {
        gImuReady = false;
        return false;
    }

    gImuAddr = addr;
    imu_write_reg_addr(gImuAddr, IMU_DEVICE_CONFIG, 0x01U);
    delay_ms(120U);

    if (!imu_start_motion_sensors(gImuAddr)) {
        gImuReady = false;
        return false;
    }

    gImuReady = true;
    return imu_rebias_gyro_z_fast();
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

static void task3_open_turn(uint8_t direction, uint32_t turnMs)
{
    if (direction == TASK3_TURN_LEFT) {
        motors_spin_left_dir();
    } else {
        motors_spin_right_dir();
    }

    for (uint32_t t = 0; t < turnMs; t++) {
        uint8_t duty = (t < TASK3_TURN_KICK_MS) ? TASK3_TURN_KICK_DUTY :
            TASK3_TURN_DUTY;

        pwm_run_1ms(duty, duty);
    }

    active_brake_then_stop();
    delay_ms(TASK3_SETTLE_MS);
}

static uint8_t task3_turn_by_gyro(uint8_t direction, int32_t targetRaw,
    uint32_t fallbackMs)
{
    int32_t turn = 0;
    uint32_t gyroReadMs = 0;
    uint32_t validGyroMs = 0;
    uint32_t elapsedMs = 0;
    uint32_t noGyroStopMs = (fallbackMs < TASK3_TURN_NO_GYRO_STOP_MS) ?
        fallbackMs : TASK3_TURN_NO_GYRO_STOP_MS;

    if (!gImuReady) {
        task3_open_turn(direction, fallbackMs);
        return 0U;
    }

    if (direction == TASK3_TURN_LEFT) {
        motors_spin_left_dir();
    } else {
        motors_spin_right_dir();
    }

    for (uint32_t t = 0; t < TASK3_TURN_MAX_MS; t++) {
        int16_t gzRaw = 0;
        uint8_t duty = (t < TASK3_TURN_KICK_MS) ? TASK3_TURN_KICK_DUTY :
            TASK3_TURN_DUTY;

        if (((t % IMU_GYRO_SAMPLE_MS) == 0U) && imu_read_gyro_z(&gzRaw)) {
            int32_t sample = abs_i32((int32_t)gzRaw - gGyroZBias);

            if (t >= TASK3_TURN_IGNORE_MS) {
                gyroReadMs += IMU_GYRO_SAMPLE_MS;
            }
            if (sample > IMU_GYRO_SAMPLE_MAX_RAW) {
                sample = IMU_GYRO_SAMPLE_MAX_RAW;
            }
            if ((t >= TASK3_TURN_IGNORE_MS) &&
                (sample > IMU_TURN_DRIFT_DEAD_RAW)) {
                turn += sample * (int32_t)IMU_GYRO_SAMPLE_MS;
                validGyroMs += IMU_GYRO_SAMPLE_MS;
            }
        }

        if ((t > noGyroStopMs) && (validGyroMs == 0U)) {
            elapsedMs = t;
            active_brake_then_stop();
            if (elapsedMs < fallbackMs) {
                delay_ms(30U);
                task3_open_turn(direction, fallbackMs - elapsedMs);
            }
            return 0U;
        }
        if ((t >= TASK3_TURN_MIN_MS) && (turn >= targetRaw)) {
            active_brake_then_stop();
            delay_ms(TASK3_SETTLE_MS);
            return 0U;
        }

        pwm_run_1ms(duty, duty);
    }

    active_brake_then_stop();
    delay_ms(TASK3_SETTLE_MS);
    if (gyroReadMs == 0U) {
        if (TASK3_TURN_MAX_MS < fallbackMs) {
            task3_open_turn(direction, fallbackMs - TASK3_TURN_MAX_MS);
        }
        return 0U;
    }
    return 3U;
}

static bool task3_heading_to_line(uint32_t ignoreMs, uint32_t settleMs)
{
    uint32_t confirmMs = 0;
    int32_t heading = 0;
    int32_t lastCorrection = 0;

    motors_forward_dir();

    for (uint32_t t = 0; t < TASK3_FIND_LINE_MAX_MS; t++) {
        uint8_t mask = track_read_mask();
        int16_t gzRaw = 0;
        int32_t correction = 0;

        if ((t > ignoreMs) &&
            line_detected_with_mask(mask, TASK3_FIND_LINE_MASK, 1U)) {
            confirmMs++;
        } else {
            confirmMs = 0U;
        }

        if (imu_read_gyro_z(&gzRaw)) {
            int32_t gz = (int32_t)gzRaw - gGyroZBias;

            if ((gz > IMU_TURN_DRIFT_DEAD_RAW) ||
                (gz < -IMU_TURN_DRIFT_DEAD_RAW)) {
                heading += gz;
            }
            correction = (heading * TASK3_HEADING_KP) / TASK3_HEADING_CORR_DIV;
        }
        lastCorrection = correction;

        if (confirmMs >= TASK3_FIND_LINE_CONFIRM_MS) {
            for (uint32_t settle = 0; settle < settleMs; settle++) {
                pwm_run_1ms(clamp_duty((int32_t)TASK3_STRAIGHT_LEFT_DUTY +
                        lastCorrection),
                    clamp_duty((int32_t)TASK3_STRAIGHT_RIGHT_DUTY -
                        lastCorrection));
            }
            active_brake_then_stop();
            delay_ms(TASK3_SETTLE_MS);
            return true;
        }

        pwm_run_1ms(clamp_duty((int32_t)TASK3_STRAIGHT_LEFT_DUTY + correction),
            clamp_duty((int32_t)TASK3_STRAIGHT_RIGHT_DUTY - correction));
    }

    active_brake_then_stop();
    delay_ms(TASK3_SETTLE_MS);
    return false;
}

static void task3_capture_line(void)
{
    uint32_t confirmMs = 0;

    motors_forward_dir();

    for (uint32_t t = 0; t < TASK3_CAPTURE_MAX_MS; t++) {
        uint8_t mask = track_read_mask();

        if (line_detected_with_mask(mask, TASK3_CAPTURE_MASK, 1U)) {
            confirmMs++;
        } else {
            confirmMs = 0U;
        }

        if (confirmMs >= TASK3_FIND_LINE_CONFIRM_MS) {
            break;
        }

        pwm_run_1ms(TASK3_CAPTURE_LEFT_DUTY, TASK3_CAPTURE_RIGHT_DUTY);
    }
}

static void task3_cb_follow_step(uint8_t mask, int16_t *lastError)
{
    uint8_t leftDuty;
    uint8_t rightDuty;

    if ((mask & 0x03U) != 0U) {
        *lastError = -7;
        leftDuty = 78U;
        rightDuty = 8U;
    } else if ((mask & 0xC0U) != 0U) {
        *lastError = 7;
        leftDuty = 28U;
        rightDuty = 46U;
    } else if (mask != 0U) {
        int16_t error = track_error(mask);
        int32_t correction = (int32_t)error * TASK3_CB_KP;

        *lastError = error;
        leftDuty = clamp_duty((int32_t)TASK3_CB_BASE_LEFT_DUTY - correction);
        rightDuty = clamp_duty((int32_t)TASK3_CB_BASE_RIGHT_DUTY + correction);
    } else if (*lastError < 0) {
        leftDuty = 76U;
        rightDuty = 8U;
    } else if (*lastError > 0) {
        leftDuty = 24U;
        rightDuty = 48U;
    } else {
        leftDuty = TASK3_CB_BASE_LEFT_DUTY;
        rightDuty = TASK3_CB_BASE_RIGHT_DUTY;
    }

    pwm_run_1ms(leftDuty, rightDuty);
}

static void task3_da_follow_step(uint8_t mask, int16_t *lastError)
{
    uint8_t leftDuty;
    uint8_t rightDuty;

    if ((mask & 0x03U) != 0U) {
        *lastError = -7;
        leftDuty = 44U;
        rightDuty = 18U;
    } else if ((mask & 0xC0U) != 0U) {
        *lastError = 7;
        leftDuty = 10U;
        rightDuty = 68U;
    } else if (mask != 0U) {
        int16_t error = track_error(mask);
        int32_t correction = (int32_t)error * TASK3_DA_KP;

        *lastError = error;
        leftDuty = clamp_duty((int32_t)TASK3_DA_BASE_LEFT_DUTY - correction);
        rightDuty = clamp_duty((int32_t)TASK3_DA_BASE_RIGHT_DUTY + correction);
    } else if (*lastError < 0) {
        leftDuty = 42U;
        rightDuty = 18U;
    } else if (*lastError > 0) {
        leftDuty = 10U;
        rightDuty = 68U;
    } else {
        leftDuty = TASK3_DA_BASE_LEFT_DUTY;
        rightDuty = TASK3_DA_BASE_RIGHT_DUTY;
    }

    pwm_run_1ms(leftDuty, rightDuty);
}

static void task3_follow_arc_until_lost(bool rightArc)
{
    uint32_t lostMs = 0;
    uint32_t arcMinMs = rightArc ? TASK3_DA_ARC_MIN_MS : TASK3_CB_ARC_MIN_MS;
    uint32_t lostConfirmMs = rightArc ? TASK3_DA_ARC_LOST_CONFIRM_MS :
        TASK3_CB_ARC_LOST_CONFIRM_MS;
    int16_t lastError = rightArc ? 7 : -7;

    motors_forward_dir();

    for (uint32_t t = 0; ; t++) {
        uint8_t mask = track_read_mask();

        if ((t > arcMinMs) && ((mask & ARC_LOST_MASK) == 0U)) {
            lostMs++;
        } else {
            lostMs = 0U;
        }

        if (lostMs >= lostConfirmMs) {
            break;
        }

        if (rightArc) {
            task3_da_follow_step(mask, &lastError);
        } else {
            task3_cb_follow_step(mask, &lastError);
        }
    }

    active_brake_then_stop();
}

static void task3_capture_da_arc(void)
{
    int16_t lastError = 7;
    uint32_t capturedMs = 0;

    motors_forward_dir();

    for (uint32_t t = 0;
         (t < TASK3_CAPTURE_MAX_MS) && (capturedMs < TASK3_DA_CAPTURE_MS);
         t++) {
        uint8_t mask = track_read_mask();

        if (mask != 0U) {
            capturedMs++;
        }
        task3_da_follow_step(mask, &lastError);
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

static bool prepare_imu_or_fail(void)
{
    motors_safe_stop();
    if (!gImuReady && !imu_init_for_route()) {
        notice_fail_code(2U);
        return false;
    }

    return true;
}

static bool task3_rebias_or_fail(void)
{
    active_brake_then_stop();
    if (!imu_rebias_gyro_z_fast()) {
        gImuReady = false;
    }
    delay_ms(TASK3_SETTLE_MS);
    return true;
}

static bool run_task_3_core(bool noticeDone)
{
    uint8_t turnStatus;

    if (!prepare_imu_or_fail()) {
        return false;
    }

    turnStatus = task3_turn_by_gyro(TASK3_TURN_RIGHT, TASK3_A_TO_AC_TURN_RAW,
        TASK3_A_TO_AC_OPEN_TURN_MS);
    if (turnStatus != 0U) {
        notice_fail_code(turnStatus);
        return false;
    }
    if (!task3_heading_to_line(TASK3_FIND_LINE_IGNORE_MS, 60U)) {
        notice_fail_code(4U);
        return false;
    }
    notice_pass_point();
    delay_ms(TASK3_POINT_DELAY_MS);

    if (!task3_rebias_or_fail()) {
        return false;
    }
    turnStatus = task3_turn_by_gyro(TASK3_TURN_LEFT, TASK3_C_TO_CB_TURN_RAW,
        TASK3_C_TO_CB_OPEN_TURN_MS);
    if (turnStatus != 0U) {
        notice_fail_code(turnStatus);
        return false;
    }
    task3_capture_line();
    task3_follow_arc_until_lost(false);
    notice_pass_point();
    delay_ms(TASK3_POINT_DELAY_MS);

    if (!task3_rebias_or_fail()) {
        return false;
    }
    turnStatus = task3_turn_by_gyro(TASK3_TURN_LEFT, TASK3_B_TO_BD_TURN_RAW,
        TASK3_B_TO_BD_OPEN_TURN_MS);
    if (turnStatus != 0U) {
        notice_fail_code(turnStatus);
        return false;
    }
    if (!task3_heading_to_line(TASK3_BD_FIND_LINE_IGNORE_MS, 60U)) {
        notice_fail_code(4U);
        return false;
    }
    notice_pass_point();
    delay_ms(TASK3_POINT_DELAY_MS);

    turnStatus = task3_turn_by_gyro(TASK3_TURN_RIGHT, TASK3_D_TO_DA_TURN_RAW,
        TASK3_D_TO_DA_OPEN_TURN_MS);
    if (turnStatus != 0U) {
        notice_fail_code(turnStatus);
        return false;
    }
    task3_capture_da_arc();
    task3_follow_arc_until_lost(true);

    if (noticeDone) {
        notice_arrived();
    }
    return true;
}

static void run_task_3(void)
{
    (void)run_task_3_core(true);
}

static void run_task_4(void)
{
    if (!prepare_imu_or_fail()) {
        return;
    }

    for (uint8_t lap = 0; lap < 4U; lap++) {
        if (!run_task_3_core(false)) {
            return;
        }
        delay_ms(120U);
    }

    notice_arrived();
}

static void run_task3_debug(uint8_t mode)
{
    uint8_t status = 0U;

    if (!prepare_imu_or_fail()) {
        return;
    }

    if (mode == 31U) {
        status = task3_turn_by_gyro(TASK3_TURN_RIGHT, TASK3_A_TO_AC_TURN_RAW,
            TASK3_A_TO_AC_OPEN_TURN_MS);
        if ((status == 0U) && !task3_heading_to_line(TASK3_FIND_LINE_IGNORE_MS, 60U)) {
            status = 4U;
        }
    } else if (mode == 32U) {
        status = task3_turn_by_gyro(TASK3_TURN_LEFT, TASK3_C_TO_CB_TURN_RAW,
            TASK3_C_TO_CB_OPEN_TURN_MS);
        task3_capture_line();
    } else if (mode == 33U) {
        task3_follow_arc_until_lost(false);
    } else if (mode == 34U) {
        status = task3_turn_by_gyro(TASK3_TURN_LEFT, TASK3_B_TO_BD_TURN_RAW,
            TASK3_B_TO_BD_OPEN_TURN_MS);
        if ((status == 0U) && !task3_heading_to_line(TASK3_FIND_LINE_IGNORE_MS, 60U)) {
            status = 4U;
        }
    } else if (mode == 35U) {
        status = task3_turn_by_gyro(TASK3_TURN_RIGHT, TASK3_D_TO_DA_TURN_RAW,
            TASK3_D_TO_DA_OPEN_TURN_MS);
        task3_capture_da_arc();
    } else if (mode == 36U) {
        task3_follow_arc_until_lost(true);
    }

    if (status == 0U) {
        notice_arrived();
    } else {
        notice_fail_code(status);
    }
}

static void run_imu_static_test(void)
{
    uint16_t goodReads = 0;

    motors_safe_stop();
    if (!prepare_imu_or_fail()) {
        return;
    }

    for (uint16_t i = 0; i < 200U; i++) {
        int16_t gz = 0;

        if (imu_read_gyro_z(&gz)) {
            goodReads++;
            gpio_write(LED_PORT, LED_PIN, true);
        } else {
            gpio_write(LED_PORT, LED_PIN, false);
        }
        delay_ms(10U);
    }

    motors_safe_stop();
    gpio_write(LED_PORT, LED_PIN, false);

    if (goodReads < 150U) {
        notice_fail_code(2U);
        return;
    }

    notice_arrived();
}

static void run_tasks_1_to_2_by_key(void)
{
    wait_start_key();
    run_task_1();
    motors_safe_stop();

    wait_start_key();
    run_task_2();
    motors_safe_stop();
}

static uint8_t select_task_by_key(void)
{
    uint8_t task = 1U;

    notice_task_selected(task);

    while (1) {
        uint32_t pressedMs = 0;

        while (!key_pressed()) {
            delay_ms(10U);
        }

        while (key_pressed()) {
            if (pressedMs < 3000U) {
                pressedMs += 10U;
            }
            delay_ms(10U);
        }
        delay_ms(120U);

        if (pressedMs >= 1200U) {
            notice_task_selected(task);
            delay_ms(300U);
            return task;
        }

        task++;
        if (task > 5U) {
            task = 1U;
        }
        notice_task_selected(task);
    }
}

static void run_selected_task_once(void)
{
    uint8_t task = select_task_by_key();

    if (task == 1U) {
        run_task_1();
    } else if (task == 2U) {
        run_task_2();
    } else if (task == 3U) {
        run_task_3();
    } else if (task == 4U) {
        run_task_4();
    } else {
        run_imu_static_test();
    }

    motors_safe_stop();
}

int main(void)
{
    SYSCFG_DL_init();
    gpio_write(MOTOR_STBY_PORT, MOTOR_STBY_PIN, true);
    gpio_write(LED_PORT, LED_PIN, false);
    gpio_write(BEEP_PORT, BEEP_PIN, BEEP_OFF_LEVEL);
    motors_safe_stop();

#if TASK_MODE == 99U
    run_selected_task_once();
#elif TASK_MODE == 12U
    run_tasks_1_to_2_by_key();
#elif TASK_MODE == 34U
    wait_start_key();
    run_task_3();
    motors_safe_stop();

    wait_start_key();
    run_task_4();
    motors_safe_stop();
#elif (TASK_MODE >= 31U) && (TASK_MODE <= 36U)
    wait_start_key();
    run_task3_debug(TASK_MODE);
#else
    wait_start_key();
#if TASK_MODE == 4U
    run_task_4();
#elif TASK_MODE == 3U
    run_task_3();
#elif TASK_MODE == 2U
    run_task_2();
#else
    run_task_1();
#endif
#endif

    while (1) {
        motors_safe_stop();
        delay_ms(20U);
    }
}
