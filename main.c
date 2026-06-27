#include "ti_msp_dl_config.h"

#define TRACK_BLACK_IS_HIGH 0U

#define IMU_TEST_MODE 0U

#define TASK_MODE 2U
#define TASK2_STOP_AT_C_DEBUG 0U

#define STRAIGHT_LEFT_DUTY 53U
#define STRAIGHT_RIGHT_DUTY 60U
#define AC_STRAIGHT_LEFT_DUTY 53U
#define AC_STRAIGHT_RIGHT_DUTY 60U
#define BD_STRAIGHT_LEFT_DUTY 48U
#define BD_STRAIGHT_RIGHT_DUTY 54U
#define BD_ALIGN_LEFT_DUTY 80U
#define BD_ALIGN_RIGHT_DUTY 80U
#define BD_ALIGN_MS 700U
#define BD_ALIGN_TARGET_RAW 6500
#define BD_ALIGN_MAX_MS 170U
#define CB_ENTRY_LEFT_DUTY 78U
#define CB_ENTRY_RIGHT_DUTY 18U
#define CB_ENTRY_MAX_MS 500U
#define CB_ENTRY_MIN_MS 220U
#define CB_ENTRY_MASK 0x3CU
#define CB_ENTRY_CONFIRM_MS 40U
#define CB_ARC_BASE_LEFT_DUTY 58U
#define CB_ARC_BASE_RIGHT_DUTY 24U
#define CB_ARC_KP 8
#define CB_ARC_POINT_MIN_MS 1400U
#define CB_ARC_POINT_CONFIRM_MS 25U
#define CB_ARC_POINT_MIN_COUNT 5U
#define CB_ARC_MAX_MS 6200U
#define TASK3_LINE_BASE_LEFT_DUTY 24U
#define TASK3_LINE_BASE_RIGHT_DUTY 30U
#define TASK3_LINE_KP 10
#define DA_ARC_BASE_LEFT_DUTY 22U
#define DA_ARC_BASE_RIGHT_DUTY 43U
#define DA_ARC_KP 4
#define DA_ACQUIRE_MS 480U
#define DA_ENTRY_ARC_LEFT_DUTY 8U
#define DA_ENTRY_ARC_RIGHT_DUTY 60U
#define DA_ENTRY_ARC_MS 285U
#define DA_ENTRY_LEFT_DUTY 70U
#define DA_ENTRY_RIGHT_DUTY 70U
#define DA_ENTRY_TARGET_RAW 6000
#define DA_ENTRY_MAX_MS 180U
#define A_RESTART_LEFT_DUTY 72U
#define A_RESTART_RIGHT_DUTY 72U
#define A_RESTART_TARGET_RAW BD_ALIGN_TARGET_RAW
#define A_RESTART_MAX_MS 260U
#define DA_CENTER_MASK 0x18U
#define DA_CENTER_MAX_MS 520U
#define CD_STRAIGHT_LEFT_DUTY 54U
#define CD_STRAIGHT_RIGHT_DUTY 60U
#define CD_ALIGN_LEFT_DUTY 35U
#define CD_ALIGN_RIGHT_DUTY 72U
#define CD_ALIGN_MS 350U
#define CD_CENTER_MASK 0x18U
#define CD_CENTER_MAX_MS 260U
#define LINE_BASE_LEFT_DUTY 32U
#define LINE_BASE_RIGHT_DUTY 38U
#define LINE_KP 7
#define TASK2_ARC_BASE_LEFT_DUTY 26U
#define TASK2_ARC_BASE_RIGHT_DUTY 44U
#define TASK2_ARC_KP 7
#define TASK2_ARC_LOST_CONFIRM_MS 160U
#define START_IGNORE_MS 1200U
#define LINE_DETECT_MASK 0xFFU
#define LINE_DETECT_MIN_COUNT 1U
#define AC_LINE_DETECT_MASK 0x3CU
#define AC_LINE_DETECT_MIN_COUNT 1U
#define BD_LINE_DETECT_MASK 0xFFU
#define BD_LINE_DETECT_MIN_COUNT 1U
#define CD_LINE_DETECT_MASK 0x3CU
#define CD_LINE_DETECT_MIN_COUNT 1U
#define LINE_DETECT_CONFIRM_MS 1U
#define STRAIGHT_LEAVE_LINE_IGNORE_MS 500U
#define ARC_MIN_RUN_MS 800U
#define ARC_LOST_CONFIRM_MS 120U
#define ARC_LOST_MASK 0xFFU
#define ARC_SETTLE_MS 350U
#define BRAKE_MS 80U
#define BEEP_HALF_PERIOD_CYCLES (CPUCLK_FREQ / 3000U)

#define PWM_PERIOD_TICKS 200U
#define IMU_WHO_AM_I_REG 0x75U
#define IMU_WHO_AM_I_EXPECTED 0x47U
#define I2C_TIMEOUT_CYCLES 40000U
#define IMU_I2C_ADDR_ALT 0x69U
#define IMU_DEVICE_CONFIG 0x11U
#define IMU_REG_BANK_SEL 0x76U
#define IMU_PWR_MGMT0 0x4EU
#define IMU_GYRO_CONFIG0 0x4FU
#define IMU_ACCEL_CONFIG0 0x50U
#define IMU_ACCEL_DATA_X1 0x1FU
#define IMU_ACCEL_DATA_Y1 0x21U
#define IMU_ACCEL_DATA_Z1 0x23U
#define IMU_GYRO_DATA_X1 0x25U
#define IMU_GYRO_DATA_Y1 0x27U
#define IMU_GYRO_DATA_Z1 0x29U
#define IMU_ACCEL_DEAD_RAW 2500
#define IMU_GYRO_DEAD_RAW 120
#define IMU_HEADING_KP 3
#define IMU_HEADING_CORR_DIV 900
#define IMU_GYRO_DRIFT_DEAD_RAW 20


#define PWM_STEP_DELAY_CYCLES (CPUCLK_FREQ / 200000U)

static uint8_t gImuAddr = IMU_I2C_ADDR;
static int32_t gGyroZBias = 0;
static bool gImuReady = false;
static uint8_t gLastLineHitMask = 0U;

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
    for (uint32_t i = 0; i < 180U; i++) {
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

static void notice_imu_success(void)
{
    for (uint32_t i = 0; i < 120U; i++) {
        gpio_write(BEEP_PORT, BEEP_PIN, true);
        delay_cycles(BEEP_HALF_PERIOD_CYCLES);
        gpio_write(BEEP_PORT, BEEP_PIN, false);
        delay_cycles(BEEP_HALF_PERIOD_CYCLES);
    }

    for (uint8_t i = 0; i < 5U; i++) {
        gpio_write(LED_PORT, LED_PIN, true);
        delay_ms(80U);
        gpio_write(LED_PORT, LED_PIN, false);
        delay_ms(80U);
    }
}

static void notice_imu_fail(void)
{
    for (uint8_t beep = 0; beep < 3U; beep++) {
        for (uint32_t i = 0; i < 80U; i++) {
            gpio_write(BEEP_PORT, BEEP_PIN, true);
            delay_cycles(BEEP_HALF_PERIOD_CYCLES);
            gpio_write(BEEP_PORT, BEEP_PIN, false);
            delay_cycles(BEEP_HALF_PERIOD_CYCLES);
        }
        delay_ms(140U);
    }

    for (uint8_t i = 0; i < 3U; i++) {
        gpio_write(LED_PORT, LED_PIN, true);
        delay_ms(450U);
        gpio_write(LED_PORT, LED_PIN, false);
        delay_ms(450U);
    }
}

static void notice_imu_fail_code(uint8_t code)
{
    for (uint8_t i = 0; i < code; i++) {
        for (uint32_t n = 0; n < 70U; n++) {
            gpio_write(BEEP_PORT, BEEP_PIN, true);
            delay_cycles(BEEP_HALF_PERIOD_CYCLES);
            gpio_write(BEEP_PORT, BEEP_PIN, false);
            delay_cycles(BEEP_HALF_PERIOD_CYCLES);
        }
        delay_ms(160U);
    }

    delay_ms(400U);

    for (uint8_t i = 0; i < code; i++) {
        gpio_write(LED_PORT, LED_PIN, true);
        delay_ms(380U);
        gpio_write(LED_PORT, LED_PIN, false);
        delay_ms(380U);
    }
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
        uint32_t status = DL_I2C_getControllerStatus(IMU_I2C);

        if ((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) {
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
    if (!i2c_wait_bus_done()) {
        return false;
    }

    if (!i2c_wait_idle_status()) {
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

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static bool imu_detect_addr(uint8_t *addr, bool reportError)
{
    uint8_t who68 = 0U;
    uint8_t who69 = 0U;
    bool ok68;
    bool ok69;

    ok68 = imu_read_reg_addr(IMU_I2C_ADDR, IMU_WHO_AM_I_REG, &who68);
    ok69 = imu_read_reg_addr(IMU_I2C_ADDR_ALT, IMU_WHO_AM_I_REG, &who69);

    if (ok68 && (who68 == IMU_WHO_AM_I_EXPECTED)) {
        *addr = IMU_I2C_ADDR;
        return true;
    }
    if (ok69 && (who69 == IMU_WHO_AM_I_EXPECTED)) {
        *addr = IMU_I2C_ADDR_ALT;
        return true;
    } else if (reportError && (ok68 || ok69)) {
        notice_imu_fail_code(3U);
    } else if (reportError) {
        notice_imu_fail_code(2U);
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
    if (!imu_write_reg_addr(addr, IMU_ACCEL_CONFIG0, 0x06U)) {
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

static bool imu_init_for_route(void)
{
    int32_t sum = 0;
    uint16_t count = 0;
    bool detected = false;

    delay_ms(300U);
    for (uint8_t retry = 0; retry < 5U; retry++) {
        if (imu_detect_addr(&gImuAddr, false)) {
            detected = true;
            break;
        }
        delay_ms(120U);
    }

    if (!detected) {
        gImuReady = false;
        for (uint8_t i = 0; i < 2U; i++) {
            gpio_write(LED_PORT, LED_PIN, true);
            delay_ms(120U);
            gpio_write(LED_PORT, LED_PIN, false);
            delay_ms(120U);
        }
        return false;
    }

    imu_write_reg_addr(gImuAddr, IMU_DEVICE_CONFIG, 0x01U);
    delay_ms(120U);
    if (!imu_detect_addr(&gImuAddr, false)) {
        gImuReady = false;
        for (uint8_t i = 0; i < 2U; i++) {
            gpio_write(LED_PORT, LED_PIN, true);
            delay_ms(120U);
            gpio_write(LED_PORT, LED_PIN, false);
            delay_ms(120U);
        }
        return false;
    }

    if (!imu_start_motion_sensors(gImuAddr)) {
        gImuReady = false;
        for (uint8_t i = 0; i < 2U; i++) {
            gpio_write(LED_PORT, LED_PIN, true);
            delay_ms(120U);
            gpio_write(LED_PORT, LED_PIN, false);
            delay_ms(120U);
        }
        return false;
    }

    gImuReady = true;
    gpio_write(LED_PORT, LED_PIN, true);
    for (uint16_t i = 0; i < 300U; i++) {
        int16_t gz = 0;

        if (imu_read_gyro_z(&gz)) {
            sum += gz;
            count++;
        }
        delay_ms(5U);
    }
    gpio_write(LED_PORT, LED_PIN, false);

    if (count == 0U) {
        gImuReady = false;
        for (uint8_t i = 0; i < 2U; i++) {
            gpio_write(LED_PORT, LED_PIN, true);
            delay_ms(120U);
            gpio_write(LED_PORT, LED_PIN, false);
            delay_ms(120U);
        }
        return false;
    }

    gGyroZBias = sum / (int32_t)count;
    gpio_write(LED_PORT, LED_PIN, true);
    delay_ms(120U);
    gpio_write(LED_PORT, LED_PIN, false);
    return true;
}

static void run_imu_comm_test(void)
{
    uint8_t addr = IMU_I2C_ADDR;
    int16_t ax0 = 0;
    int16_t ay0 = 0;
    int16_t az0 = 0;
    int16_t gx0 = 0;
    int16_t gy0 = 0;
    int16_t gz0 = 0;

    motors_safe_stop();
    delay_ms(100U);

    if (!imu_detect_addr(&addr, true)) {
        return;
    }
    if (!imu_start_motion_sensors(addr)) {
        notice_imu_fail_code(4U);
        return;
    }

    notice_imu_success();

    gpio_write(LED_PORT, LED_PIN, true);
    delay_ms(700U);
    imu_read_i16_addr(addr, IMU_ACCEL_DATA_X1, &ax0);
    imu_read_i16_addr(addr, IMU_ACCEL_DATA_Y1, &ay0);
    imu_read_i16_addr(addr, IMU_ACCEL_DATA_Z1, &az0);
    imu_read_i16_addr(addr, IMU_GYRO_DATA_X1, &gx0);
    imu_read_i16_addr(addr, IMU_GYRO_DATA_Y1, &gy0);
    imu_read_i16_addr(addr, IMU_GYRO_DATA_Z1, &gz0);
    delay_ms(700U);
    gpio_write(LED_PORT, LED_PIN, false);

    while (1) {
        int16_t axRaw = 0;
        int16_t ayRaw = 0;
        int16_t azRaw = 0;
        int16_t gxRaw = 0;
        int16_t gyRaw = 0;
        int16_t gzRaw = 0;
        int32_t ax;
        int32_t ay;
        int32_t az;
        int32_t gx;
        int32_t gy;
        int32_t gz;

        motors_safe_stop();
        if (!imu_read_i16_addr(addr, IMU_ACCEL_DATA_X1, &axRaw) ||
            !imu_read_i16_addr(addr, IMU_ACCEL_DATA_Y1, &ayRaw) ||
            !imu_read_i16_addr(addr, IMU_ACCEL_DATA_Z1, &azRaw) ||
            !imu_read_i16_addr(addr, IMU_GYRO_DATA_X1, &gxRaw) ||
            !imu_read_i16_addr(addr, IMU_GYRO_DATA_Y1, &gyRaw) ||
            !imu_read_i16_addr(addr, IMU_GYRO_DATA_Z1, &gzRaw)) {
            notice_imu_fail_code(4U);
            return;
        }

        ax = (int32_t)axRaw - ax0;
        ay = (int32_t)ayRaw - ay0;
        az = (int32_t)azRaw - az0;
        gx = (int32_t)gxRaw - gx0;
        gy = (int32_t)gyRaw - gy0;
        gz = (int32_t)gzRaw - gz0;

        if (gz > IMU_GYRO_DEAD_RAW) {
            gpio_write(LED_PORT, LED_PIN, true);
            gpio_write(BEEP_PORT, BEEP_PIN, false);
        } else if (gz < -IMU_GYRO_DEAD_RAW) {
            gpio_write(LED_PORT, LED_PIN, false);
            gpio_write(BEEP_PORT, BEEP_PIN, true);
            delay_ms(20U);
            gpio_write(BEEP_PORT, BEEP_PIN, false);
        } else if ((abs_i32(ax) > IMU_ACCEL_DEAD_RAW) ||
                   (abs_i32(ay) > IMU_ACCEL_DEAD_RAW) ||
                   (abs_i32(az) > IMU_ACCEL_DEAD_RAW) ||
                   (abs_i32(gx) > IMU_GYRO_DEAD_RAW) ||
                   (abs_i32(gy) > IMU_GYRO_DEAD_RAW)) {
            gpio_write(LED_PORT, LED_PIN, true);
            gpio_write(BEEP_PORT, BEEP_PIN, true);
            delay_ms(20U);
            gpio_write(BEEP_PORT, BEEP_PIN, false);
        } else {
            gpio_write(LED_PORT, LED_PIN, false);
            gpio_write(BEEP_PORT, BEEP_PIN, false);
        }

        delay_ms(20U);
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

static void run_straight_to_line_with_duty(uint8_t leftDuty, uint8_t rightDuty,
    uint8_t detectMask, uint8_t minCount, bool brakeAtEnd)
{
    uint32_t confirmMs = 0;

    motors_forward_dir();

    for (uint32_t t = 0; ; t++) {
        uint8_t mask = track_read_mask();

        if ((t > START_IGNORE_MS) && line_detected_with_mask(mask, detectMask, minCount)) {
            confirmMs++;
        } else {
            confirmMs = 0U;
        }

        if (confirmMs >= LINE_DETECT_CONFIRM_MS) {
            gLastLineHitMask = mask;
            break;
        }

        pwm_run_1ms(leftDuty, rightDuty);
    }

    if (brakeAtEnd) {
        active_brake_then_stop();
    }
}

static void run_heading_straight_to_line_with_duty(uint8_t leftDuty, uint8_t rightDuty,
    uint8_t detectMask, uint8_t minCount, bool brakeAtEnd)
{
    uint32_t confirmMs = 0;
    int32_t heading = 0;

    motors_forward_dir();

    for (uint32_t t = 0; ; t++) {
        uint8_t mask = track_read_mask();
        int16_t gzRaw = 0;
        int32_t correction = 0;
        int32_t gz = 0;

        if ((t > START_IGNORE_MS) && line_detected_with_mask(mask, detectMask, minCount)) {
            confirmMs++;
        } else {
            confirmMs = 0U;
        }

        if (confirmMs >= LINE_DETECT_CONFIRM_MS) {
            gLastLineHitMask = mask;
            break;
        }

        if (imu_read_gyro_z(&gzRaw)) {
            gz = (int32_t)gzRaw - gGyroZBias;
            if ((gz > IMU_GYRO_DRIFT_DEAD_RAW) || (gz < -IMU_GYRO_DRIFT_DEAD_RAW)) {
                heading += gz;
            }
            correction = (heading * IMU_HEADING_KP) / IMU_HEADING_CORR_DIV;
        }

        pwm_run_1ms(clamp_duty((int32_t)leftDuty + correction),
            clamp_duty((int32_t)rightDuty - correction));
    }

    if (brakeAtEnd) {
        active_brake_then_stop();
    }
}

static void run_straight_to_line(void)
{
    run_straight_to_line_with_duty(STRAIGHT_LEFT_DUTY, STRAIGHT_RIGHT_DUTY,
        LINE_DETECT_MASK, LINE_DETECT_MIN_COUNT, true);
}

static void run_cd_straight_to_line(void)
{
    run_straight_to_line_with_duty(CD_STRAIGHT_LEFT_DUTY, CD_STRAIGHT_RIGHT_DUTY,
        CD_LINE_DETECT_MASK, CD_LINE_DETECT_MIN_COUNT, false);
}

static void run_ac_straight_to_line(void)
{
    run_heading_straight_to_line_with_duty(AC_STRAIGHT_LEFT_DUTY, AC_STRAIGHT_RIGHT_DUTY,
        AC_LINE_DETECT_MASK, AC_LINE_DETECT_MIN_COUNT, false);
}

static void run_bd_straight_to_line(void)
{
    run_heading_straight_to_line_with_duty(BD_STRAIGHT_LEFT_DUTY, BD_STRAIGHT_RIGHT_DUTY,
        BD_LINE_DETECT_MASK, BD_LINE_DETECT_MIN_COUNT, true);
}

static void bd_turn_toward_d(void)
{
    int32_t turn = 0;

    motors_spin_left_dir();

    for (uint32_t t = 0; t < BD_ALIGN_MAX_MS; t++) {
        int16_t gzRaw = 0;

        if (imu_read_gyro_z(&gzRaw)) {
            int32_t gz = (int32_t)gzRaw - gGyroZBias;

            if ((gz > IMU_GYRO_DRIFT_DEAD_RAW) || (gz < -IMU_GYRO_DRIFT_DEAD_RAW)) {
                turn += gz;
            }
            if (abs_i32(turn) >= BD_ALIGN_TARGET_RAW) {
                break;
            }
        } else if (t >= BD_ALIGN_MS) {
            break;
        }

        pwm_run_1ms(BD_ALIGN_LEFT_DUTY, BD_ALIGN_RIGHT_DUTY);
    }
}

static void cb_entry_capture_line(void)
{
    uint32_t confirmMs = 0;

    motors_forward_dir();

    for (uint32_t t = 0; t < CB_ENTRY_MAX_MS; t++) {
        uint8_t mask = track_read_mask();

        if (line_detected_with_mask(mask, CB_ENTRY_MASK, 1U)) {
            confirmMs++;
        } else {
            confirmMs = 0U;
        }

        if ((t > CB_ENTRY_MIN_MS) && (confirmMs >= CB_ENTRY_CONFIRM_MS)) {
            break;
        }

        pwm_run_1ms(CB_ENTRY_LEFT_DUTY, CB_ENTRY_RIGHT_DUTY);
    }
}

static void cd_exit_align_right(void)
{
    motors_forward_dir();

    for (uint32_t t = 0; t < CD_ALIGN_MS; t++) {
        pwm_run_1ms(CD_ALIGN_LEFT_DUTY, CD_ALIGN_RIGHT_DUTY);
    }
}

static void da_entry_align(void)
{
    int32_t turn = 0;

    motors_spin_right_dir();

    for (uint32_t t = 0; t < DA_ENTRY_MAX_MS; t++) {
        int16_t gzRaw = 0;

        if (imu_read_gyro_z(&gzRaw)) {
            int32_t gz = (int32_t)gzRaw - gGyroZBias;

            if ((gz > IMU_GYRO_DRIFT_DEAD_RAW) || (gz < -IMU_GYRO_DRIFT_DEAD_RAW)) {
                turn += gz;
            }
            if (abs_i32(turn) >= DA_ENTRY_TARGET_RAW) {
                break;
            }
        }

        pwm_run_1ms(DA_ENTRY_LEFT_DUTY, DA_ENTRY_RIGHT_DUTY);
    }
}

static void a_turn_toward_c(void)
{
    int32_t turn = 0;

    motors_spin_right_dir();

    for (uint32_t t = 0; t < A_RESTART_MAX_MS; t++) {
        int16_t gzRaw = 0;

        if (imu_read_gyro_z(&gzRaw)) {
            int32_t gz = (int32_t)gzRaw - gGyroZBias;

            if ((gz > IMU_GYRO_DRIFT_DEAD_RAW) || (gz < -IMU_GYRO_DRIFT_DEAD_RAW)) {
                turn += gz;
            }
            if (abs_i32(turn) >= A_RESTART_TARGET_RAW) {
                break;
            }
        }

        pwm_run_1ms(A_RESTART_LEFT_DUTY, A_RESTART_RIGHT_DUTY);
    }
}

static void da_arc_follow_step(uint8_t mask, int16_t *lastError);
static void da_acquire_follow_step(uint8_t mask, int16_t *lastError);

static void da_entry_arc_left(void)
{
    uint32_t entryMs = 230U;
    uint8_t leftDuty = DA_ENTRY_ARC_LEFT_DUTY;
    uint8_t rightDuty = DA_ENTRY_ARC_RIGHT_DUTY;

    if ((gLastLineHitMask & 0xC0U) != 0U) {
        entryMs = DA_ENTRY_ARC_MS;
        leftDuty = 6U;
        rightDuty = 64U;
    } else if ((gLastLineHitMask & 0x30U) != 0U) {
        entryMs = 250U;
        leftDuty = 8U;
        rightDuty = 60U;
    } else if ((gLastLineHitMask & 0x0CU) != 0U) {
        entryMs = 190U;
        leftDuty = 12U;
        rightDuty = 50U;
    } else if ((gLastLineHitMask & 0x03U) != 0U) {
        entryMs = 140U;
        leftDuty = 18U;
        rightDuty = 42U;
    }

    motors_forward_dir();

    for (uint32_t t = 0; t < entryMs; t++) {
        pwm_run_1ms(leftDuty, rightDuty);
    }
}

static void da_center_on_line(void)
{
    motors_forward_dir();

    for (uint32_t t = 0; t < DA_CENTER_MAX_MS; t++) {
        uint8_t mask = track_read_mask();

        if ((mask & DA_CENTER_MASK) != 0U) {
            break;
        }

        if ((mask & 0xE0U) != 0U) {
            pwm_run_1ms(8U, 82U);
        } else if ((mask & 0x07U) != 0U) {
            pwm_run_1ms(82U, 8U);
        } else {
            pwm_run_1ms(24U, 34U);
        }
    }
}

static void cd_center_on_line(void)
{
    motors_forward_dir();

    for (uint32_t t = 0; t < CD_CENTER_MAX_MS; t++) {
        uint8_t mask = track_read_mask();

        if ((mask & CD_CENTER_MASK) != 0U) {
            break;
        }

        if ((mask & 0xE0U) != 0U) {
            pwm_run_1ms(12U, 72U);
        } else if ((mask & 0x07U) != 0U) {
            pwm_run_1ms(72U, 12U);
        } else {
            pwm_run_1ms(24U, 34U);
        }
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

static void task2_arc_follow_step(uint8_t mask, int16_t *lastError)
{
    uint8_t leftDuty;
    uint8_t rightDuty;

    if (mask != 0U) {
        int16_t error = track_error(mask);
        int32_t correction = (int32_t)error * TASK2_ARC_KP;

        *lastError = error;
        leftDuty = clamp_duty((int32_t)TASK2_ARC_BASE_LEFT_DUTY - correction);
        rightDuty = clamp_duty((int32_t)TASK2_ARC_BASE_RIGHT_DUTY + correction);
    } else if (*lastError < 0) {
        leftDuty = 76U;
        rightDuty = 8U;
    } else if (*lastError > 0) {
        leftDuty = 8U;
        rightDuty = 82U;
    } else {
        leftDuty = TASK2_ARC_BASE_LEFT_DUTY;
        rightDuty = TASK2_ARC_BASE_RIGHT_DUTY;
    }

    pwm_run_1ms(leftDuty, rightDuty);
}

static void task3_line_follow_step(uint8_t mask, int16_t *lastError)
{
    uint8_t leftDuty;
    uint8_t rightDuty;

    if ((mask & 0x03U) != 0U) {
        *lastError = -7;
        leftDuty = 88U;
        rightDuty = 8U;
    } else if ((mask & 0xC0U) != 0U) {
        *lastError = 7;
        leftDuty = 8U;
        rightDuty = 88U;
    } else if (mask != 0U) {
        int16_t error = track_error(mask);
        int32_t correction = (int32_t)error * TASK3_LINE_KP;

        *lastError = error;
        leftDuty = clamp_duty((int32_t)TASK3_LINE_BASE_LEFT_DUTY - correction);
        rightDuty = clamp_duty((int32_t)TASK3_LINE_BASE_RIGHT_DUTY + correction);
    } else if (*lastError < 0) {
        leftDuty = 92U;
        rightDuty = 6U;
    } else if (*lastError > 0) {
        leftDuty = 6U;
        rightDuty = 92U;
    } else {
        leftDuty = TASK3_LINE_BASE_LEFT_DUTY;
        rightDuty = TASK3_LINE_BASE_RIGHT_DUTY;
    }

    pwm_run_1ms(leftDuty, rightDuty);
}

static void da_arc_follow_step(uint8_t mask, int16_t *lastError)
{
    uint8_t leftDuty;
    uint8_t rightDuty;

    if ((mask & 0x03U) != 0U) {
        *lastError = -7;
        leftDuty = 58U;
        rightDuty = 20U;
    } else if ((mask & 0xC0U) != 0U) {
        *lastError = 7;
        leftDuty = 16U;
        rightDuty = 70U;
    } else if (mask != 0U) {
        int16_t error = track_error(mask);
        int32_t correction = (int32_t)error * DA_ARC_KP;

        *lastError = error;
        leftDuty = clamp_duty((int32_t)DA_ARC_BASE_LEFT_DUTY - correction);
        rightDuty = clamp_duty((int32_t)DA_ARC_BASE_RIGHT_DUTY + correction);
    } else if (*lastError < 0) {
        leftDuty = 60U;
        rightDuty = 20U;
    } else if (*lastError > 0) {
        leftDuty = 16U;
        rightDuty = 72U;
    } else {
        leftDuty = DA_ARC_BASE_LEFT_DUTY;
        rightDuty = DA_ARC_BASE_RIGHT_DUTY;
    }

    pwm_run_1ms(leftDuty, rightDuty);
}

static void da_acquire_follow_step(uint8_t mask, int16_t *lastError)
{
    uint8_t leftDuty;
    uint8_t rightDuty;

    if ((mask & 0x03U) != 0U) {
        *lastError = -7;
        leftDuty = 78U;
        rightDuty = 12U;
    } else if ((mask & 0xC0U) != 0U) {
        *lastError = 7;
        leftDuty = 8U;
        rightDuty = 88U;
    } else if (mask != 0U) {
        int16_t error = track_error(mask);
        int32_t correction = (int32_t)error * 9;

        *lastError = error;
        leftDuty = clamp_duty(18 - correction);
        rightDuty = clamp_duty(42 + correction);
    } else if (*lastError < 0) {
        leftDuty = 82U;
        rightDuty = 10U;
    } else {
        *lastError = 7;
        leftDuty = 8U;
        rightDuty = 90U;
    }

    pwm_run_1ms(leftDuty, rightDuty);
}

static void cb_arc_follow_step(uint8_t mask, int16_t *lastError)
{
    uint8_t leftDuty;
    uint8_t rightDuty;

    if ((mask & 0x03U) != 0U) {
        *lastError = -7;
        leftDuty = 96U;
        rightDuty = 4U;
    } else if ((mask & 0xC0U) != 0U) {
        *lastError = 7;
        leftDuty = 36U;
        rightDuty = 52U;
    } else if (mask != 0U) {
        int16_t error = track_error(mask);
        int32_t correction = (int32_t)error * CB_ARC_KP;

        *lastError = error;
        leftDuty = clamp_duty((int32_t)CB_ARC_BASE_LEFT_DUTY - correction);
        rightDuty = clamp_duty((int32_t)CB_ARC_BASE_RIGHT_DUTY + correction);
    } else if (*lastError < 0) {
        leftDuty = 98U;
        rightDuty = 3U;
    } else if (*lastError > 0) {
        leftDuty = 28U;
        rightDuty = 58U;
    } else {
        leftDuty = CB_ARC_BASE_LEFT_DUTY;
        rightDuty = CB_ARC_BASE_RIGHT_DUTY;
    }

    pwm_run_1ms(leftDuty, rightDuty);
}

static void settle_on_arc(void)
{
    int16_t lastError = 0;

    motors_forward_dir();

    for (uint32_t t = 0; t < ARC_SETTLE_MS; t++) {
        uint8_t mask = track_read_mask();
        task2_arc_follow_step(mask, &lastError);
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

        if (lostConfirmMs >= TASK2_ARC_LOST_CONFIRM_MS) {
            break;
        }

        task2_arc_follow_step(mask, &lastError);
    }

    active_brake_then_stop();
}

static void cb_run_arc_until_lost(void)
{
    uint32_t lostConfirmMs = 0;
    uint32_t pointConfirmMs = 0;
    int16_t lastError = -7;

    motors_forward_dir();

    for (uint32_t t = 0; ; t++) {
        uint8_t mask = track_read_mask();
        uint8_t blackCount = bit_count(mask);

        if ((t > ARC_MIN_RUN_MS) && ((mask & ARC_LOST_MASK) == 0U)) {
            lostConfirmMs++;
        } else {
            lostConfirmMs = 0U;
        }

        if ((t > CB_ARC_POINT_MIN_MS) && (blackCount >= CB_ARC_POINT_MIN_COUNT)) {
            pointConfirmMs++;
        } else {
            pointConfirmMs = 0U;
        }

        if ((lostConfirmMs >= ARC_LOST_CONFIRM_MS) ||
            (pointConfirmMs >= CB_ARC_POINT_CONFIRM_MS) ||
            (t >= CB_ARC_MAX_MS)) {
            break;
        }

        cb_arc_follow_step(mask, &lastError);
    }

    active_brake_then_stop();
}

static void task3_settle_on_arc(void)
{
    int16_t lastError = 0;

    motors_forward_dir();

    for (uint32_t t = 0; t < ARC_SETTLE_MS; t++) {
        uint8_t mask = track_read_mask();
        task3_line_follow_step(mask, &lastError);
    }
}

static void task3_run_arc_until_lost(void)
{
    uint32_t lostConfirmMs = 0;
    int16_t lastError = 0;

    motors_forward_dir();
    task3_settle_on_arc();

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

        task3_line_follow_step(mask, &lastError);
    }

    active_brake_then_stop();
}

static void da_run_arc_until_lost(void)
{
    uint32_t lostConfirmMs = 0;
    int16_t lastError = 7;

    motors_forward_dir();

    for (uint32_t t = 0; t < DA_ACQUIRE_MS; t++) {
        uint8_t mask = track_read_mask();
        da_acquire_follow_step(mask, &lastError);
    }

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

        da_arc_follow_step(mask, &lastError);
    }

    active_brake_then_stop();
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
#if TASK2_STOP_AT_C_DEBUG
    return;
#endif
    delay_ms(80U);
    cd_exit_align_right();
    cd_center_on_line();
    run_cd_straight_to_line();
    notice_pass_point();
    delay_ms(30U);
    run_arc_until_lost();
    notice_arrived();
}

static void run_task_3_lap(bool finalLap)
{
    run_ac_straight_to_line();
    notice_pass_point();
    delay_ms(30U);
    cb_entry_capture_line();
    cb_run_arc_until_lost();
    notice_pass_point();
    delay_ms(20U);
    bd_turn_toward_d();
    run_bd_straight_to_line();
    notice_pass_point();
    delay_ms(80U);
    da_entry_arc_left();
    da_center_on_line();
    da_run_arc_until_lost();

    if (finalLap) {
        notice_arrived();
    } else {
        notice_pass_point();
        delay_ms(60U);
        a_turn_toward_c();
        delay_ms(30U);
    }
}

static void run_task_3(void)
{
    run_task_3_lap(true);
}

static void run_task_4(void)
{
    for (uint8_t lap = 0; lap < 4U; lap++) {
        run_task_3_lap(lap == 3U);
    }
}

static void run_tasks_1_to_3_by_key(void)
{
    wait_start_key();
    run_task_1();
    motors_safe_stop();

    wait_start_key();
    run_task_2();
    motors_safe_stop();

    wait_start_key();
    run_task_3();
    motors_safe_stop();
}

int main(void)
{
    SYSCFG_DL_init();
    gpio_write(MOTOR_STBY_PORT, MOTOR_STBY_PIN, true);
    gpio_write(LED_PORT, LED_PIN, false);
    gpio_write(BEEP_PORT, BEEP_PIN, false);
    motors_safe_stop();

#if IMU_TEST_MODE
    run_imu_comm_test();
    while (1) {
        motors_safe_stop();
        delay_ms(20U);
    }
#endif

    imu_init_for_route();

#if TASK_MODE == 123U
    run_tasks_1_to_3_by_key();
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
