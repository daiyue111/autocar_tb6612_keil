#include "imu.h"

#include "app_io.h"
#include "motor.h"

#define SPI_TIMEOUT_CYCLES 40000U
#define IMU_WHO_AM_I_REG 0x75U
#define IMU_WHO_AM_I_EXPECTED 0x47U
#define IMU_REG_BANK_SEL 0x76U
#define IMU_PWR_MGMT0 0x4EU
#define IMU_GYRO_CONFIG0 0x4FU
#define IMU_GYRO_DATA_Z1 0x29U
#define IMU_BIAS_SETTLE_MS 220U
#define IMU_BIAS_SAMPLES 32U
#define IMU_BIAS_SAMPLE_MS 4U

static int32_t gGyroZBias = 0;
static bool gGyroReady = false;

static bool spi_transfer_byte(uint8_t tx, uint8_t *rx)
{
    while (!DL_SPI_isRXFIFOEmpty(IMU_SPI)) {
        (void) DL_SPI_receiveData8(IMU_SPI);
    }

    for (uint32_t t = 0U; t < SPI_TIMEOUT_CYCLES; t++) {
        if (!DL_SPI_isTXFIFOFull(IMU_SPI)) {
            DL_SPI_transmitData8(IMU_SPI, tx);
            break;
        }
        if (t == (SPI_TIMEOUT_CYCLES - 1U)) {
            return false;
        }
    }

    for (uint32_t t = 0U; t < SPI_TIMEOUT_CYCLES; t++) {
        if (!DL_SPI_isRXFIFOEmpty(IMU_SPI)) {
            *rx = DL_SPI_receiveData8(IMU_SPI);
            return true;
        }
    }

    return false;
}

static bool imu_spi_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t dummy = 0U;

    app_gpio_write(IMU_SPI_CS_PORT, IMU_SPI_CS_PIN, false);
    delay_cycles(8U);

    if (!spi_transfer_byte((uint8_t)(reg | 0x80U), &dummy) ||
        !spi_transfer_byte(0x00U, value)) {
        app_gpio_write(IMU_SPI_CS_PORT, IMU_SPI_CS_PIN, true);
        return false;
    }

    while (DL_SPI_isBusy(IMU_SPI)) {
    }
    delay_cycles(8U);
    app_gpio_write(IMU_SPI_CS_PORT, IMU_SPI_CS_PIN, true);
    return true;
}

static bool imu_spi_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t dummy = 0U;

    app_gpio_write(IMU_SPI_CS_PORT, IMU_SPI_CS_PIN, false);
    delay_cycles(8U);

    if (!spi_transfer_byte((uint8_t)(reg & 0x7FU), &dummy) ||
        !spi_transfer_byte(value, &dummy)) {
        app_gpio_write(IMU_SPI_CS_PORT, IMU_SPI_CS_PIN, true);
        return false;
    }

    while (DL_SPI_isBusy(IMU_SPI)) {
    }
    delay_cycles(8U);
    app_gpio_write(IMU_SPI_CS_PORT, IMU_SPI_CS_PIN, true);
    return true;
}

static bool imu_spi_read_i16(uint8_t regHigh, int16_t *value)
{
    uint8_t high = 0U;
    uint8_t low = 0U;

    if (!imu_spi_read_reg(regHigh, &high) ||
        !imu_spi_read_reg((uint8_t)(regHigh + 1U), &low)) {
        return false;
    }

    *value = (int16_t)(((uint16_t)high << 8) | low);
    return true;
}

static bool imu_start_gyro(void)
{
    uint8_t who = 0U;

    app_delay_ms(120U);
    if (!imu_spi_read_reg(IMU_WHO_AM_I_REG, &who) ||
        (who != IMU_WHO_AM_I_EXPECTED)) {
        return false;
    }
    if (!imu_spi_write_reg(IMU_REG_BANK_SEL, 0x00U)) {
        return false;
    }
    if (!imu_spi_write_reg(IMU_GYRO_CONFIG0, 0x06U)) {
        return false;
    }
    if (!imu_spi_write_reg(IMU_PWR_MGMT0, 0x0FU)) {
        return false;
    }
    app_delay_ms(200U);
    return true;
}

bool imu_init_gyro_z(void)
{
    int32_t sum = 0;
    uint8_t count = 0U;

    gGyroReady = false;
    if (!imu_start_gyro()) {
        return false;
    }

    motor_safe_stop();
    app_delay_ms(IMU_BIAS_SETTLE_MS);

    for (uint8_t i = 0U; i < IMU_BIAS_SAMPLES; i++) {
        int16_t gz = 0;

        if (imu_spi_read_i16(IMU_GYRO_DATA_Z1, &gz)) {
            sum += gz;
            count++;
        }
        app_delay_ms(IMU_BIAS_SAMPLE_MS);
    }

    if (count == 0U) {
        return false;
    }

    gGyroZBias = sum / (int32_t)count;
    gGyroReady = true;
    return true;
}

bool imu_read_gyro_z_delta(int32_t *delta)
{
    int16_t gz = 0;

    if (!gGyroReady || !imu_spi_read_i16(IMU_GYRO_DATA_Z1, &gz)) {
        return false;
    }

    *delta = (int32_t)gz - gGyroZBias;
    return true;
}
