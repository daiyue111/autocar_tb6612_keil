#include "task2.h"

#include "app_io.h"
#include "imu.h"
#include "motor.h"
#include "track.h"

#define TASK2_AB_LEFT_DUTY 24U
#define TASK2_AB_RIGHT_DUTY 20U
#define TASK2_CD_LEFT_DUTY 22U
#define TASK2_CD_RIGHT_DUTY 24U
#define TASK2_STRAIGHT_IGNORE_MS 1000U
#define TASK2_STRAIGHT_CONFIRM_MS 3U
#define TASK2_STRAIGHT_MAX_MS 15000U
#define TASK2_CD_HEADING_KP 2
#define TASK2_CD_HEADING_CORR_DIV 900
#define TASK2_CD_HEADING_CORR_MAX 6
#define TASK2_CD_GYRO_DEAD_RAW 8
#define TASK2_STOP_BRAKE_MS 120U
#define TASK2_C_ALIGN_RIGHT_DUTY 20U
#define TASK2_C_ALIGN_RIGHT_MS 155U

#define TASK2_ARC_LEFT_DUTY 18U
#define TASK2_ARC_RIGHT_DUTY 20U
#define TASK2_ARC_KP 5
#define TASK2_ARC_MIN_MS 1200U
#define TASK2_ARC_LOST_CONFIRM_MS 50U
#define TASK2_ARC_MAX_MS 15000U

static uint8_t clamp_duty(int32_t duty)
{
    if (duty < 0) {
        return 0U;
    }
    if (duty > 100) {
        return 100U;
    }
    return (uint8_t)duty;
}

static bool run_straight_to_line(uint8_t leftDuty, uint8_t rightDuty)
{
    uint16_t lineMs = 0U;

    for (uint32_t t = 0U; t < TASK2_STRAIGHT_MAX_MS; t++) {
        uint8_t mask = track_read_mask();

        if ((t >= TASK2_STRAIGHT_IGNORE_MS) && (mask != 0U)) {
            if (lineMs < TASK2_STRAIGHT_CONFIRM_MS) {
                lineMs++;
            }
        } else {
            lineMs = 0U;
        }

        if (lineMs >= TASK2_STRAIGHT_CONFIRM_MS) {
            motor_short_brake_ms(TASK2_STOP_BRAKE_MS);
            return true;
        }

        motor_forward_pwm_1ms(leftDuty, rightDuty);
    }

    motor_active_brake_then_stop();
    return false;
}

static bool run_gyro_straight_to_line(uint8_t leftDuty, uint8_t rightDuty)
{
    uint16_t lineMs = 0U;
    uint16_t failMs = 0U;
    int32_t heading = 0;

    for (uint32_t t = 0U; t < TASK2_STRAIGHT_MAX_MS; t++) {
        uint8_t mask = track_read_mask();
        int32_t gz = 0;
        int32_t correction = 0;

        if ((t >= TASK2_STRAIGHT_IGNORE_MS) && (mask != 0U)) {
            if (lineMs < TASK2_STRAIGHT_CONFIRM_MS) {
                lineMs++;
            }
        } else {
            lineMs = 0U;
        }

        if (lineMs >= TASK2_STRAIGHT_CONFIRM_MS) {
            motor_short_brake_ms(TASK2_STOP_BRAKE_MS);
            return true;
        }

        if (imu_read_gyro_z_delta(&gz)) {
            failMs = 0U;
            if ((gz > TASK2_CD_GYRO_DEAD_RAW) || (gz < -TASK2_CD_GYRO_DEAD_RAW)) {
                heading += gz;
            }
        } else if (failMs < 1000U) {
            failMs++;
        }

        if (failMs >= 100U) {
            motor_active_brake_then_stop();
            return false;
        }

        correction = -((heading * TASK2_CD_HEADING_KP) / TASK2_CD_HEADING_CORR_DIV);
        if (correction > TASK2_CD_HEADING_CORR_MAX) {
            correction = TASK2_CD_HEADING_CORR_MAX;
        } else if (correction < -TASK2_CD_HEADING_CORR_MAX) {
            correction = -TASK2_CD_HEADING_CORR_MAX;
        }
        motor_forward_pwm_1ms(clamp_duty((int32_t)leftDuty + correction),
            clamp_duty((int32_t)rightDuty - correction));
    }

    motor_active_brake_then_stop();
    return false;
}

static bool follow_arc_to_point(void)
{
    int16_t lastError = 0;
    uint16_t lostMs = 0U;

    for (uint32_t t = 0U; t < TASK2_ARC_MAX_MS; t++) {
        uint8_t mask = track_read_mask();

        if ((t >= TASK2_ARC_MIN_MS) && (mask == 0U)) {
            if (lostMs < TASK2_ARC_LOST_CONFIRM_MS) {
                lostMs++;
            }
        } else {
            lostMs = 0U;
        }

        if (lostMs >= TASK2_ARC_LOST_CONFIRM_MS) {
            motor_short_brake_ms(TASK2_STOP_BRAKE_MS);
            return true;
        }

        if (mask != 0U) {
            lastError = track_error(mask);
        }

        int32_t correction = ((int32_t)lastError * TASK2_ARC_KP) / 10;
        uint8_t leftDuty = clamp_duty((int32_t)TASK2_ARC_LEFT_DUTY + correction);
        uint8_t rightDuty = clamp_duty((int32_t)TASK2_ARC_RIGHT_DUTY - correction);

        motor_forward_pwm_1ms(leftDuty, rightDuty);
    }

    motor_active_brake_then_stop();
    return false;
}

static void align_right_before_cd(void)
{
    for (uint32_t t = 0U; t < TASK2_C_ALIGN_RIGHT_MS; t++) {
        motor_spin_right_pwm_1ms(TASK2_C_ALIGN_RIGHT_DUTY);
    }
    motor_short_brake_ms(60U);
}

void task2_run(void)
{
    if (!imu_init_gyro_z()) {
        app_led_blink(2U);
        return;
    }

    if (!run_straight_to_line(TASK2_AB_LEFT_DUTY, TASK2_AB_RIGHT_DUTY)) {
        app_led_blink(3U);
        return;
    }
    app_beep_short();
    app_led_blink(1U);

    app_delay_ms(150U);

    if (!follow_arc_to_point()) {
        app_led_blink(3U);
        return;
    }
    app_beep_short();
    app_led_blink(1U);

    app_delay_ms(150U);
    align_right_before_cd();

    if (!run_gyro_straight_to_line(TASK2_CD_LEFT_DUTY, TASK2_CD_RIGHT_DUTY)) {
        app_led_blink(3U);
        return;
    }
    app_beep_short();
    app_led_blink(1U);

    app_delay_ms(150U);

    if (!follow_arc_to_point()) {
        app_led_blink(3U);
        return;
    }
    app_beep_short();
    app_led_blink(1U);
}
