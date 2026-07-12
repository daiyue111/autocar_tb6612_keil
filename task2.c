#include "task2.h"

#include "app_io.h"
#include "imu.h"
#include "motor.h"
#include "track.h"

#define TASK2_AB_LEFT_DUTY 24U
#define TASK2_AB_RIGHT_DUTY 20U
#define TASK2_CD_LEFT_DUTY 23U
#define TASK2_CD_RIGHT_DUTY 23U
#define TASK2_STRAIGHT_IGNORE_MS 1000U
#define TASK2_STRAIGHT_CONFIRM_MS 3U
#define TASK2_STRAIGHT_MAX_MS 15000U
#define TASK2_HEADING_HALF_RAW 3600000
#define TASK2_HEADING_FULL_RAW 7200000
#define TASK2_HEADING_KP 2
#define TASK2_HEADING_CORR_DIV 450000
#define TASK2_HEADING_CORR_MAX 8
#define TASK2_CD_HEADING_CORR_DIV 150000
#define TASK2_CD_HEADING_CORR_MAX 18
#define TASK2_HEADING_DEAD_RAW 8
#define TASK2_HEADING_SAMPLE_SCALE 4
#define TASK2_STRAIGHT_RATE_CORR_DIV 1200
#define TASK2_STRAIGHT_RATE_CORR_MAX 4
#define TASK2_CD_RATE_CORR_DIV 350
#define TASK2_CD_RATE_CORR_MAX 10
#define TASK2_STRAIGHT_RATE_DEAD_RAW 25
#define TASK2_STOP_BRAKE_MS 120U
#define TASK2_C_ALIGN_RIGHT_RAW 680000
#define TASK2_C_ALIGN_RIGHT_DUTY 22U
#define TASK2_C_ALIGN_RIGHT_MAX_MS 900U
#define TASK2_C_ALIGN_SAMPLE_MS 8U
#define TASK2_C_ALIGN_SAMPLE_MAX_RAW 12000

#define TASK2_ARC_LEFT_DUTY 17U
#define TASK2_ARC_RIGHT_DUTY 19U
#define TASK2_ARC_KP 6
#define TASK2_ARC_HEADING_CORR_MAX 1
#define TASK2_ARC_MIN_MS 1200U
#define TASK2_ARC_TARGET_MS 6000U
#define TASK2_ARC_LOST_CONFIRM_MS 80U
#define TASK2_ARC_MAX_MS 15000U

typedef struct {
    int32_t raw;
    uint16_t failMs;
} Task2Heading;

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

static int32_t clamp_i32(int32_t value, int32_t limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static bool heading_update_with_delta(Task2Heading *heading, int32_t *delta)
{
    int32_t gz = 0;

    if (imu_read_gyro_z_delta(&gz)) {
        heading->failMs = 0U;
        *delta = gz;
        if ((gz > TASK2_HEADING_DEAD_RAW) || (gz < -TASK2_HEADING_DEAD_RAW)) {
            heading->raw += gz * (int32_t)TASK2_HEADING_SAMPLE_SCALE;
        }
        return true;
    }

    if (heading->failMs < 1000U) {
        heading->failMs++;
    }
    return heading->failMs < 100U;
}

static bool heading_update(Task2Heading *heading)
{
    int32_t delta = 0;

    return heading_update_with_delta(heading, &delta);
}

static int32_t heading_correction(Task2Heading *heading, int32_t targetRaw,
    int32_t corrDiv, int32_t limit)
{
    int32_t error = heading->raw - targetRaw;
    int32_t correction = -((error * TASK2_HEADING_KP) / corrDiv);

    return clamp_i32(correction, limit);
}

static bool run_heading_straight_to_line(Task2Heading *heading,
    int32_t targetRaw, uint8_t leftDuty, uint8_t rightDuty,
    int32_t headingDiv, int32_t headingLimit, int32_t rateDiv,
    int32_t rateLimit)
{
    uint16_t lineMs = 0U;

    for (uint32_t t = 0U; t < TASK2_STRAIGHT_MAX_MS; t++) {
        uint8_t mask = track_read_mask();
        int32_t gz = 0;
        int32_t correction = 0;
        int32_t rateCorrection = 0;

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

        if (!heading_update_with_delta(heading, &gz)) {
            motor_active_brake_then_stop();
            return false;
        }

        correction = heading_correction(heading, targetRaw, headingDiv,
            headingLimit);
        if ((gz > TASK2_STRAIGHT_RATE_DEAD_RAW) ||
            (gz < -TASK2_STRAIGHT_RATE_DEAD_RAW)) {
            rateCorrection = -(gz / rateDiv);
            rateCorrection = clamp_i32(rateCorrection, rateLimit);
        }
        correction = clamp_i32(correction + rateCorrection, headingLimit);
        motor_forward_pwm_1ms(clamp_duty((int32_t)leftDuty + correction),
            clamp_duty((int32_t)rightDuty - correction));
    }

    motor_active_brake_then_stop();
    return false;
}

static bool follow_arc_to_point(Task2Heading *heading, int32_t startRaw,
    int32_t targetRaw)
{
    int16_t lastError = 0;
    uint16_t lostMs = 0U;

    for (uint32_t t = 0U; t < TASK2_ARC_MAX_MS; t++) {
        uint8_t mask = track_read_mask();
        int32_t arcTarget = targetRaw;
        int32_t headingAssist = 0;

        if ((t >= TASK2_ARC_MIN_MS) && (mask == 0U)) {
            if (lostMs < TASK2_ARC_LOST_CONFIRM_MS) {
                lostMs++;
            }
        } else {
            lostMs = 0U;
        }

        if (lostMs >= TASK2_ARC_LOST_CONFIRM_MS) {
            motor_short_brake_ms(TASK2_STOP_BRAKE_MS);
            heading->raw = targetRaw;
            return true;
        }

        if (mask != 0U) {
            lastError = track_error(mask);
        }

        int32_t correction = ((int32_t)lastError * TASK2_ARC_KP) / 10;

        if (t < TASK2_ARC_TARGET_MS) {
            arcTarget = startRaw + (((targetRaw - startRaw) * (int32_t)t) /
                (int32_t)TASK2_ARC_TARGET_MS);
        }

        if (!heading_update(heading)) {
            motor_active_brake_then_stop();
            return false;
        }
        headingAssist = heading_correction(heading, arcTarget,
            TASK2_HEADING_CORR_DIV,
            TASK2_ARC_HEADING_CORR_MAX);

        uint8_t leftDuty = clamp_duty((int32_t)TASK2_ARC_LEFT_DUTY + correction +
            headingAssist);
        uint8_t rightDuty = clamp_duty((int32_t)TASK2_ARC_RIGHT_DUTY - correction -
            headingAssist);

        motor_forward_pwm_1ms(leftDuty, rightDuty);
    }

    motor_active_brake_then_stop();
    return false;
}

static bool align_right_by_gyro(Task2Heading *heading, int32_t alignRaw,
    int32_t targetHeadingRaw)
{
    int32_t turn = 0;
    uint16_t readCount = 0U;

    for (uint32_t t = 0U; t < TASK2_C_ALIGN_RIGHT_MAX_MS; t++) {
        if ((t % TASK2_C_ALIGN_SAMPLE_MS) == 0U) {
            int32_t gz = 0;

            motor_pwm_off();
            delay_cycles(CPUCLK_FREQ / 20000U);
            if (imu_read_gyro_z_delta(&gz)) {
                int32_t sample = (gz < 0) ? -gz : gz;

                readCount++;
                if (sample > TASK2_C_ALIGN_SAMPLE_MAX_RAW) {
                    sample = TASK2_C_ALIGN_SAMPLE_MAX_RAW;
                }
                if (sample > TASK2_HEADING_DEAD_RAW) {
                    turn += sample * (int32_t)TASK2_C_ALIGN_SAMPLE_MS *
                        (int32_t)TASK2_HEADING_SAMPLE_SCALE;
                }
            }
        }

        motor_spin_right_pwm_1ms(TASK2_C_ALIGN_RIGHT_DUTY);

        if (turn >= alignRaw) {
            motor_short_brake_ms(TASK2_STOP_BRAKE_MS);
            heading->raw = targetHeadingRaw;
            heading->failMs = 0U;
            return true;
        }
    }

    motor_active_brake_then_stop();
    return readCount > 0U;
}

void task2_run(void)
{
    Task2Heading heading = {0, 0U};

    if (!imu_init_gyro_z()) {
        app_led_blink(2U);
        return;
    }

    if (!run_heading_straight_to_line(&heading, 0, TASK2_AB_LEFT_DUTY,
            TASK2_AB_RIGHT_DUTY, TASK2_HEADING_CORR_DIV,
            TASK2_HEADING_CORR_MAX, TASK2_STRAIGHT_RATE_CORR_DIV,
            TASK2_STRAIGHT_RATE_CORR_MAX)) {
        app_led_blink(3U);
        return;
    }
    app_beep_short();
    app_led_blink(1U);

    app_delay_ms(150U);

    if (!follow_arc_to_point(&heading, 0, TASK2_HEADING_HALF_RAW)) {
        app_led_blink(3U);
        return;
    }
    app_beep_short();
    app_led_blink(1U);

    app_delay_ms(150U);

    if (!align_right_by_gyro(&heading, TASK2_C_ALIGN_RIGHT_RAW,
            TASK2_HEADING_HALF_RAW)) {
        app_led_blink(4U);
        return;
    }

    if (!run_heading_straight_to_line(&heading, TASK2_HEADING_HALF_RAW,
            TASK2_CD_LEFT_DUTY, TASK2_CD_RIGHT_DUTY,
            TASK2_CD_HEADING_CORR_DIV, TASK2_CD_HEADING_CORR_MAX,
            TASK2_CD_RATE_CORR_DIV, TASK2_CD_RATE_CORR_MAX)) {
        app_led_blink(3U);
        return;
    }
    app_beep_short();
    app_led_blink(1U);

    app_delay_ms(150U);

    if (!follow_arc_to_point(&heading, TASK2_HEADING_HALF_RAW,
            TASK2_HEADING_FULL_RAW)) {
        app_led_blink(3U);
        return;
    }
    app_beep_short();
    app_led_blink(1U);
}
