#include "task3.h"

#include "app_io.h"
#include "imu.h"
#include "motor.h"
#include "track.h"

#define TASK3_A_TO_AC_TURN_RAW 1280000
#define TASK3_B_TO_BD_TURN_RAW 1610000
#define TASK3_TURN_DUTY 26U
#define TASK3_TURN_KICK_DUTY 32U
#define TASK3_TURN_KICK_MS 30U
#define TASK3_TURN_IGNORE_MS 40U
#define TASK3_TURN_MIN_MS 80U
#define TASK3_TURN_MAX_MS 3000U
#define TASK3_TURN_SAMPLE_MS 8U
#define TASK3_TURN_SAMPLE_SCALE 4
#define TASK3_TURN_SAMPLE_MAX_RAW 12000
#define TASK3_TURN_DEAD_RAW 8

#define TASK3_STRAIGHT_LEFT_DUTY 22U
#define TASK3_STRAIGHT_RIGHT_DUTY 24U
#define TASK3_STRAIGHT_IGNORE_MS 1000U
#define TASK3_STRAIGHT_CONFIRM_MS 3U
#define TASK3_STRAIGHT_MAX_MS 12000U
#define TASK3_HEADING_KP 2
#define TASK3_HEADING_CORR_DIV 900
#define TASK3_HEADING_CORR_MAX 6
#define TASK3_GYRO_DEAD_RAW 8
#define TASK3_STOP_BRAKE_MS 120U

#define TASK3_C_ALIGN_LEFT_DUTY 20U
#define TASK3_C_ALIGN_LEFT_MS 150U
#define TASK3_D_TO_DA_ALIGN_RAW 950000
#define TASK3_ARC_LEFT_DUTY 17U
#define TASK3_ARC_RIGHT_DUTY 19U
#define TASK3_ARC_KP 6
#define TASK3_ARC_MIN_MS 1200U
#define TASK3_ARC_LOST_CONFIRM_MS 80U
#define TASK3_ARC_MAX_MS 15000U

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

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

static bool turn_right_by_gyro(int32_t targetRaw)
{
    int32_t turn = 0;
    uint16_t readCount = 0U;

    for (uint32_t t = 0U; t < TASK3_TURN_MAX_MS; t++) {
        uint8_t duty = (t < TASK3_TURN_KICK_MS) ? TASK3_TURN_KICK_DUTY :
            TASK3_TURN_DUTY;

        if ((t % TASK3_TURN_SAMPLE_MS) == 0U) {
            int32_t gz = 0;

            motor_pwm_off();
            delay_cycles(CPUCLK_FREQ / 20000U);
            if (imu_read_gyro_z_delta(&gz)) {
                int32_t sample = abs_i32(gz);

                readCount++;
                if (t >= TASK3_TURN_IGNORE_MS) {
                    if (sample > TASK3_TURN_SAMPLE_MAX_RAW) {
                        sample = TASK3_TURN_SAMPLE_MAX_RAW;
                    }
                    if (sample > TASK3_TURN_DEAD_RAW) {
                        turn += sample * (int32_t)TASK3_TURN_SAMPLE_MS *
                            (int32_t)TASK3_TURN_SAMPLE_SCALE;
                    }
                }
            }
        }

        motor_spin_right_pwm_1ms(duty);

        if ((t >= TASK3_TURN_MIN_MS) && (turn >= targetRaw)) {
            motor_short_brake_ms(TASK3_STOP_BRAKE_MS);
            return true;
        }
    }

    motor_active_brake_then_stop();
    return readCount > 0U;
}

static bool turn_left_by_gyro(int32_t targetRaw)
{
    int32_t turn = 0;
    uint16_t readCount = 0U;

    for (uint32_t t = 0U; t < TASK3_TURN_MAX_MS; t++) {
        uint8_t duty = (t < TASK3_TURN_KICK_MS) ? TASK3_TURN_KICK_DUTY :
            TASK3_TURN_DUTY;

        if ((t % TASK3_TURN_SAMPLE_MS) == 0U) {
            int32_t gz = 0;

            motor_pwm_off();
            delay_cycles(CPUCLK_FREQ / 20000U);
            if (imu_read_gyro_z_delta(&gz)) {
                int32_t sample = abs_i32(gz);

                readCount++;
                if (t >= TASK3_TURN_IGNORE_MS) {
                    if (sample > TASK3_TURN_SAMPLE_MAX_RAW) {
                        sample = TASK3_TURN_SAMPLE_MAX_RAW;
                    }
                    if (sample > TASK3_TURN_DEAD_RAW) {
                        turn += sample * (int32_t)TASK3_TURN_SAMPLE_MS *
                            (int32_t)TASK3_TURN_SAMPLE_SCALE;
                    }
                }
            }
        }

        motor_spin_left_pwm_1ms(duty);

        if ((t >= TASK3_TURN_MIN_MS) && (turn >= targetRaw)) {
            motor_short_brake_ms(TASK3_STOP_BRAKE_MS);
            return true;
        }
    }

    motor_active_brake_then_stop();
    return readCount > 0U;
}

static bool gyro_straight_to_line(void)
{
    uint16_t lineMs = 0U;
    uint16_t failMs = 0U;
    int32_t heading = 0;

    for (uint32_t t = 0U; t < TASK3_STRAIGHT_MAX_MS; t++) {
        uint8_t mask = track_read_mask();
        int32_t gz = 0;
        int32_t correction = 0;

        if ((t >= TASK3_STRAIGHT_IGNORE_MS) && (mask != 0U)) {
            if (lineMs < TASK3_STRAIGHT_CONFIRM_MS) {
                lineMs++;
            }
        } else {
            lineMs = 0U;
        }

        if (lineMs >= TASK3_STRAIGHT_CONFIRM_MS) {
            motor_short_brake_ms(TASK3_STOP_BRAKE_MS);
            return true;
        }

        if (imu_read_gyro_z_delta(&gz)) {
            failMs = 0U;
            if ((gz > TASK3_GYRO_DEAD_RAW) || (gz < -TASK3_GYRO_DEAD_RAW)) {
                heading += gz;
            }
        } else if (failMs < 1000U) {
            failMs++;
        }

        if (failMs >= 100U) {
            motor_active_brake_then_stop();
            return false;
        }

        correction = -((heading * TASK3_HEADING_KP) / TASK3_HEADING_CORR_DIV);
        if (correction > TASK3_HEADING_CORR_MAX) {
            correction = TASK3_HEADING_CORR_MAX;
        } else if (correction < -TASK3_HEADING_CORR_MAX) {
            correction = -TASK3_HEADING_CORR_MAX;
        }

        motor_forward_pwm_1ms(clamp_duty((int32_t)TASK3_STRAIGHT_LEFT_DUTY +
                correction),
            clamp_duty((int32_t)TASK3_STRAIGHT_RIGHT_DUTY - correction));
    }

    motor_active_brake_then_stop();
    return false;
}

static void align_left_before_cb(void)
{
    for (uint32_t t = 0U; t < TASK3_C_ALIGN_LEFT_MS; t++) {
        motor_spin_left_pwm_1ms(TASK3_C_ALIGN_LEFT_DUTY);
    }
    motor_short_brake_ms(60U);
}

static bool align_right_before_da(void)
{
    return turn_right_by_gyro(TASK3_D_TO_DA_ALIGN_RAW);
}

static bool follow_arc_to_point(void)
{
    int16_t lastError = 0;
    uint16_t lostMs = 0U;

    for (uint32_t t = 0U; t < TASK3_ARC_MAX_MS; t++) {
        uint8_t mask = track_read_mask();

        if ((t >= TASK3_ARC_MIN_MS) && (mask == 0U)) {
            if (lostMs < TASK3_ARC_LOST_CONFIRM_MS) {
                lostMs++;
            }
        } else {
            lostMs = 0U;
        }

        if (lostMs >= TASK3_ARC_LOST_CONFIRM_MS) {
            motor_short_brake_ms(TASK3_STOP_BRAKE_MS);
            return true;
        }

        if (mask != 0U) {
            lastError = track_error(mask);
        }

        int32_t correction = ((int32_t)lastError * TASK3_ARC_KP) / 10;
        uint8_t leftDuty = clamp_duty((int32_t)TASK3_ARC_LEFT_DUTY + correction);
        uint8_t rightDuty = clamp_duty((int32_t)TASK3_ARC_RIGHT_DUTY - correction);

        motor_forward_pwm_1ms(leftDuty, rightDuty);
    }

    motor_active_brake_then_stop();
    return false;
}

bool task3_run_one_lap_with_turns(int32_t aToAcTurnRaw,
    int32_t bToBdTurnRaw)
{
    if (!turn_right_by_gyro(aToAcTurnRaw)) {
        app_led_blink(4U);
        return false;
    }
    app_beep_short();
    app_led_blink(1U);

    app_delay_ms(120U);

    if (!gyro_straight_to_line()) {
        app_led_blink(3U);
        return false;
    }
    app_beep_short();
    app_led_blink(1U);

    app_delay_ms(120U);
    align_left_before_cb();

    if (!follow_arc_to_point()) {
        app_led_blink(3U);
        return false;
    }
    app_beep_short();
    app_led_blink(1U);

    app_delay_ms(120U);

    if (!turn_left_by_gyro(bToBdTurnRaw)) {
        app_led_blink(4U);
        return false;
    }
    app_beep_short();
    app_led_blink(1U);

    app_delay_ms(120U);

    if (!gyro_straight_to_line()) {
        app_led_blink(3U);
        return false;
    }
    app_beep_short();
    app_led_blink(1U);

    app_delay_ms(120U);

    if (!align_right_before_da()) {
        app_led_blink(4U);
        return false;
    }

    if (!follow_arc_to_point()) {
        app_led_blink(3U);
        return false;
    }
    app_beep_short();
    app_led_blink(1U);
    return true;
}

bool task3_run_one_lap(void)
{
    return task3_run_one_lap_with_turns(TASK3_A_TO_AC_TURN_RAW,
        TASK3_B_TO_BD_TURN_RAW);
}

void task3_run(void)
{
    if (!imu_init_gyro_z()) {
        app_led_blink(2U);
        return;
    }

    (void)task3_run_one_lap();
}
