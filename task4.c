#include "task4.h"

#include "app_io.h"
#include "imu.h"
#include "task3.h"

#define TASK4_TEST_LAPS 4U
#define TASK4_FIRST_A_TURN_RAW 1310000
#define TASK4_REPEAT_A_TURN_RAW 1850000
#define TASK4_B_TO_BD_TURN_RAW 1610000

void task4_run(void)
{
    if (!imu_init_gyro_z()) {
        app_led_blink(2U);
        return;
    }

    for (uint8_t lap = 0U; lap < TASK4_TEST_LAPS; lap++) {
        int32_t aTurn = (lap == 0U) ? TASK4_FIRST_A_TURN_RAW :
            TASK4_REPEAT_A_TURN_RAW;

        if (!task3_run_one_lap_with_turns(aTurn,
                TASK4_B_TO_BD_TURN_RAW)) {
            return;
        }
        app_delay_ms(180U);
    }
}
