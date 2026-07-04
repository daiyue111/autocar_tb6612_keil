#include "track.h"
#include "app_io.h"

#define TRACK_BLACK_IS_HIGH 0U

uint8_t track_read_mask(void)
{
    uint8_t mask = 0U;
    bool x1 = app_gpio_read(TRACK_X1_PORT, TRACK_X1_PIN);
    bool x2 = app_gpio_read(TRACK_X2_PORT, TRACK_X2_PIN);
    bool x3 = app_gpio_read(TRACK_X3_PORT, TRACK_X3_PIN);
    bool x4 = app_gpio_read(TRACK_X4_PORT, TRACK_X4_PIN);
    bool x5 = app_gpio_read(TRACK_X5_PORT, TRACK_X5_PIN);
    bool x6 = app_gpio_read(TRACK_X6_PORT, TRACK_X6_PIN);
    bool x7 = app_gpio_read(TRACK_X7_PORT, TRACK_X7_PIN);
    bool x8 = app_gpio_read(TRACK_X8_PORT, TRACK_X8_PIN);

#if TRACK_BLACK_IS_HIGH
    if (x1) { mask |= 0x01U; }
    if (x2) { mask |= 0x02U; }
    if (x3) { mask |= 0x04U; }
    if (x4) { mask |= 0x08U; }
    if (x5) { mask |= 0x10U; }
    if (x6) { mask |= 0x20U; }
    if (x7) { mask |= 0x40U; }
    if (x8) { mask |= 0x80U; }
#else
    if (!x1) { mask |= 0x01U; }
    if (!x2) { mask |= 0x02U; }
    if (!x3) { mask |= 0x04U; }
    if (!x4) { mask |= 0x08U; }
    if (!x5) { mask |= 0x10U; }
    if (!x6) { mask |= 0x20U; }
    if (!x7) { mask |= 0x40U; }
    if (!x8) { mask |= 0x80U; }
#endif

    return mask;
}

bool track_has_line(void)
{
    return track_read_mask() != 0U;
}

uint8_t track_black_count(uint8_t mask)
{
    uint8_t count = 0U;

    for (uint8_t i = 0U; i < 8U; i++) {
        if ((mask & (uint8_t)(1U << i)) != 0U) {
            count++;
        }
    }

    return count;
}

int16_t track_error(uint8_t mask)
{
    static const int16_t weights[8] = {-35, -25, -15, -5, 5, 15, 25, 35};
    int16_t sum = 0;
    uint8_t count = 0U;

    for (uint8_t i = 0U; i < 8U; i++) {
        if ((mask & (uint8_t)(1U << i)) != 0U) {
            sum += weights[i];
            count++;
        }
    }

    if (count == 0U) {
        return 0;
    }
    return sum / (int16_t)count;
}
