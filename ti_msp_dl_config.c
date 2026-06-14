#include "ti_msp_dl_config.h"

#define GPIOA_OUTPUT_PINS \
    (MOTOR_BIN1_PIN | MOTOR_BIN2_PIN | MOTOR_CIN1_PIN | MOTOR_CIN2_PIN | \
     BEEP_PIN)
#define GPIOB_OUTPUT_PINS \
    (MOTOR_STBY_PIN | MOTOR_AIN1_PIN | MOTOR_AIN2_PIN | MOTOR_PWMA_PIN | \
     MOTOR_PWMB_PIN | MOTOR_PWMC_PIN | MOTOR_DIN1_PIN | MOTOR_DIN2_PIN | \
     MOTOR_PWMD_PIN | LED_PIN)

static void init_output_pin(IOMUX_PINCM pin)
{
    DL_GPIO_initDigitalOutput(pin);
}

static void init_input_pullup_pin(IOMUX_PINCM pin)
{
    DL_GPIO_initDigitalInputFeatures(pin, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_DOWN, DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

static void init_key_pullup_pin(IOMUX_PINCM pin)
{
    DL_GPIO_initDigitalInputFeatures(pin, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

void SYSCFG_DL_init(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    delay_cycles(16);

    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);

    init_output_pin(MOTOR_STBY_IOMUX);
    init_output_pin(MOTOR_AIN1_IOMUX);
    init_output_pin(MOTOR_AIN2_IOMUX);
    init_output_pin(MOTOR_PWMA_IOMUX);
    init_output_pin(MOTOR_BIN1_IOMUX);
    init_output_pin(MOTOR_BIN2_IOMUX);
    init_output_pin(MOTOR_PWMB_IOMUX);
    init_output_pin(MOTOR_CIN1_IOMUX);
    init_output_pin(MOTOR_CIN2_IOMUX);
    init_output_pin(MOTOR_PWMC_IOMUX);
    init_output_pin(MOTOR_DIN1_IOMUX);
    init_output_pin(MOTOR_DIN2_IOMUX);
    init_output_pin(MOTOR_PWMD_IOMUX);
    init_output_pin(LED_IOMUX);
    init_output_pin(BEEP_IOMUX);
    init_input_pullup_pin(TRACK_X1_IOMUX);
    init_input_pullup_pin(TRACK_X2_IOMUX);
    init_input_pullup_pin(TRACK_X3_IOMUX);
    init_input_pullup_pin(TRACK_X4_IOMUX);
    init_input_pullup_pin(TRACK_X5_IOMUX);
    init_input_pullup_pin(TRACK_X6_IOMUX);
    init_input_pullup_pin(TRACK_X7_IOMUX);
    init_input_pullup_pin(TRACK_X8_IOMUX);
    init_key_pullup_pin(KEY_START_IOMUX);

    DL_GPIO_clearPins(GPIOA, GPIOA_OUTPUT_PINS);
    DL_GPIO_clearPins(GPIOB, GPIOB_OUTPUT_PINS);
    DL_GPIO_enableOutput(GPIOA, GPIOA_OUTPUT_PINS);
    DL_GPIO_enableOutput(GPIOB, GPIOB_OUTPUT_PINS);

}
