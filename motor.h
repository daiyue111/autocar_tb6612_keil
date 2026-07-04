#ifndef MOTOR_H
#define MOTOR_H

#include "ti_msp_dl_config.h"

void motor_safe_stop(void);
void motor_forward_dir(void);
void motor_pwm_off(void);
void motor_pwm_run_1ms(uint8_t leftDuty, uint8_t rightDuty);
void motor_active_brake_then_stop(void);
void motor_short_brake_ms(uint32_t ms);
void motor_forward_test(void);
void motor_forward_pwm_1ms(uint8_t leftDuty, uint8_t rightDuty);
void motor_spin_right_pwm_1ms(uint8_t duty);
void motor_spin_left_pwm_1ms(uint8_t duty);

#endif
