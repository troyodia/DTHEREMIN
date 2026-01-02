#ifndef PWM_H
#define PWM_H
// PWM driver for theremin sound generation
#include <stdint.h>

void pwm_init(void);
float get_duty_cycle(uint32_t *ccr_value);
#endif // PWM_H