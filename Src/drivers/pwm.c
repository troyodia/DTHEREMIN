#include "pwm.h"
#include "io.h"
#include "assert_handler.h"
#include "stm32l476xx.h"
#include <stdbool.h>
// To get the desired sample rate of the timer 100khz
// using the formula fclk/((PSC+1) *(ARR+1)), good to make PSC as close to 0 as possible
// with fclk 80Mhz, PSC as 0, ARR is approximatly 800
#define PRESCALER (0)
#define AUTO_RELOAD (800U)

static bool intialized = false;
static const struct io_config pwm_config = { .mode = IO_MODE_ALTFN,
                                             .pupd = IO_NO_PUPD,
                                             .speed = IO_SPEED_VERY_HIGH,
                                             .type = IO_TYPE_PP,
                                             .af = IO_AF_2 };

/* DMA triggered by TIM1 events in dds.c writes the duty cycle into TIM3->CCR1*/
void pwm_init(void)
{
    ASSERT(!intialized);
    struct io_config current_config;
    io_get_io_config(AUDIO_PWM_OUT, &current_config);
    ASSERT(io_compare_io_config(&current_config, &pwm_config));
    RCC->APB1ENR1 |= 0x1 << 1; // enable TIM3 clock

    TIM3->ARR = AUTO_RELOAD - 1;
    TIM3->PSC = PRESCALER;

    TIM3->CCMR1 &= ~0x3; // configure capture/compare channel 1 of TIM3 as output
    TIM3->CCMR1 |= 0x1 << 3; // enable preload register for output compare
    // enable PWM mode 1
    TIM3->CCMR1 &= ~(0x7 << 4);
    TIM3->CCMR1 |= 0x6 << 4;
    // clear the capture compare register 1
    TIM3->CCR1 = 0;

    TIM3->CR1 |= 0x1 << 7; // enable auto-reload preload
    TIM3->EGR |= 0x1; // enable update generation
    TIM3->CCER |= 0x1; // enable capture/compare output
    TIM3->CR1 |= 0x1; // enable TIM3

    intialized = true;
}

float get_duty_cycle(uint32_t *ccr_value)
{
    volatile uint32_t ccr = TIM3->CCR1;
    *ccr_value = ccr;
    volatile float frac = ((float)ccr / AUTO_RELOAD);
    float duty_cycle = (frac * 100);
    return duty_cycle;
}
