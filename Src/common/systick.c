#include "systick.h"
#include "stm32l476xx.h"
#define CYCLES_PER_MS (80000U)

static uint32_t systick_ms = 0;
void systick_init(void)
{
    // Reload value number of clock clycles per millsec -> L = N/Sysclk
    // want 1ms so N is the sysclk freq
    // it starts from 0 so do N - 1
    SysTick->LOAD = CYCLES_PER_MS - 1;
    // clear the current vlu register
    SysTick->VAL = 0;
    // set the clock source to the system clock
    // set the systick interrupt that is called when a count down is completed
    // enable systick timer
    SysTick->CTRL = (0x1 << 2) | (0x1 << 1) | (0x1 << 0);
}

uint32_t systick_millis(void)
{
    return systick_ms;
}

void SysTick_Handler(void)
{
    systick_ms++;
}