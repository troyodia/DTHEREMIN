#include "assert_handler.h"
#include "defines.h"
#include "uart.h"
#include "stm32l4xx.h"
#include "../../external/printf/printf.h"
#define BREAKPOINT __asm volatile("bkpt #0");

// Text + Program counter + Mull termination \0
#define ASSERT_STRING_MAX_SIZE (15u + 6u + 1u)
#define GPIO_SET_LOW(GPIO, pin)                                                                    \
    {                                                                                              \
        do {                                                                                       \
            GPIO->MODER &= ~(0x3 << (2 * pin));                                                    \
            GPIO->MODER |= (0x1 << (2 * pin));                                                     \
            GPIO->OTYPER &= ~(0x1 << pin);                                                         \
            GPIO->OSPEEDR &= ~(0x3 << (2 * pin));                                                  \
            GPIO->OSPEEDR |= (0x3 << (2 * pin));                                                   \
            GPIO->ODR &= ~(1 << pin);                                                              \
        } while (0);                                                                               \
    }
static void assert_trace(void *program_counter)
{
    // configure UART TX pin
    RCC->AHB2ENR |= 0x1 << 2;
    GPIOC->MODER &= ~(0x3 << (2 * 10));
    GPIOC->MODER |= (0x2 << (2 * 10));
    GPIOC->OTYPER &= ~(0x1 << 10);
    GPIOC->OSPEEDR &= ~(0x2 << (2 * 10));
    GPIOC->OSPEEDR |= (0x2 << (2 * 10));
    GPIOC->AFR[1] &= ~(0xF << ((10 - 8) * 4));
    GPIOC->AFR[1] |= (0x7 << ((10 - 8) * 4));
    uart_init_assert();

    char assert_string[ASSERT_STRING_MAX_SIZE];
    // writes to assert_string buffer the fomratted string
    snprintf(assert_string, sizeof(assert_string), "ASSERT 0x%x\n", program_counter);
    uart_trace_assert(assert_string);
}
static void assert_blink_led(void)
{
    /* Accessing registers directly to config TEST LED to prevent
       calling functions that have assertions, this prevents recursive assertions*/
    RCC->AHB2ENR |= 0x1;
    GPIO_SET_LOW(GPIOA, 5);
    volatile int j;
    while (1) {
        GPIOA->ODR ^= (0x1 << 5);
        BUSY_WAIT_ms(90)
    };
}

void assert_handler(void *program_counter)
{
    BREAKPOINT
    assert_trace(program_counter);
    assert_blink_led();
}