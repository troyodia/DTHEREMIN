#include "uart.h"
#include "io.h"
#include "defines.h"
#include "ring_buffer.h"
#include "assert_handler.h"
#include "stm32l4xx.h"
#include "stdbool.h"
#include <assert.h>
#include <string.h>
#define UART_RING_BUFFER_SIZE (16)
static uint8_t buffer[UART_RING_BUFFER_SIZE];
static struct ring_buffer tx_buffer = {
    .buffer = buffer, .head = 0, .tail = 0, .size = (sizeof(buffer) / sizeof(buffer[0]))
};
/* Caluculate the USARTDIV which is used to configure the baudrate for the USART peripheral
 * USARTDIV = SystemCoreClock / baudrate
 * The USART peripheral here is configured to run in normal mode (16 smapling, 8 smapling is
diasabled)*/
#define BAUDRATE (115200U)
#define SYSCLK (80000000U)
#define FCK (SYSCLK)
#define USARTDIV (FCK / BAUDRATE)

static_assert(sizeof(USARTDIV) < 0xFFFFFFFFu, "USARTDIV must fit into 32s-bits");
static_assert(USARTDIV >= 16U, "USARTDIV for the BAUDRATE must be atleast 16 decimal");

static bool initialized = false;
static void init_uart_clock(void)
{
    // set clock for USART3 in APB1
    RCC->APB1ENR1 |= (0x1 << 18);
}
static inline void uart_tx_enable_interrupt(void)
{
    // TXEIE (transmit interrupt enable)
    USART3->CR1 |= 0x1 << 7;
}
static void uart_configure(void)
{
    USART3->CR1 &= ~(0x1); // disable UE
    USART3->CR1 &= ~((0x1 << 12) | (0x1 << 28)); // set the word length as 8 bits

    USART3->BRR = USARTDIV; // set the baudrate as 1152000

    USART3->CR2 &= ~(0x3 << 12); // set number of stop bits as 1

    /* set TE (transmit enable), RE (recive enable) bits first
     set the UE (USART enable) bit after all configurations have been done*/
    USART3->CR1 |= ((0x1 << 3) | (0x1 << 2) | 0x1);
}
void uart_init(void)
{
    ASSERT(!initialized);
    init_uart_clock();
    uart_configure();

    NVIC_EnableIRQ(USART3_IRQn);

    initialized = true;
}
static void uart_tx_start(void)
{
    if (!ring_buffer_empty(&tx_buffer)) {
        uart_tx_enable_interrupt();
    }
}
void uart_init_assert(void)
{
    NVIC_DisableIRQ(USART3_IRQn);
    init_uart_clock();
    uart_configure();
    NVIC_EnableIRQ(USART3_IRQn);
}
void uart_putchar_polling(char c)
{
    /* wait for TXE to be empty
       TXE is the transmit data empty register
       it is set when data is transfered to the shft register */
    while (!(USART3->ISR & (0x1 << 7)))
        ;

    /* some terminals need to see the carriage-return \r character after the line feed \n to
     start a new line */
    USART3->TDR = c;
    if (c == '\n') {
        while (!(USART3->ISR & (0x1 << 7)))
            ;
        uart_putchar_polling('\r');
    }
}

void uart_trace_assert(const char *string)
{
    int i = 0;
    while (string[i] != '\0') {
        uart_putchar_polling(string[i]);
        i++;
    }
}
void USART3_IRQHandler(void)
{ /* check for if TXE and TXEIE are ready (both 1)
   * always check the flage and its interrupt to ensure the interrupt cleanly ends
   * without TXEIE checked, even if TXEIE is disabled another interrupt could trigger the ISR
   * since TXE is 1 after TDR is emptied the if guard is awlays true during so the ISR hangs in the
   * 2nd if guard
   * so checking the diabled TXEIE prevents the ISR from happening even if it is called */
    if (USART3->ISR & (0x1 << 7) && (USART3->CR1 & (0x1 << 7))) {

        ASSERT_INTERRUPT(!ring_buffer_empty(&tx_buffer));
        // Add character to TDR
        USART3->TDR = ring_buffer_peek(&tx_buffer);

        // remove the transmitted data from the TX buffer
        ring_buffer_get(&tx_buffer);

        // clear the TXEIE interrupt
        USART3->CR1 &= ~(0x1 << 7);

        // send all the data in the ring buffer if it is filled faster than it is read
        if (!ring_buffer_empty(&tx_buffer)) {
            uart_tx_start();
        }
    }

    /* check for if TC and TCIE are ready*/
    if (USART3->ISR & (0x1 << 6) && (USART3->CR1 & (0x1 << 6))) {
        // clear the TCIE interrupt
        USART3->CR1 &= ~(0x1 << 6);
    }
}
//_putchar is the function name required by the printf implementation
// renamed the uart driver interrupt function to _putchar
void _putchar(char c)
{
    // wait till ring buffer empties if full
    while (ring_buffer_full(&tx_buffer))
        ;
    NVIC_DisableIRQ(USART3_IRQn);
    bool tx_ongoing = !ring_buffer_empty(&tx_buffer);
    ring_buffer_put(&tx_buffer, c);
    if (!tx_ongoing) {
        uart_tx_start();
    }
    NVIC_EnableIRQ(USART3_IRQn);
    if (c == '\n') {
        _putchar('\r');
    }
}
