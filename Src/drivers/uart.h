#ifndef UART_H
#define UART_H
/*Uart driver to print messages to the terminal to aid in debugging
 * Makes use of printf sub module under external/prinf folder*/
void uart_init(void);
void uart_putchar_polling(char c);
void _putchar(char c);

// to be called only by assert handler
void uart_init_assert(void);
void uart_trace_assert(const char *string);

#endif // UART_H
