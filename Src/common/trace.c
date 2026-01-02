#include "trace.h"
#include "uart.h"
#include "assert_handler.h"
#include "../../external/printf/printf.h"
#include <stdbool.h>
#include <stdarg.h>

static bool initialized = false;

void trace_init(void)
{
    ASSERT(!initialized);
    uart_init();
    initialized = true;
}
void trace(const char *format, ...)
{
    ASSERT(initialized);
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}