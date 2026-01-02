#ifndef ASSERT_HANDLER_H
#include <stdint.h>
/* Assert on microcontroller to turn off motors during a fault*/
#define ASSERT(expression)                                                                         \
    do {                                                                                           \
        if (!(expression)) {                                                                       \
            void *pc;                                                                              \
            asm("mov %0, pc" : "=r"(pc));                                                          \
            assert_handler(pc);                                                                    \
        }                                                                                          \
    } while (0)
#define ASSERT_INTERRUPT(expression)                                                               \
    do {                                                                                           \
        if (!(expression)) {                                                                       \
            while (1)                                                                              \
                ;                                                                                  \
        }                                                                                          \
    } while (0)

void assert_handler(void *program_counter);

#endif // ASSERT_HANDLER_H