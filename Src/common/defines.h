#ifndef DEFINES_H
#define DEFINES_H
#define UNUSED(x) (void)(x)
#define SUPPRESS_UNUSED __attribute__((unused))
#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))
// for x%n == x&(n-1) for n=2^k, so mod 2 == x&1
#define MOD_2(x) (x & 1)
#define IS_PULSE_ODD(pulse) MOD_2(pulse)
#define ABS(x) (x < 0 ? (-1 * x) : (x))
// STM32L3 runs at default 4MHz
// switched to 80Mhz using HSI and PLL clocks
#define CLOCK_FREQ_1MHZ (1000000U)
#define CLOCK_FREQ_80MHZ (80U * CLOCK_FREQ_1MHZ)
#define CLOCK_FREQ_80MHZ_ms (CLOCK_FREQ_80MHZ / 1000U)
#define BUSY_WAIT_ms(delay_ms)                                                                     \
    for (j = (delay_ms * CLOCK_FREQ_80MHZ_ms); j > 0; j--) {                                       \
    }; // takes the count for how long you want to delay
#endif

#define TIMER_PRESCALER (40u)
