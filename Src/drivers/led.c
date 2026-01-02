#include "led.h"
#include "assert_handler.h"
#include "defines.h"
#include "io.h"
#include "stdbool.h"
static bool initialized = false;
static const struct io_config led_config = { .mode = IO_MODE_OUPUT,
                                             .pupd = IO_NO_PUPD,
                                             .speed = IO_SPEED_VERY_HIGH,
                                             .type = IO_TYPE_PP,
                                             .af = IO_AF_NONE };
void led_init(void)
{
    /* First assert that led_init has not been called yet
     * This is to prevent multiple calls to the function*/
    ASSERT(!initialized);
    struct io_config current_config;

    io_get_io_config(IO_TEST_LED, &current_config);

    ASSERT(io_compare_io_config(&current_config, &led_config));
    initialized = true;
};
void led_set(led_e led, led_state_e state)
{
    ASSERT(initialized);
    io_out_e out = state == LED_STATE_ON ? IO_OUT_HIGH : IO_OUT_LOW;
    switch (led) {
    case LED_TEST:
        io_set_output(IO_TEST_LED, out);
        break;
    }
};