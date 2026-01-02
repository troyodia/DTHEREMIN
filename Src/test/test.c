#include "../drivers/io.h"
#include "../drivers/mcu_init.h"
#include "../common/assert_handler.h"
#include "../common/trace.h"
#include "../drivers/led.h"
#include "../common/defines.h"
#include "../common/systick.h"
#include "../drivers/pwm.h"
#include "../drivers/adc.h"
#include "../drivers/dds.h"
#include "../drivers/gp2y0a41sk0f.h"
#include "../drivers/lp_filter.h"
#include "stm32l476xx.h"
static const io_e io_pins[] = { IO_IR_SENSOR_1, IO_IR_SENSOR_2, IO_UART_TX, IO_UART_RX,

                                AUDIO_PWM_OUT,  IO_TEST_LED };
static void test_setup(void)
{
    mcu_init();
}
SUPPRESS_UNUSED
static void test_assert(void)
{
    test_setup();
    ASSERT(0);
}

SUPPRESS_UNUSED
static void test_blink_led(void)
{
    test_setup();
    led_init();
    volatile int j;
    led_state_e led_state = LED_STATE_OFF;
    while (1) {
        led_state = (led_state == LED_STATE_OFF) ? LED_STATE_ON : LED_STATE_OFF;
        led_set(LED_TEST, led_state);
        BUSY_WAIT_ms(80)
    }
}

SUPPRESS_UNUSED
static void test_trace(void)
{
    test_setup();
    trace_init();
    volatile int j;
    while (1) {
        TRACE("TEST TRACE %d", 1);
        BUSY_WAIT_ms(60);
    }
}

SUPPRESS_UNUSED
static void io_PA_8_isr(void)
{
    led_set(LED_TEST, LED_STATE_ON);
}

SUPPRESS_UNUSED
static void io_PB_3_isr(void)
{
    led_set(LED_TEST, LED_STATE_OFF);
}

SUPPRESS_UNUSED
static void test_io_interrupt(void)
{
    test_setup();
    const struct io_config input_config = { .mode = IO_MODE_INPUT,
                                            .pupd = IO_PORT_PU,
                                            .speed = IO_SPEED_LOW,
                                            .type = IO_TYPE_PP,
                                            .af = IO_AF_NONE };
    io_configure((io_e)IO_PA_8, &input_config);
    io_configure((io_e)IO_PB_3, &input_config);
    led_init();

    io_interrupt_config((io_e)IO_PB_3, io_PB_3_isr, IO_FALLING_TRIGGER, IO_EXTI_N_3);
    io_enable_interrupt(IO_EXTI_3_LINE);
    io_interrupt_config((io_e)IO_PA_8, io_PA_8_isr, IO_FALLING_TRIGGER, IO_EXTI_N_0);
    io_enable_interrupt(IO_EXTI_9_5_LINE);

    while (1)

        ;
}

SUPPRESS_UNUSED
static void test_duty_cycle(void)
{
    test_setup();
    trace_init();
    pwm_init();
    dds_init();
    volatile int j;
    uint32_t ccr_value;
    while (1) {
        TRACE("DUTY CYCLE: %.2f, CCR1: %d\n", get_duty_cycle(&ccr_value), ccr_value);
        BUSY_WAIT_ms(10)
    }
}

SUPPRESS_UNUSED
static void test_adc(void)
{
    test_setup();
    trace_init();
    adc_init();
    volatile int j;
    float pitch_voltage = 0.0f;
    float pitch_distance = 0.0f;

    adc_channel_values adc_channel_data = { 0 };

    while (1) {
        adc_read_channel_values(adc_channel_data);

        gp2y0a41sk0f_adc_to_voltage_test(&pitch_voltage, &pitch_distance, adc_channel_data[1]);

        TRACE("IR P_VOLTAGE |IR P_DISTANCE %d, %.2f\n, %.2f\n", adc_channel_data[1], pitch_voltage,
              pitch_distance);
        BUSY_WAIT_ms(200);
    };
}

SUPPRESS_UNUSED
static void test_system(void)
{
    test_setup();
    systick_init();
    trace_init();
    gp2y0a41sk0f_init();
    pwm_init();
    dds_init();
    volatile int j;
    uint32_t ccr_value;

    while (1) {
        TRACE("DUTY CYCLE: %.2f, CCR1: %d\n", get_duty_cycle(&ccr_value), ccr_value);
        BUSY_WAIT_ms(80)
    };
}
/* Main function
 * The code is complied and  flashed using the MAKEFILE in the directory tree
 * The command  "GCC_PATH=C:/gcc-arm-none-eabi/bin TEST=test_system make" is used to run the make
 * file for with the main function pointing to test_system*/
int main()
{
    TEST();
    ASSERT(0);
}