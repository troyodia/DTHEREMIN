#include "io.h"
#include "../common/defines.h"
#include "../common/assert_handler.h"
#include "adc.h"
#include "stm32l4xx.h"
#include <assert.h>
#include <stddef.h>

/*
- Use the bit pattern of the port enums assigned to the
io_e pins to get their port and pin numbers
- The pin number is represented by the last 4 bits of the bits (15 is the largest pin no)
- The port number is represented by the 4 bits directly before the 4 pin bits
- Enums are represented as 8 bytes using compiler flag -fshort-enums
*/
#define IO_PORT_OFFSET (4U)
#define IO_PORT_MASK (0xFu << IO_PORT_OFFSET)
#define IO_PIN_MASK (0xFu)
// To get rid of error squiggles since vscode intellisense thinks enums are 4bytes
#if defined(__INTELLISENSE__)
_Static_assert(sizeof(io_ports_e) == 4, "IntelliSense only: workaround");
#else
static_assert(sizeof(io_ports_e) == 1,
              "Unexpected enum size"
              "-fshort-enums missing?");
#endif
#define IO_PORT_CNT (8U)
#define IO_PORT_CNT_ISR (3U)
#define IO_PIN_PER_PORT_CNT (16U)
#define IO_PIN_CNT (48U)
#define IO_EXTI_INTERRUPT_LINES_CNT (7U)
/* Array holds the initial config for each pin
 * Configure unused pins as input with pull down resistors
 * Configure the pin to prevent unpredictable noise for floating pins   */
#define UNUSED_PIN_CONFIG                                                                          \
    {                                                                                              \
        IO_MODE_INPUT, IO_PORT_PD, IO_SPEED_LOW, IO_TYPE_PP, IO_AF_NONE                            \
    }
#define ADC_PIN_CONFIG                                                                             \
    {                                                                                              \
        IO_MODE_ANALOG, IO_NO_PUPD, IO_SPEED_LOW, IO_TYPE_PP, IO_AF_NONE                           \
    }

static const struct io_config io_pins_initial_configs[IO_PIN_CNT] = {
    // line detectors set up as analog input
    [IO_IR_SENSOR_1] = ADC_PIN_CONFIG,
    [IO_IR_SENSOR_2] = ADC_PIN_CONFIG,

    // Buttons to change state of theremin
    [IO_BTN_MAP_MODE] = { .mode = IO_MODE_INPUT,
                          .pupd = IO_PORT_PU,
                          .speed = IO_SPEED_LOW,
                          .type = IO_TYPE_PP,
                          .af = IO_AF_NONE },
    [IO_BTN_OCTAVE_UP] = { .mode = IO_MODE_INPUT,
                           .pupd = IO_PORT_PU,
                           .speed = IO_SPEED_LOW,
                           .type = IO_TYPE_PP,
                           .af = IO_AF_NONE },
    [IO_BTN_OCTAVE_DOWN] = { .mode = IO_MODE_INPUT,
                             .pupd = IO_PORT_PU,
                             .speed = IO_SPEED_LOW,
                             .type = IO_TYPE_PP,
                             .af = IO_AF_NONE },
    [IO_BTN_MODULATION] = { .mode = IO_MODE_INPUT,
                            .pupd = IO_PORT_PU,
                            .speed = IO_SPEED_LOW,
                            .type = IO_TYPE_PP,
                            .af = IO_AF_NONE },
    // test led setup as an output, held low initiailly
    [IO_TEST_LED] = { .mode = IO_MODE_OUPUT,
                      .pupd = IO_NO_PUPD,
                      .speed = IO_SPEED_VERY_HIGH,
                      .type = IO_TYPE_PP,
                      .af = IO_AF_NONE },

    /* uart transmit
     * mode: alternate function
     * pupd: no pupd (output not required)*/
    [IO_UART_TX] = { .mode = IO_MODE_ALTFN,
                     .pupd = IO_NO_PUPD,
                     .speed = IO_SPEED_HIGH,
                     .type = IO_TYPE_PP,
                     .af = IO_AF_7 },

    /* uart recieve
     * mode: alternate function
     * pupd: pulled down*/
    [IO_UART_RX] = { .mode = IO_MODE_ALTFN,
                     .pupd = IO_PORT_PD,
                     .speed = IO_SPEED_LOW,
                     .type = IO_TYPE_PP,
                     .af = IO_AF_7 },

    // motor pwm pins set up as alternate function
    [AUDIO_PWM_OUT] = { .mode = IO_MODE_ALTFN,
                        .pupd = IO_NO_PUPD,
                        .speed = IO_SPEED_VERY_HIGH,
                        .type = IO_TYPE_PP,
                        .af = IO_AF_2 },
    [IO_USER_BTN] = { .mode = IO_MODE_INPUT,
                      .pupd = IO_PORT_PU,
                      .speed = IO_SPEED_LOW,
                      .type = IO_TYPE_PP,
                      .af = IO_AF_NONE },
    [IO_UNUSED_1] = { .mode = IO_MODE_INPUT,
                      .pupd = IO_PORT_PU,
                      .speed = IO_SPEED_LOW,
                      .type = IO_TYPE_PP,
                      .af = IO_AF_NONE },

    [IO_UNUSED_2] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_3] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_4] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_5] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_6] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_7] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_8] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_9] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_10] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_11] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_12] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_13] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_14] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_15] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_16] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_17] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_18] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_19] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_20] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_21] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_22] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_23] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_24] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_25] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_26] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_27] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_28] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_29] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_30] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_31] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_32] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_33] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_34] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_35] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_36] = UNUSED_PIN_CONFIG,
    [IO_UNUSED_37] = UNUSED_PIN_CONFIG,
};
static io_e io_adc_pins[] = { IO_IR_SENSOR_1, IO_IR_SENSOR_2 };

void io_init(void)
{
    io_e io_pin;
    const struct io_config io_unused_config = { IO_MODE_INPUT, IO_PORT_PD, IO_SPEED_LOW, IO_TYPE_PP,
                                                IO_AF_NONE };

    for (io_pin = (io_e)IO_PA_0; io_pin < ARRAY_SIZE(io_pins_initial_configs); io_pin++) {
        if (io_pin == IO_UNUSED_12 || io_pin == IO_UNUSED_13) {
            continue;
        }
        io_configure(io_pin, &io_pins_initial_configs[io_pin]);
    }

    io_configure(IO_UNUSED_40, &io_unused_config);
}
static uint8_t io_get_port(io_e io)
{
    return (io & IO_PORT_MASK) >> IO_PORT_OFFSET;
}
static uint8_t io_get_pin_idx(io_e io)
{
    return (io & IO_PIN_MASK);
}
typedef enum
{
    PORTA,
    PORTB,
    PORTC,
} io_port_e;
/*- Array of GPIO ports, the ports are defined as struct pointers in the header files
  - Used an array of the ports to allow for cleaner code and no switch statments
  - Added only 3 ports since only A,B and C are being used in the sumo bot*/
static GPIO_TypeDef *const gpio_port_regs[IO_PORT_CNT] = { GPIOA, GPIOB, GPIOC, GPIOD,
                                                           GPIOE, GPIOF, GPIOG, GPIOH };

static isr_function isr_functions[IO_PORT_CNT_ISR][IO_PIN_PER_PORT_CNT] = {
    [PORTA] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                NULL, NULL },
    [PORTB] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                NULL, NULL },
    [PORTC] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                NULL, NULL },
};
static const IRQn_Type exti_interrupt_lines[IO_EXTI_INTERRUPT_LINES_CNT] = {
    [IO_EXTI_0_LINE] = EXTI0_IRQn,         [IO_EXTI_1_LINE] = EXTI1_IRQn,
    [IO_EXTI_2_LINE] = EXTI2_IRQn,         [IO_EXTI_3_LINE] = EXTI3_IRQn,
    [IO_EXTI_4_LINE] = EXTI4_IRQn,         [IO_EXTI_9_5_LINE] = EXTI9_5_IRQn,
    [IO_EXTI_15_10_LINE] = EXTI15_10_IRQn,
};
void io_configure(io_e io, const struct io_config *config)
{
    io_port_clock_init(io);
    io_set_mode(io, config->mode);
    io_set_output_type(io, config->type);
    io_set_pupd(io, config->pupd); // no pupd for output
    switch (config->mode) {
    case IO_MODE_INPUT:
        break;

    case IO_MODE_OUPUT:
        io_set_output_speed(io, config->speed);
        break;

    case IO_MODE_ALTFN:
        io_set_output_speed(io, config->speed);
        io_set_AF(io, config->af);
        break;

    case IO_MODE_ANALOG:
        io_set_analog_switch_crl_reg(io);
        break;
    }
}
void io_port_clock_init(io_e io)
{
    const uint8_t port = io_get_port(io);

    RCC->AHB2ENR |= (0x1 << port);
}

void io_set_mode(io_e io, io_mode_e mode)
{

    const uint8_t pin = io_get_pin_idx(io);
    const uint8_t port = io_get_port(io);

    GPIO_TypeDef *GPIO = gpio_port_regs[port];

    GPIO->MODER &= ~(0x3 << (2 * pin));
    GPIO->MODER |= (mode << (2 * pin));
};

void io_set_output_type(io_e io, io_out_type_e type)
{
    const uint8_t pin = io_get_pin_idx(io);
    const uint8_t port = io_get_port(io);

    GPIO_TypeDef *GPIO = gpio_port_regs[port];
    switch (type) {
    case IO_TYPE_PP:
        GPIO->OTYPER &= ~(0x1 << pin);
        break;
    case IO_TYPE_OD:
        GPIO->OTYPER |= (0x1 << pin);
        break;
    }
}
void io_set_pupd(io_e io, io_pupd_e pupd)
{
    const uint8_t pin = io_get_pin_idx(io);
    const uint8_t port = io_get_port(io);

    GPIO_TypeDef *GPIO = gpio_port_regs[port];
    GPIO->PUPDR &= ~(0x3 << (2 * pin));
    GPIO->PUPDR |= (pupd << (2 * pin));
}
void io_set_output_speed(io_e io, io_ouput_speed_e speed)
{
    const uint8_t pin = io_get_pin_idx(io);
    const uint8_t port = io_get_port(io);

    GPIO_TypeDef *GPIO = gpio_port_regs[port];
    GPIO->OSPEEDR &= ~(0x3 << (2 * pin));
    GPIO->OSPEEDR |= (speed << (2 * pin));
}
void io_set_output(io_e io, io_out_e out)
{
    const uint8_t pin = io_get_pin_idx(io);
    const uint8_t port = io_get_port(io);

    GPIO_TypeDef *GPIO = gpio_port_regs[port];

    switch (out) {
    case IO_OUT_HIGH:
        GPIO->ODR |= (0x1 << pin);
        break;
    case IO_OUT_LOW:
        GPIO->ODR &= ~(0x1 << pin);
        break;
    }
}

io_in_e io_get_input(io_e io)
{
    const uint8_t pin = io_get_pin_idx(io);
    const uint8_t port = io_get_port(io);

    const GPIO_TypeDef *GPIO = gpio_port_regs[port];
    const uint8_t input = GPIO->IDR & (0x1 << pin);
    return input ? IO_IN_HIGH : IO_IN_LOW;
}
void io_set_AF(io_e io, io_af_e af)
{
    const uint8_t pin = io_get_pin_idx(io);
    const uint8_t port = io_get_port(io);
    uint8_t pin_idx = pin;
    GPIO_TypeDef *GPIO = gpio_port_regs[port];
    uint8_t af_section = pin / 8;
    /* To account for the AFRH of the AF register, counts up from bit 8
     * Could use modulu but need to conserve memory*/
    if (pin > 7) {
        pin_idx = pin - 8;
    }
    GPIO->AFR[af_section] &= ~(0xF << (pin_idx * 4));
    GPIO->AFR[af_section] |= (af << (pin_idx * 4));
}
void io_set_analog_switch_crl_reg(io_e io)
{
    const uint8_t pin = io_get_pin_idx(io);
    const uint8_t port = io_get_port(io);
    GPIO_TypeDef *GPIO = gpio_port_regs[port];
    GPIO->ASCR |= (0x1 << pin);
}
io_e *get_io_adc_pins(uint8_t *size)
{
    *size = ARRAY_SIZE(io_adc_pins);
    return io_adc_pins;
}
uint8_t io_adc_idx(io_ports_e io)
{
    uint8_t idx;
    switch (io) {
    case IO_PC_0:
        idx = ADC2_IN1;
        break;
    case IO_PC_1:
        idx = ADC2_IN2;
        break;
    case IO_PC_2:
        idx = ADC2_IN3;
        break;
    case IO_PC_3:
        idx = ADC2_IN4;
        break;
    case IO_PA_0:
        idx = ADC2_IN5;
        break;
    case IO_PA_1:
        idx = ADC2_IN6;
        break;
    case IO_PA_2:
        idx = ADC2_IN7;
        break;
    case IO_PA_3:
        idx = ADC2_IN8;
        break;
    case IO_PA_4:
        idx = ADC2_IN9;
        break;
    case IO_PA_5:
        idx = ADC2_IN10;
        break;
    case IO_PA_6:
        idx = ADC2_IN11;
        break;
    case IO_PA_7:
        idx = ADC2_IN12;
        break;
    case IO_PC_4:
        idx = ADC2_IN13;
        break;
    case IO_PC_5:
        idx = ADC2_IN14;
        break;
    case IO_PB_0:
        idx = ADC2_IN15;
        break;
    case IO_PB_1:
        idx = ADC2_IN16;
        break;
    default:
        idx = 0;
        break;
    }
    return idx;
}
void io_get_io_config(io_e io, struct io_config *current_config)
{
    const uint8_t pin = io_get_pin_idx(io);
    const uint8_t port = io_get_port(io);

    const GPIO_TypeDef *GPIO = gpio_port_regs[port];
    current_config->mode = (io_mode_e)((GPIO->MODER & (0x3 << (2 * pin))) >> (2 * pin));
    current_config->pupd = (io_pupd_e)((GPIO->PUPDR & (0x3 << (2 * pin))) >> (2 * pin));
    current_config->speed = (io_ouput_speed_e)((GPIO->OSPEEDR & (0x3 << (2 * pin))) >> (2 * pin));
    current_config->type = (io_out_type_e)((GPIO->OTYPER & (0x1 << pin)) >> pin);
};
bool io_compare_io_config(const struct io_config *config1, const struct io_config *config2)
{
    if (config1->mode != config2->mode) {
        return false;
    }
    if (config1->pupd != config2->pupd) {
        return false;
    }
    if (config1->speed != config2->speed) {
        return false;
    }
    if (config1->type != config2->type) {
        return false;
    }
    return true;
};
void io_interrupt_clock_init(void)
{
    RCC->APB2ENR |= 0x1;
}
static void io_set_interrupt_trigger(io_e io, io_trigger_e trigger)
{
    const uint8_t pin = io_get_pin_idx(io);

    switch (trigger) {
    case IO_FALLING_TRIGGER:
        EXTI->FTSR1 |= (0x1 << pin);
        EXTI->RTSR1 &= ~(0x1 << pin);
        break;
    case IO_RISING_TRIGGER:
        EXTI->RTSR1 |= (0x1 << pin);
        EXTI->FTSR1 &= ~(0x1 << pin);
        break;
    }
}
static void io_register_isr(io_e io, isr_function isr)
{
    const uint8_t pin = io_get_pin_idx(io);
    const uint8_t port = io_get_port(io);
    ASSERT(isr_functions[port][pin] == NULL);
    isr_functions[port][pin] = isr;
}
void io_interrupt_config(io_e io, isr_function isr, io_trigger_e trigger,
                         io_exti_section_e exti_section)
{
    io_interrupt_clock_init();
    const uint8_t pin = io_get_pin_idx(io);
    const uint8_t port = io_get_port(io);
    int exti = pin / 4;
    SYSCFG->EXTICR[exti] &= ~(0xF << exti_section);
    SYSCFG->EXTICR[exti] |= (port << exti_section);

    EXTI->IMR1 |= (0x1 << pin);
    io_set_interrupt_trigger(io, trigger);
    io_register_isr(io, isr);
}

static inline void io_unregister_isr(io_e io)
{
    const uint8_t pin = io_get_pin_idx(io);
    const uint8_t port = io_get_port(io);
    isr_functions[port][pin] = NULL;
}
static void io_unset_interrupt_trigger(io_e io, io_trigger_e trigger)
{
    const uint8_t pin = io_get_pin_idx(io);

    switch (trigger) {
    case IO_FALLING_TRIGGER:
        EXTI->FTSR1 &= ~(0x1 << pin);
        break;
    case IO_RISING_TRIGGER:
        EXTI->RTSR1 &= ~(0x1 << pin);
        break;
    }
}
void io_deconfigure_interrupt(io_e io, io_trigger_e trigger, io_exti_section_e exti_section)
{
    const uint8_t pin = io_get_pin_idx(io);
    const uint8_t port = io_get_port(io);
    uint8_t exti = port / 4;
    SYSCFG->EXTICR[exti] &= ~(0xF << exti_section);

    EXTI->IMR1 &= ~(0x1 << pin);
    io_unset_interrupt_trigger(io, trigger);
    io_unregister_isr(io);
}
void io_enable_interrupt(io_exti_line_e line)
{
    NVIC_EnableIRQ(exti_interrupt_lines[line]);
}
void io_disable_interrupt(io_exti_line_e line)
{
    NVIC_DisableIRQ(exti_interrupt_lines[line]);
}
void io_set_priority_interrupt(io_exti_line_e line, uint32_t priority)
{
    NVIC_SetPriority(exti_interrupt_lines[line], priority);
}
static void io_isr(io_e io)
{
    const uint8_t pin = io_get_pin_idx(io);
    const uint8_t port = io_get_port(io);
    if (EXTI->PR1 & (0x1 << pin)) {
        EXTI->PR1 = (0x1 << pin); // clear the PR1 flag for the EXTI line
        if (isr_functions[port][pin] != NULL) {
            isr_functions[port][pin]();
        }
    }
}

void EXTI0_IRQHandler(void)
{
    io_isr((io_e)IO_PA_0);
}
void EXTI1_IRQHandler(void)
{
    io_isr((io_e)IO_PC_1);
    io_isr((io_e)IO_PB_1);
    io_isr((io_e)IO_PA_1);
}
void EXTI2_IRQHandler(void)
{
    io_isr((io_e)IO_PC_2);
    io_isr((io_e)IO_PB_2);
    io_isr((io_e)IO_PA_2);
}
void EXTI3_IRQHandler(void)
{
    io_isr((io_e)IO_PB_3);
}
void EXTI4_IRQHandler(void)
{
    io_isr((io_e)IO_PB_4);
}
void EXTI9_5_IRQHandler(void)
{
    io_isr((io_e)IO_PB_5);
}
void EXTI15_10_IRQHandler(void)
{
    io_ports_e io;
    for (io = IO_PA_10; io <= IO_PA_15; io++) {
        io_isr((io_e)io);
    }
    for (io = IO_PB_10; io <= IO_PB_15; io++) {
        io_isr((io_e)io);
    }
    for (io = IO_PC_10; io <= IO_PC_15; io++) {
        io_isr((io_e)io);
    }
}