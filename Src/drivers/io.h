#ifndef IO_H
#define IO_H
/*STM32 IO Peripheral driver*/
#include "stdbool.h"
#include <stdint.h>

// ENUMS
typedef enum
{
    IO_PA_0,
    IO_PA_1,
    IO_PA_2,
    IO_PA_3,
    IO_PA_4,
    IO_PA_5,
    IO_PA_6,
    IO_PA_7,
    IO_PA_8,
    IO_PA_9,
    IO_PA_10,
    IO_PA_11,
    IO_PA_12,
    IO_PA_13,
    IO_PA_14,
    IO_PA_15,
    IO_PB_0,
    IO_PB_1,
    IO_PB_2,
    IO_PB_3,
    IO_PB_4,
    IO_PB_5,
    IO_PB_6,
    IO_PB_7,
    IO_PB_8,
    IO_PB_9,
    IO_PB_10,
    IO_PB_11,
    IO_PB_12,
    IO_PB_13,
    IO_PB_14,
    IO_PB_15,
    IO_PC_0,
    IO_PC_1,
    IO_PC_2,
    IO_PC_3,
    IO_PC_4,
    IO_PC_5,
    IO_PC_6,
    IO_PC_7,
    IO_PC_8,
    IO_PC_9,
    IO_PC_10,
    IO_PC_11,
    IO_PC_12,
    IO_PC_13,
    IO_PC_14,
    IO_PC_15,
    IO_PD_2 = 50,
    IO_PH_0 = 112,
    IO_PH_1
} io_ports_e;

typedef enum
{
    // PA
    IO_BTN_MAP_MODE = IO_PA_0,
    IO_UNUSED_1 = IO_PA_1,
    IO_UNUSED_2 = IO_PA_2,
    IO_UNUSED_3 = IO_PA_3,
    IO_UNUSED_4 = IO_PA_4,
    IO_TEST_LED = IO_PA_5,
    IO_UNUSED_5 = IO_PA_6,
    IO_UNUSED_6 = IO_PA_7,
    IO_UNUSED_7 = IO_PA_8,
    IO_UNUSED_8 = IO_PA_9,
    IO_UNUSED_9 = IO_PA_10,
    IO_UNUSED_10 = IO_PA_11,
    IO_UNUSED_11 = IO_PA_12,
    IO_UNUSED_12 = IO_PA_13,
    IO_UNUSED_13 = IO_PA_14,
    IO_UNUSED_14 = IO_PA_15,
    // PB
    IO_UNUSED_15 = IO_PB_0,
    IO_UNUSED_16 = IO_PB_1,
    IO_UNUSED_17 = IO_PB_2,
    IO_BTN_OCTAVE_UP = IO_PB_3,
    IO_BTN_OCTAVE_DOWN = IO_PB_4,
    IO_BTN_MODULATION = IO_PB_5,
    IO_UNUSED_18 = IO_PB_6,
    IO_UNUSED_19 = IO_PB_7,
    IO_UNUSED_20 = IO_PB_8,
    IO_UNUSED_21 = IO_PB_9,
    IO_UNUSED_22 = IO_PB_10,
    IO_UNUSED_23 = IO_PB_11,
    IO_UNUSED_24 = IO_PB_12,
    IO_UNUSED_25 = IO_PB_13,
    IO_UNUSED_26 = IO_PB_14,
    IO_UNUSED_27 = IO_PB_15,

    // PC
    IO_IR_SENSOR_1 = IO_PC_0,
    IO_IR_SENSOR_2 = IO_PC_1,
    IO_UNUSED_28 = IO_PC_2,
    IO_UNUSED_29 = IO_PC_3,
    IO_UNUSED_30 = IO_PC_4,
    IO_UNUSED_31 = IO_PC_5,
    AUDIO_PWM_OUT = IO_PC_6,
    IO_UNUSED_32 = IO_PC_7,
    IO_UNUSED_33 = IO_PC_8,
    IO_UNUSED_34 = IO_PC_9,
    IO_UART_TX = IO_PC_10,
    IO_UART_RX = IO_PC_11,
    IO_UNUSED_35 = IO_PC_12,
    IO_USER_BTN = IO_PC_13,
    IO_UNUSED_36 = IO_PC_14,
    IO_UNUSED_37 = IO_PC_15,
    // PD
    IO_UNUSED_38 = IO_PD_2,
    // PH
    IO_UNUSED_39 = IO_PH_0,
    IO_UNUSED_40 = IO_PH_1,

} io_e;

typedef enum
{
    IO_MODE_INPUT,
    IO_MODE_OUPUT,
    IO_MODE_ALTFN,
    IO_MODE_ANALOG
} io_mode_e;

typedef enum
{
    IO_NO_PUPD, // no pull up or pull down
    IO_PORT_PU, // pull up
    IO_PORT_PD, // pull down
    IO_RESERVED // reserved
} io_pupd_e;
typedef enum
{
    IO_SPEED_LOW,
    IO_SPEED_MEDIUM,
    IO_SPEED_HIGH,
    IO_SPEED_VERY_HIGH
} io_ouput_speed_e;
typedef enum
{
    IO_IN_LOW,
    IO_IN_HIGH,
} io_in_e;
typedef enum
{
    IO_OUT_LOW,
    IO_OUT_HIGH,
} io_out_e;
typedef enum
{
    IO_TYPE_PP, // push pull
    IO_TYPE_OD // open drain
} io_out_type_e;
typedef enum
{
    IO_AF_0,
    IO_AF_1,
    IO_AF_2,
    IO_AF_3,
    IO_AF_4,
    IO_AF_5,
    IO_AF_6,
    IO_AF_7,
    IO_AF_8,
    IO_AF_9,
    IO_AF_10,
    IO_AF_11,
    IO_AF_12,
    IO_AF_13,
    IO_AF_14,
    IO_AF_15,
    IO_AF_NONE
} io_af_e;
typedef enum
{
    IO_FALLING_TRIGGER,
    IO_RISING_TRIGGER
} io_trigger_e;
/*Enum contains the 4 sections of the 16 bit EXTICRN register
 * N represents the particular EXTI regsiter (1-4)*/
typedef enum
{
    IO_EXTI_N_0,
    IO_EXTI_N_1 = 4,
    IO_EXTI_N_2 = 8,
    IO_EXTI_N_3 = 12,
} io_exti_section_e;
typedef enum
{
    IO_EXTI_0_LINE,
    IO_EXTI_1_LINE,
    IO_EXTI_2_LINE,
    IO_EXTI_3_LINE,
    IO_EXTI_4_LINE,
    IO_EXTI_9_5_LINE,
    IO_EXTI_15_10_LINE,
} io_exti_line_e;
// STRUCT
struct io_config
{
    io_mode_e mode;
    io_pupd_e pupd;
    io_ouput_speed_e speed;
    io_out_type_e type;
    io_af_e af;
};

// FUNCTIONS
void io_init(void);
void io_configure(io_e io, const struct io_config *config);
void io_get_io_config(io_e io, struct io_config *current_config);
bool io_compare_io_config(const struct io_config *config1, const struct io_config *config2);
void io_port_clock_init(io_e io);
void io_set_mode(io_e io, io_mode_e mode);
void io_set_output_type(io_e io, io_out_type_e type);
void io_set_pupd(io_e io, io_pupd_e pupd);
void io_set_output_speed(io_e io, io_ouput_speed_e speed);
void io_set_AF(io_e io, io_af_e af);
void io_set_output(io_e io, io_out_e out);
io_in_e io_get_input(io_e io);
io_e *get_io_adc_pins(uint8_t *size);
uint8_t io_adc_idx(io_ports_e io);
void io_set_analog_switch_crl_reg(io_e io);

typedef void (*isr_function)(void);
void io_interrupt_clock_init(void);
void io_interrupt_config(io_e io, isr_function isr, io_trigger_e trigger,
                         io_exti_section_e exti_section);
void io_deconfigure_interrupt(io_e io, io_trigger_e trigger, io_exti_section_e exti_section);
void io_enable_interrupt(io_exti_line_e line);
void io_disable_interrupt(io_exti_line_e line);
void io_set_priority_interrupt(io_exti_line_e line, uint32_t priority);
#endif // IO_H