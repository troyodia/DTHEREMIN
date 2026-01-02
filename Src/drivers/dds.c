#include "dds.h"
#include "map.h"
#include "assert_handler.h"
#include "io.h"
#include "defines.h"
#include "trace.h"
#include "systick.h"
#include "stm32l476xx.h"
#include <stdbool.h>
#include <assert.h>
#include <stdint.h>
#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846) // Define M_PI with its value
#endif // M_pI

#define TABLE_MAX_LENGTH (2000U)
#define AUDIO_DUTY_CYCLE_MAX_LENGTH (64U)
#define SAMPLE_RATE (44100U)
#define N_CYCLES (3U)
#define FREQ_MIN_SAFE (50.0f)
#define FREQ_MAX_SAFE (18000.0f)
#define SENSOR_DISTANCE_UPDATE_RATE (50U)
#define MAX_TABLE_VALUE (4095.0f) // for 12 bit resolution
#define MAX_LOWER_OCTAVE (0.015625f)
#define MAX_UPPER_OCTAVE (64.0f)
// To get the desired sample rate of the timer 44.1khz
// using the formula fclk/((PSC+1) *(ARR+1)), good to make PSC as close to 0 as possible
// with fclk 80Mhz, PSC as 0, ARR is approximatly 1814
#define PRESCALER (0)
#define AUTO_RELOAD (1814U)
#define PWM_ARR (800.0f)
#define DUTY_CYCLE_CORRECTION ((PWM_ARR - 1.0f) / MAX_TABLE_VALUE)

static int16_t look_up_table[TABLE_MAX_LENGTH];
static uint16_t audio_duty_cycles[AUDIO_DUTY_CYCLE_MAX_LENGTH] = { 0 };
static uint32_t phase_acc = 0; // phase accumulator
static uint32_t phase_inc = 0; // phase increment
static uint32_t phase_acc_m = 0; // phase accumulator
static uint32_t phase_inc_m = 0; // phase increment
static bool initialized = false;
static bool first_pass = true;

static float frequency_current = 0.0f;
static float volume_current = 0.0f;
static uint8_t control_cnt = 0;
static dds_mode_e dds_mode = DDS_MODE_NORMAL;
static map_mode_e map_mode = MAP_MODE_LINEAR;
static float current_octave = 1.0f;

static uint32_t current_millis = 0;
static uint32_t previous_mode_millis = 0;
static uint32_t previous_modulation_millis = 0;
static uint32_t previous_octave_up_millis = 0;
static uint32_t previous_octave_down_millis = 0;

SUPPRESS_UNUSED
static void timer_init(void)
{
    RCC->APB1ENR1 |= 0x1; // enable TIM2 clock
    TIM2->ARR = AUTO_RELOAD - 1;
    TIM2->PSC = PRESCALER;

    TIM2->DIER |= 0x1 << 8; // trigger DMA request on update event enable

    TIM2->EGR |= 0x1; // enable update generation
    TIM2->CR1 |= 0x1; // enable timer
}

SUPPRESS_UNUSED
static void dma_init(void)
{
    RCC->AHB1ENR |= 0x1; // enable DMA1 clock
    DMA1_Channel2->CPAR = (uint32_t)&TIM3->CCR1; // Peripheral address
    DMA1_Channel2->CMAR = (uint32_t)audio_duty_cycles; // memory address
    DMA1_Channel2->CNDTR = AUDIO_DUTY_CYCLE_MAX_LENGTH; // length of transfer
    // select the TIM2_UP as the efault trigger on DMA1_Channel2
    DMA1_CSELR->CSELR &= ~(0xF << 4);
    DMA1_CSELR->CSELR |= (0x4 << 4);
    // set size of memory address and peripheral address
    DMA1_Channel2->CCR &= ~((0x3 << 8) | (0x3 << 10));
    DMA1_Channel2->CCR |= ((0x1 << 8) | (0x1 << 10));
    /*
     enable circular mode for DMA
     *enables Memory increment mode, since going through a buffer
     *set data direction from memory to peripheral (TIM3)
     *enables half transfer interrupt
     *enables full transfer interrupt*/
    DMA1_Channel2->CCR |= (0x1 << 5) | (0x1 << 7) | (0x1 << 4) | (0x1 << 2) | (0x1 << 1);
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    NVIC_SetPriority(DMA1_Channel2_IRQn, 5);
    DMA1_Channel2->CCR |= 0x1; // enable DMA
}
// ISR for map button interrupt
static void io_map_mode_btn_isr(void)
{
    current_millis = systick_millis();

    if ((current_millis - previous_mode_millis) > 5) {
        if (map_mode == MAP_MODE_LINEAR) {
            map_mode = MAP_MODE_PIANO;
        } else {
            map_mode = MAP_MODE_LINEAR;
        }
        previous_mode_millis = current_millis;
    }
}
// ISR for sound modulation button interrupt
static void io_dds_modulation_btn_isr(void)
{
    current_millis = systick_millis();

    if ((current_millis - previous_modulation_millis) > 5) {
        if (dds_mode == DDS_MODE_NORMAL) {
            dds_mode = DDS_MODE_MODULATION;
        } else {
            dds_mode = DDS_MODE_NORMAL;
            previous_modulation_millis = current_millis;
        }
    }
}
// ISR for octave down button interrupt
static void io_octave_down_btn_isr(void)
{
    current_millis = systick_millis();

    if ((current_millis - previous_octave_down_millis) > 5) {
        if (current_octave > MAX_LOWER_OCTAVE) {
            current_octave = current_octave / 8.0f;
            previous_octave_down_millis = current_millis;
        }
    }
}
// ISR for octave up button interrupt
static void io_octave_up_btn_isr(void)
{
    current_millis = systick_millis();

    if ((current_millis - previous_octave_up_millis) > 5) {
        if (current_octave < MAX_UPPER_OCTAVE) {
            current_octave = current_octave * 8.0f;
            previous_octave_up_millis = current_millis;
        }
    }
}
// initilaized button interrupts
static void btn_interrupts_init(void)
{
    io_interrupt_config((io_e)IO_PA_0, io_map_mode_btn_isr, IO_FALLING_TRIGGER, IO_EXTI_N_0);
    io_set_priority_interrupt(IO_EXTI_0_LINE, 1);
    io_enable_interrupt(IO_EXTI_0_LINE);

    io_interrupt_config((io_e)IO_PB_3, io_octave_up_btn_isr, IO_FALLING_TRIGGER, IO_EXTI_N_3);
    io_set_priority_interrupt(IO_EXTI_3_LINE, 2);
    io_enable_interrupt(IO_EXTI_3_LINE);

    io_interrupt_config((io_e)IO_PB_4, io_octave_down_btn_isr, IO_FALLING_TRIGGER, IO_EXTI_N_0);
    io_set_priority_interrupt(IO_EXTI_3_LINE, 3);
    io_enable_interrupt(IO_EXTI_4_LINE);

    io_interrupt_config((io_e)IO_PB_5, io_dds_modulation_btn_isr, IO_FALLING_TRIGGER, IO_EXTI_N_1);
    io_set_priority_interrupt(IO_EXTI_9_5_LINE, 4);
    io_enable_interrupt(IO_EXTI_9_5_LINE);
}

SUPPRESS_UNUSED
static void fill_square_wave_table(float duty_cycle, int16_t *table_value, uint16_t i)
{
    int16_t samples_per_period;
    int16_t threshold;
    samples_per_period = TABLE_MAX_LENGTH / 24;
    threshold = (int16_t)(duty_cycle * samples_per_period);

    int16_t pos_in_period = i % samples_per_period;
    if (pos_in_period < threshold) {
        *table_value = (int16_t)MAX_TABLE_VALUE / 2;
    } else {
        *table_value = 0;
    }
}

SUPPRESS_UNUSED
static void fill_look_up_table(void)
{
    /*A 2000 sample table, using 12 bit resolution
     *limitting resolution to DAC, incase of use
     * DAC->12bit vs TIM2 -> 16-32 bits
     * max value for 12 bit is 4095*/
    int16_t table_value = 0;
    for (uint16_t i = 0; i < TABLE_MAX_LENGTH; i++) {
        fill_square_wave_table(0.75f, &table_value, i);
        int16_t max_table_val = (int16_t)(MAX_TABLE_VALUE / 2);
        if (table_value > max_table_val) {
            table_value = max_table_val;
        } else if (table_value < (-1 * max_table_val)) {
            table_value = (-1 * max_table_val);
        }
        look_up_table[i] = table_value;
    }
}

SUPPRESS_UNUSED
static float lut_lookup_interp(uint32_t phase_acc)
{
    // phase_acc: 32-bit
    uint32_t index = phase_acc >> (32 - 10); // *table_value part
    uint32_t frac = (phase_acc >> (32 - 10 - 16)) & 0xFFFF; // 16-bit fraction
    int16_t a = look_up_table[index & (TABLE_MAX_LENGTH - 1)];
    int16_t b = look_up_table[(index + 1) & (TABLE_MAX_LENGTH - 1)];
    // linear interpolate in 16-bit fixed point
    int32_t val = ((uint32_t)a * (0x10000 - frac) + (uint32_t)b * frac) >> 16;
    float norm = (float)((uint16_t)val / (MAX_TABLE_VALUE / 2));
    return norm;
}

SUPPRESS_UNUSED
static float hz_to_inc(float frequency)
{
    float num = (2.0f * frequency * M_PI);
    return (num / (float)SAMPLE_RATE);
}
SUPPRESS_UNUSED
static void dds_phase_accumlation(uint16_t *audio_data, uint8_t size)
{
    volatile uint16_t sample = 0;
    volatile float frequency = 0.0f;
    volatile float volume = 0.0f;
    for (uint8_t i = 0; i < size; i++) {
        if (first_pass || (++control_cnt >= SENSOR_DISTANCE_UPDATE_RATE)) {
            control_cnt = first_pass ? control_cnt : 0;
            first_pass = false;
            volume = map_distance_volume_frequency();
            switch (map_mode) {
            case MAP_MODE_LINEAR:
                frequency = map_distance_pitch_frequency(MAP_MODE_LINEAR);
                break;

            case MAP_MODE_PIANO:
                frequency = map_distance_pitch_frequency(MAP_MODE_PIANO);
                break;
            }

            frequency *= current_octave;

            uint64_t num = (uint64_t)((double)frequency * 4294967296.0);
            uint32_t new_inc = (uint32_t)(num / (uint64_t)SAMPLE_RATE);
            uint32_t new_inc_m = hz_to_inc(100.0f);

            __disable_irq();
            frequency_current = frequency;
            volume_current = volume;
            phase_inc = new_inc;
            phase_inc_m = new_inc_m;
            __enable_irq();
        }

        if ((frequency_current == 0.0f) || (volume_current == 0.0f)) {
            sample = 0;
        } else {
            phase_acc += phase_inc;
            if (dds_mode == DDS_MODE_MODULATION) {
                phase_acc_m += phase_inc_m;
                if (phase_acc_m > 2 * M_PI)
                    phase_acc_m -= 2 * M_PI;

                float fm_mod = 0.1f * sinf(5 * phase_acc_m);
                uint32_t fm_inc_mod = (uint32_t)(
                    (frequency * (1.0f + (fm_mod * 0.01f)) * (1ULL << 32)) / SAMPLE_RATE);

                phase_acc += fm_inc_mod;
            }
            float duty_norm = (lut_lookup_interp(phase_acc) * volume_current * 0.5f) + 0.5f;

            sample =
                (uint16_t)(duty_norm * PWM_ARR); // scale the sample value to PWM range 0 to 799
        }
        audio_data[i] = sample;
    }
}

void dds_init(void)
{
    ASSERT(!initialized);
    fill_look_up_table();
    // preload audio buffer with samples before initbailizing timer and DMA
    dds_phase_accumlation(audio_duty_cycles, AUDIO_DUTY_CYCLE_MAX_LENGTH);
    dma_init();
    timer_init();
    btn_interrupts_init();
    initialized = true;
}

// this interrupt is to fill future halves of the audio buffer
void DMA1_CH2_IRQHandler(void)
{
    // interrupt is raised when half of the transfer is done
    if (DMA1->ISR & 0x1 << 6) {
        DMA1->IFCR = 0x1 << 6;
        dds_phase_accumlation(audio_duty_cycles, AUDIO_DUTY_CYCLE_MAX_LENGTH / 2);
    }
    //  interrupt is raised when the full transfer is done
    if (DMA1->ISR & 0x1 << 5) {
        DMA1->IFCR = 0x1 << 5;
        dds_phase_accumlation(&audio_duty_cycles[AUDIO_DUTY_CYCLE_MAX_LENGTH / 2],
                              AUDIO_DUTY_CYCLE_MAX_LENGTH / 2);
    }
}
