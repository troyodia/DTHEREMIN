#include "adc.h"
#include "assert_handler.h"
#include "io.h"
#include "defines.h"
#include "stm32l4xx.h"
#include <stdbool.h>
static bool initialized = false;
static io_e *adc_pins;
static uint8_t adc_pins_arr_size;
static uint8_t adc_channels[] = { 1, 2 };
static volatile adc_channel_values adc_channel_data;
static volatile adc_channel_values adc_channel_data_cache;
static uint8_t dma_data_cnt = ADC_CHANNEL_CNT;
static uint8_t adc_sequence_scan_length = 0x1;

static void adc_set_chs_sample_time()
{
    for (uint8_t channel = 0; channel < ARRAY_SIZE(adc_channels); channel++) {
        ADC2->SMPR1 &= ~(0x6 << (adc_channels[channel] * 3));
        ADC2->SMPR1 |= (0x6 << (adc_channels[channel] * 3));
    }
}

static void adc_set_channel_sequences()
{
    // tell the ADC that 2 conversations will take place,
    // enables scan mode
    ADC2->SQR1 &= ~0xF;
    ADC2->SQR1 |= adc_sequence_scan_length;
    // set each channel as a sequence in the SQR regsiters 1-4
    for (uint8_t channel = 0; channel < ARRAY_SIZE(adc_channels); channel++) {
        uint8_t adc_channel = adc_channels[channel];

        ADC2->SQR1 &= ~(0x1F << (6 * (adc_channel)));
        ADC2->SQR1 |= (adc_channel << (6 * adc_channel));
    }
}

static void dma_init(void)
{
    RCC->AHB1ENR |= 0x1 << 1; // DMA2 clock
    /* set the peripheral adc_pins_arr_size, ADC data register is 32 bits but converts to 12 bits so
     * 16 is enough set the memory adc_pins_arr_size, adc_pins_arr_size of the array is 16 bits*/
    DMA2_Channel4->CCR &= ~((0x3 << 8) | (0x3 << 10));
    DMA2_Channel4->CCR |= (0x1 << 8) | (0x1 << 10);

    DMA2_Channel4->CCR &= ~(0x1 << 4); // set data transfer direction, reading from peripheral ADC

    DMA2_Channel4->CNDTR = dma_data_cnt; // Set the number of data to transfer, 4 Line senaors
    DMA2_Channel4->CMAR =
        (uint32_t)adc_channel_data; // set the DMA memory address to the adc data array
    DMA2_Channel4->CPAR =
        (uint32_t)&ADC2->DR; // set the peripheral address to the ADC data register
    /* enable memory address increment mode
     * enable circular mode mode
     * enable DMA*/
    DMA2_Channel4->CCR |= (0x1 << 7) | (0x1 << 5) | 0x1;
}
static inline void adc_start_conversion(void)
{
    ADC2->CR |= 0x1 << 2; // start ADC conversion
}

void adc_init(void)
{
    ASSERT(!initialized);
    adc_pins = get_io_adc_pins(&adc_pins_arr_size);
    // Clocks
    RCC->AHB2ENR |= 0x1 << 13; // adc clock
    RCC->CCIPR &= ~(0x3 << 28); // set ADC clock source to PLLSAI1R
    RCC->CCIPR |= (0x1 << 28); // set ADC clock source to PLLSAI1R
    ADC123_COMMON->CCR &= ~(0x3 << 16);
    ADC123_COMMON->CCR &= ~(0xF << 18);
    ADC123_COMMON->CCR |= (0x2 << 18); // reduce clock rate for ADC by a division of 4
    // DMA
    dma_init();
    // ADC
    /*check to see if ADC is enabled, if it is set ADDIS to disable ADC*/
    if (ADC2->CR & (0x1)) {
        ADC2->CR |= 0x1 << 1;
        while (ADC2->CR & (0x1))
            ;
    }
    /*Exit deep power mode by setting the DEEPPWD bit to 0 first, as ADC is by default in deep power
     * down mode */
    ADC2->CR &= ~(0x1 << 29);
    /* enable the ADC voltage regulator ADVREGEN, this should be enabled before calibrating or
     enabling ADC */
    ADC2->CR |= 0x1 << 28;
    /*wait for startup timer ~20us after configureing DEEPPWD and ADVREGEN
    volatile so complier doesnt optimize the delay away*/
    for (volatile int i = 0; i < 15000; i++)
        ;

    // calibrate ADC2
    // use ADC CAL define to avaoid cpp bit shift error
    ADC2->CR |= ADC_CR_ADCAL;
    while (ADC2->CR & (ADC_CR_ADCAL))
        ;
    adc_set_chs_sample_time(); // set sampling rate for each channel
    adc_set_channel_sequences();

    // enable Overrun mode
    // enable continious mode for ADC
    // enable DMA circular mode for ADC
    // enable DMA for ADC
    ADC1->CFGR = 0;
    ADC1->CFGR |= (0x1 << 12);
    ADC2->CFGR |= (0x1 << 13) | (0x1 << 1) | 0x1;
    ADC2->IER |= 0x1 << 3; // ADC enbale end of sequence
    NVIC_SetPriority(ADC1_2_IRQn, 6);
    NVIC_EnableIRQ(ADC1_2_IRQn); // enable ADC 1 and 2 global interrupt

    ADC2->CR |= 0x1; // enable ADC
    while (!(ADC2->ISR & 0x1))
        ;
    adc_start_conversion();

    initialized = true;
}

void ADC1_2_IRQHandler(void)
{
    /*Check the EOSS flag and EOS register to see if a sequence of conversions has ended and the
     * interrupt is enabled*/
    if ((ADC2->ISR & (0x1 << 3)) && (ADC2->IER & (0x1 << 3))) {
        /*copy ADC channel data from DMA output memeory location into a cache array
         * do this to not interfer with the DMA output memory location
         */
        for (uint8_t channel = 0; channel < dma_data_cnt; channel++) {
            adc_channel_data_cache[channel] = adc_channel_data[channel];
        }
        // clear the flag after handling the interrupt
        ADC2->ISR |= (0x1 << 3);
    }
}

void adc_read_channel_values(adc_channel_values ch_vals)

{
    /* disable all interrupts during copying to prevent any data corruptions from another set of
     * channel readings*/
    __disable_irq();
    for (uint8_t i = 0; i < adc_pins_arr_size; i++) {
        const uint8_t channel_idx = io_adc_idx((io_ports_e)adc_pins[i]);
        ch_vals[channel_idx] = adc_channel_data_cache[channel_idx];
    }
    __enable_irq();
}

SUPPRESS_UNUSED
// to test adc function of a single pin using polling
void adc_single_init(void)
{

    RCC->AHB2ENR |= 0x1 << 13; // adc clock
    RCC->CCIPR &= ~(0x3 << 28); // set ADC clock source to PLLSAI1R
    RCC->CCIPR |= (0x1 << 28); // set ADC clock source to PLLSAI1R
    ADC123_COMMON->CCR &= ~(0x3 << 16);
    ADC123_COMMON->CCR &= ~(0xF << 18);
    ADC123_COMMON->CCR |= (0x4 << 18); // reduce clock rate for ADC
    // GPIOA->ASCR |= 0x1;
    /*check to see if ADC is enabled, if it is set ADDIS to disable ADC*/
    if (ADC1->CR & (0x1)) {
        ADC1->CR |= 0x1 << 1;
        while (ADC1->CR & (0x1))
            ;
    }
    // ADC1->CR = 0;
    /*Exit deep power mode by setting the DEEPPWD bit to 0 first, as ADC is by default in deep power
     * down mode */
    ADC1->CR &= ~(0x1 << 29);
    /* enable the ADC voltage regulator ADVREGEN, this should be enabled before calibrating or
     enabling ADC */
    ADC1->CR |= 0x1 << 28;
    /*wait for startup timer ~20us after configureing DEEPPWD and ADVREGEN
    volatile so complier doesnt optimize the delay away*/
    for (volatile int i = 0; i < 20000; i++)
        ;

    // calibrate ADC2
    // use ADC CAL define to avaoid cpp bit shift error
    ADC2->CR |= ADC_CR_ADCAL;
    while (ADC2->CR & (ADC_CR_ADCAL))
        ;

    ADC1->CFGR = 0;
    ADC1->CFGR |= ADC_CFGR_OVRMOD;

    ADC1->SMPR1 &= ~(0x7 << (15));
    ADC1->SMPR1 |= (0x7 << 15);
    ADC1->SQR1 &= ~(0x1F << (6));
    ADC1->SQR1 |= (5 << (6));

    ADC1->CR |= 0x1; // enable ADC
    while (!(ADC1->ISR & 0x1))
        ;
}
SUPPRESS_UNUSED
void adc_read_value(uint32_t *ch_vals)
{
    ADC1->ISR |= ADC_ISR_EOC;

    ADC1->CR |= 0x1 << 2;
    while (!(ADC1->ISR & ADC_ISR_EOC))
        ;
    *ch_vals = ADC1->DR;
}