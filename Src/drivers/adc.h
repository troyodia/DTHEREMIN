#ifndef ADC_H
#define ADC_H
// driver for getting the IR sensor ADC data
#include <stdint.h>
// Using ADC2 and it has 16 channels

#define ADC_CHANNEL_CNT (2)

typedef enum
{
    ADC2_IN1,
    ADC2_IN2,
    ADC2_IN3,
    ADC2_IN4,
    ADC2_IN5,
    ADC2_IN6,
    ADC2_IN7,
    ADC2_IN8,
    ADC2_IN9,
    ADC2_IN10,
    ADC2_IN11,
    ADC2_IN12,
    ADC2_IN13,
    ADC2_IN14,
    ADC2_IN15,
    ADC2_IN16,
} adc2_chs_e;
typedef uint16_t adc_channel_values[ADC_CHANNEL_CNT];
void adc_init(void);
void adc_single_init(void);
// ch_vals after typdef -> uint16_t ch_vals[16] so already passed as a pointer after typdef
void adc_read_channel_values(adc_channel_values ch_vals);
void adc_read_value(uint32_t *ch_vals);
#endif // ADC_H