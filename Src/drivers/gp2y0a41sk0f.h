#ifndef GP2Y0A41SK0F_H
#define GP2Y0A41SK0F_H
/*Gets ADC values from gp2y0a41sk0f IR sensor*/
// distance in cm
#include "lp_filter.h"
#include "adc.h"

#include <stdint.h>
struct gp2y0a41sk0f_values
{
    float pitch_distance;
    float volume_distance;
};

typedef enum
{
    MAP_PITCH,
    MAP_VOLUME,
} map_e;

void gp2y0a41sk0f_init(void);
void gp2y0a41sk0f_adc_to_voltage(map_e map, float *voltage);
void gp2y0a41sk0f_voltage_to_distance(map_e map, struct gp2y0a41sk0f_values *gp2y0a41sk0f_distances,
                                      float voltage);
// test function
void gp2y0a41sk0f_adc_to_voltage_test(float *voltage, float *distance, uint16_t adc);

#endif // GP2Y0A41SK0F_H