#include "gp2y0a41sk0f.h"
#include "assert_handler.h"
#include "defines.h"
#include <stdbool.h>

static bool initialized = false;
#define CALIBRATION_SAMPLES (100U)
#define V_REF (3.3f)
#define MAX_ADC_VALUE (4095.0f)

/*voltage produced by the GP2Y0A41SK0F when objects exceed the minimum 4cm distance*/
#define GP2Y0A41SK0F_MIN_DISTANCE_V0 (2.9f)
/*voltage produced by the GP2Y0A41SK0F when objects exceed the maximum 30cm distance or when no
 * object is present*/
#define GP2Y0A41SK0F_MAX_DISTANCE_V0 (0.35f)

#define ADC_TO_VOLTAGE(adc) (adc * (V_REF / MAX_ADC_VALUE))
/* Formula derived from linear part of output distance with inverse of distance in datasheet
 * The slope of the linear part of the graph is 13*/
#define VOLTAGE_TO_DISTANCE(Vo) ((13.0f / Vo) - 0.42f)
#define VOLTAGE_TO_DISTANCE_50CM(Vo) ((((Vo - 2.0f) * (50.0f - 10.0f)) / (0.6f - 2.0f)) + 10.0f)

static struct lp_filter pitch_filter;
static struct lp_filter volume_filter;

void gp2y0a41sk0f_adc_to_voltage_test(float *voltage, float *distance, uint16_t adc)
{
    *voltage = (float)ADC_TO_VOLTAGE(adc);
    *distance = VOLTAGE_TO_DISTANCE(*voltage);
}

static void initialize_gp2y0a41sk0f(void)
{
    pitch_filter.vout = 0;
    volume_filter.vout = 0;
}

void gp2y0a41sk0f_init(void)
{
    ASSERT(!initialized);
    adc_init();
    initialize_gp2y0a41sk0f();
}
void gp2y0a41sk0f_adc_to_voltage(map_e map, float *voltage)
{

    adc_channel_values ir_adc_values = { 0 };
    adc_read_channel_values(ir_adc_values);
    float raw_voltage;
    switch (map) {
    case MAP_PITCH:
        raw_voltage = (float)ADC_TO_VOLTAGE(ir_adc_values[0]);
        *voltage = lp_filter_update(raw_voltage, &pitch_filter);
        break;
    case MAP_VOLUME:
        raw_voltage = (float)ADC_TO_VOLTAGE(ir_adc_values[1]);
        *voltage = lp_filter_update(raw_voltage, &volume_filter);
        break;
    }
}

void gp2y0a41sk0f_voltage_to_distance(map_e map, struct gp2y0a41sk0f_values *gp2y0a41sk0f_distances,
                                      float voltage)
{
    switch (map) {
    case MAP_PITCH:
        gp2y0a41sk0f_distances->pitch_distance = VOLTAGE_TO_DISTANCE(voltage);
        break;
    case MAP_VOLUME:
        gp2y0a41sk0f_distances->volume_distance = VOLTAGE_TO_DISTANCE(voltage);
        break;
    }
}
