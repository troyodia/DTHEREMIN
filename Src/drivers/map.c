#include "map.h"
#include "defines.h"
#include "adc.h"
#include "dds.h"
#include "gp2y0a41sk0f.h"
#include "assert_handler.h"
#include <stdbool.h>
#include <math.h>

#define MAX_DISTANCE (30.0f)
#define MIN_DISTANCE (4.0f)

#define MAX_FREQ (1000.0f)
#define MIN_FREQ (50.0f)

/*voltage produced by the GP2Y0A41SK0F when objects exceed the minimum 4cm distance*/
#define GP2Y0A41SK0F_MIN_DISTANCE_V0 (2.9f)
/*voltage produced by the GP2Y0A41SK0F when objects exceed the maximum 30cm distance or when no
 * object is present*/
#define GP2Y0A41SK0F_MAX_DISTANCE_V0 (0.1f)

#define MINOR_PENTA_SIZE (6U)
#define TONE_53_SIZE (53U)

static float pitch_voltage = 0;
static float volume_voltage = 0;

struct gp2y0a41sk0f_values gp2y0a41sk0f_distances;

static float f_map;

static const float scale_minor_pent[] = { 1.0f, 1.122f, 1.335f, 1.498f, 1.782f, 2.0f };

static float normalize_adc_distance_value(float distance)
{
    float normalized_distance = (distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
    if (normalized_distance < 0) {
        return 0;
    } else if (normalized_distance > 1) {
        return 1;
    }
    return normalized_distance;
}

SUPPRESS_UNUSED
static inline void linear_frequency_map(float *f_map, float normalized_pitch)
{
    *f_map = MIN_FREQ + (normalized_pitch * (MAX_FREQ - MIN_FREQ));
}

SUPPRESS_UNUSED
static void minor_penta_frequency_map(float *f_map, float base_freq, float normalized_pitch)
{
    int index = (int)((ARRAY_SIZE(scale_minor_pent) - 1) * normalized_pitch);
    *f_map = base_freq * scale_minor_pent[index];
}

float map_distance_pitch_frequency(map_mode_e mode)
{

    f_map = 0.0f;
    float normalized_raw_pitch = 0.0f;

    gp2y0a41sk0f_adc_to_voltage(MAP_PITCH, &pitch_voltage);
    if (pitch_voltage < GP2Y0A41SK0F_MAX_DISTANCE_V0
        || pitch_voltage > GP2Y0A41SK0F_MIN_DISTANCE_V0) {
        return f_map;
    }
    gp2y0a41sk0f_voltage_to_distance(MAP_PITCH, &gp2y0a41sk0f_distances, pitch_voltage);
    if (gp2y0a41sk0f_distances.pitch_distance < MIN_DISTANCE
        || gp2y0a41sk0f_distances.pitch_distance > MAX_DISTANCE) {
        return f_map;
    }
    normalized_raw_pitch = normalize_adc_distance_value(gp2y0a41sk0f_distances.pitch_distance);

    switch (mode) {
    case MAP_MODE_LINEAR:
        /*LINEAR MAPPING*/
        linear_frequency_map(&f_map, normalized_raw_pitch);
        break;

    case MAP_MODE_PIANO:
        /*MINOR PENTATONIC SCALE*/
        minor_penta_frequency_map(&f_map, 220.0, normalized_raw_pitch);
        break;
    }

    if (f_map < MIN_FREQ)
        f_map = MIN_FREQ;
    if (f_map > MAX_FREQ)
        f_map = MAX_FREQ;
    return f_map;
}
static float invert_volume(float volume)
{
    float volume_invert = 1.0f - volume;
    volume_invert *= volume_invert;
    return volume_invert;
}
float map_distance_volume_frequency(void)
{
    volatile float normalized_raw_vol = 0.0f;
    volatile float normalized_vol_inverted = 0.0f;
    gp2y0a41sk0f_adc_to_voltage(MAP_VOLUME, &volume_voltage);
    if (volume_voltage < GP2Y0A41SK0F_MAX_DISTANCE_V0
        || volume_voltage > GP2Y0A41SK0F_MIN_DISTANCE_V0) {
        return normalized_raw_vol;
    }
    gp2y0a41sk0f_voltage_to_distance(MAP_VOLUME, &gp2y0a41sk0f_distances, volume_voltage);
    if (gp2y0a41sk0f_distances.volume_distance < MIN_DISTANCE
        || gp2y0a41sk0f_distances.volume_distance > MAX_DISTANCE) {
        return normalized_raw_vol;
    }
    normalized_raw_vol = normalize_adc_distance_value(gp2y0a41sk0f_distances.volume_distance);
    normalized_vol_inverted = invert_volume(normalized_raw_vol);
    return normalized_vol_inverted;
}