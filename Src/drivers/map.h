#ifndef MAP_H
#define MAP_H
/* Carries out Pitch and Volume normalization and mapping */
#include <stdint.h>
typedef enum
{
    MAP_MODE_LINEAR,
    MAP_MODE_PIANO
} map_mode_e;

float map_distance_pitch_frequency(map_mode_e mode); // maps desired pitch to frequency
float map_distance_volume_frequency(void); // maps desired volume to frequency
#endif // MAP_H