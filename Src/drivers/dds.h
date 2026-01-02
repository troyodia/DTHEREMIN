#ifndef DDS_H
#define DDS_H
/*For carrying out DDS algorithm to generate duty cylces for PWM generation*/
#include <stdint.h>
typedef enum
{
    DDS_MODE_NORMAL,
    DDS_MODE_MODULATION,
} dds_mode_e;
void dds_init(void); // initialize timer, dma and create sine table
#endif // DDS_H