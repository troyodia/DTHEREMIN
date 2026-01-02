#include "lp_filter.h"
#include "defines.h"
#define ALPHA (0.35f)

void lp_set_alpha(float alpha, struct lp_filter *voltage_filter)
{
    if (alpha < 0.0f) {
        alpha = 0.0f;
    }
    if (alpha > 1.0f) {
        alpha = 1.0f;
    }
    voltage_filter->alpha = alpha;
}
SUPPRESS_UNUSED
float lp_filter_update(float voltage, struct lp_filter *voltage_filter)

{
    lp_set_alpha(ALPHA, voltage_filter);
    voltage_filter->vout =
        (voltage_filter->alpha * voltage) + ((1 - voltage_filter->alpha) * voltage_filter->vout);
    return voltage_filter->vout;
}