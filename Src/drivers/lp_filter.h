#ifndef LP_FILTER
#define LP_FILTER
/*Lowpass exponential filter to filter voltage data from IR snesors*/
struct lp_filter
{
    float alpha;
    float vout;
};
void lp_set_alpha(float alpha, struct lp_filter *voltage_filter);
float lp_filter_update(float voltage, struct lp_filter *voltage_filter);
#endif // LP_FILTER