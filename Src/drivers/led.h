#ifndef LED_H
// Led driver to control the IO_TEST_LED pin and blink the led it is connected to
#define LED_H

typedef enum
{
    LED_TEST
} led_e;
typedef enum
{
    LED_STATE_OFF,
    LED_STATE_ON,
} led_state_e;

void led_init(void);
void led_set(led_e led, led_state_e state);
#endif // LED_H
