#pragma once

#include <stdint.h>

typedef enum {
    LED_CONNECTING,     /* blue,   slow blink  - boot / zigbee steering  */
    LED_READY,          /* green,  solid       - joined, listening       */
    LED_RECEIVING,      /* green,  fast blink  - BLE data transfer       */
    LED_ERROR,          /* red,    fast blink  - error                   */
} led_state_t;

void led_status_init(void);
void led_status_set(led_state_t state);

/* Direct LED control for boot-time animations (before the LED task matters). */
void led_status_raw(uint8_t r, uint8_t g, uint8_t b);
void led_status_off(void);
