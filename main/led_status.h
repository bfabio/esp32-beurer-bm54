#pragma once

typedef enum {
    LED_BOOTING,        /* white,  slow blink  - startup              */
    LED_ZB_STEERING,    /* blue,   fast blink  - joining Zigbee net   */
    LED_ZB_JOINED,      /* blue,   solid 1s    - network joined       */
    LED_BLE_SCANNING,   /* yellow, slow blink  - looking for BM54     */
    LED_BLE_CONNECTED,  /* green,  solid       - BLE connected        */
    LED_MEASUREMENT,    /* white,  3 flashes   - measurement received */
    LED_ERROR,          /* red,    fast blink  - unrecoverable error  */
} led_state_t;

void led_status_init(void);
void led_status_set(led_state_t state);
