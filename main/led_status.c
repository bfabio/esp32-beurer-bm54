#include "led_status.h"

#include "led_strip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#define LED_GPIO    8       /* WS2812 on ESP32-H2-DevKitM-1 */
#define BRIGHTNESS  20      /* 0-255; keep low to avoid eye strain */

static const char *TAG = "led";

static led_strip_handle_t s_strip;
static led_state_t        s_state = LED_CONNECTING;
static SemaphoreHandle_t  s_mutex;

typedef struct { uint8_t r, g, b; } rgb_t;

static const rgb_t COL_BLUE  = { 0, 0, BRIGHTNESS };
static const rgb_t COL_GREEN = { 0, BRIGHTNESS, 0 };
static const rgb_t COL_RED   = { BRIGHTNESS, 0, 0 };

static void set_pixel(rgb_t c)
{
    led_strip_set_pixel(s_strip, 0, c.r, c.g, c.b);
    led_strip_refresh(s_strip);
}

static void led_off(void)
{
    led_strip_clear(s_strip);
}

static void blink(rgb_t colour, uint32_t on_ms, uint32_t off_ms)
{
    set_pixel(colour);
    vTaskDelay(pdMS_TO_TICKS(on_ms));
    led_off();
    vTaskDelay(pdMS_TO_TICKS(off_ms));
}

static void led_task(void *arg)
{
    while (1) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        led_state_t state = s_state;
        xSemaphoreGive(s_mutex);

        switch (state) {
        case LED_CONNECTING:
            blink(COL_BLUE, 500, 500);
            break;

        case LED_READY:
            set_pixel(COL_GREEN);
            vTaskDelay(pdMS_TO_TICKS(200));
            break;

        case LED_RECEIVING:
            blink(COL_GREEN, 100, 100);
            break;

        case LED_ERROR:
            blink(COL_RED, 150, 150);
            break;
        }
    }
}

void led_status_init(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num   = LED_GPIO,
        .max_leds         = 1,
        .led_model        = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip));
    led_strip_clear(s_strip);

    s_mutex = xSemaphoreCreateMutex();
    assert(s_mutex);

    xTaskCreate(led_task, "led_task", 2048, NULL, 3, NULL);
    ESP_LOGI(TAG, "LED status task started");
}

void led_status_set(led_state_t state)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_state = state;
    xSemaphoreGive(s_mutex);
}

void led_status_raw(uint8_t r, uint8_t g, uint8_t b)
{
    led_strip_set_pixel(s_strip, 0, r, g, b);
    led_strip_refresh(s_strip);
}

void led_status_off(void)
{
    led_strip_clear(s_strip);
}
