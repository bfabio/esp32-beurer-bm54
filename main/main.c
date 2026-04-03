#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ble_bp_client.h"
#include "zigbee_bp_device.h"
#include "bp_parser.h"
#include "led_status.h"

static const char *TAG = "main";

#define BOOT_BUTTON_GPIO  9
#define FACTORY_HOLD_MS   3000
#define FACTORY_WINDOW_MS 5000
#define BRIGHTNESS        20

/* Called from the NimBLE task on every BM54 indication */
static void on_measurement(const bp_measurement_t *meas)
{
    ESP_LOGI(TAG, "measurement received: systolic=%.0f diastolic=%.0f pulse=%.0f",
             meas->systolic, meas->diastolic, meas->pulse);
    led_status_set(LED_RECEIVING);
    zigbee_bp_device_update(meas);
}

/*
 * After boot, poll BOOT button for up to FACTORY_WINDOW_MS.
 *
 * Visual feedback while holding:
 *   - red blink that accelerates from slow to frantic
 *   - at FACTORY_HOLD_MS: rapid red-white-green sweep → reset confirmed
 */
static bool check_factory_reset(void)
{
    gpio_config_t btn_cfg = {
        .pin_bit_mask = 1ULL << BOOT_BUTTON_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&btn_cfg);

    int held_ms = 0;
    bool led_on = false;
    int blink_timer = 0;

    for (int elapsed = 0; elapsed < FACTORY_WINDOW_MS; elapsed += 10) {
        vTaskDelay(pdMS_TO_TICKS(10));

        if (gpio_get_level(BOOT_BUTTON_GPIO) != 0) {
            if (held_ms > 0) {
                led_status_off();
                return false;
            }
            continue;
        }

        if (held_ms == 0) {
            ESP_LOGW(TAG, "BOOT button pressed, hold %d s for factory reset...",
                     FACTORY_HOLD_MS / 1000);
        }
        held_ms += 10;

        if (held_ms >= FACTORY_HOLD_MS) {
            /* Confirm animation: red → white → green */
            led_status_raw(BRIGHTNESS, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(120));
            led_status_raw(BRIGHTNESS, BRIGHTNESS, BRIGHTNESS);
            vTaskDelay(pdMS_TO_TICKS(120));
            led_status_raw(0, BRIGHTNESS, 0);
            vTaskDelay(pdMS_TO_TICKS(400));
            led_status_off();

            ESP_LOGW(TAG, "Factory reset: erasing NVS (Zigbee + BLE bonds)");
            ESP_ERROR_CHECK(nvs_flash_erase());
            ESP_ERROR_CHECK(nvs_flash_init());
            return true;
        }

        /* Blink period: 500ms at start → 60ms near the end */
        float progress = (float)held_ms / FACTORY_HOLD_MS;
        int period_ms = 500 - (int)(440 * progress);

        blink_timer += 10;
        if (blink_timer >= period_ms / 2) {
            blink_timer = 0;
            led_on = !led_on;
            if (led_on)
                led_status_raw(BRIGHTNESS, 0, 0);
            else
                led_status_off();
        }
    }
    return false;
}

void app_main(void)
{
    /* NVS is required by both NimBLE (bond storage) and Zigbee */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    led_status_init();
    led_status_set(LED_CONNECTING);

    bool factory_new = check_factory_reset();

    ESP_LOGI(TAG, "starting Zigbee");
    zigbee_bp_device_start(factory_new);

    ESP_LOGI(TAG, "starting BLE client");
    ble_bp_client_init(on_measurement);
    ble_bp_client_start();
}
