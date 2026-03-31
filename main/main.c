#include "esp_log.h"
#include "nvs_flash.h"
#include "ble_bp_client.h"
#include "zigbee_bp_device.h"
#include "bp_parser.h"
#include "led_status.h"

static const char *TAG = "main";

static void on_measurement(const bp_measurement_t *meas)
{
    ESP_LOGI(TAG, "measurement received: systolic=%.0f diastolic=%.0f pulse=%.0f",
             meas->systolic, meas->diastolic, meas->pulse);
    led_status_set(LED_MEASUREMENT);
    zigbee_bp_device_update(meas);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    led_status_init();
    led_status_set(LED_BOOTING);

    ESP_LOGI(TAG, "starting Zigbee");
    zigbee_bp_device_start();

    ESP_LOGI(TAG, "starting BLE client");
    ble_bp_client_init(on_measurement);
    ble_bp_client_start();
}
