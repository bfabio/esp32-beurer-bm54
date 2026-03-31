#include "zigbee_bp_device.h"

#include <string.h>
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "led_status.h"
#include "nwk/esp_zigbee_nwk.h"
#include "platform/esp_zigbee_platform.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "zigbee_bp";

#define EP_SYSTOLIC   1
#define EP_DIASTOLIC  2
#define EP_PULSE      3

#define ZB_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

static QueueHandle_t s_meas_queue;

static esp_zb_ep_list_t *create_ep_list(void)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version   = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source  = 0x03,
    };

    /* EP 1: Basic + Analog Input (Systolic) */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(&basic_cfg);
        esp_zb_basic_cluster_add_attr(basic,
            ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, "Beurer");
        esp_zb_basic_cluster_add_attr(basic,
            ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, "BM54");
        esp_zb_cluster_list_add_basic_cluster(cl, basic,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_analog_input_cluster_cfg_t ai_cfg = {
            .present_value  = 0.0f,
            .out_of_service = false,
            .status_flags   = 0,
        };
        esp_zb_attribute_list_t *ai = esp_zb_analog_input_cluster_create(&ai_cfg);
        esp_zb_analog_input_cluster_add_attr(ai,
            ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID, "Systolic");
        esp_zb_cluster_list_add_analog_input_cluster(cl, ai,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint        = EP_SYSTOLIC,
            .app_profile_id  = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id   = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0,
        };
        esp_zb_ep_list_add_ep(ep_list, cl, ep_cfg);
    }

    /* EP 2: Analog Input (Diastolic) */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        esp_zb_analog_input_cluster_cfg_t ai_cfg = {
            .present_value  = 0.0f,
            .out_of_service = false,
            .status_flags   = 0,
        };
        esp_zb_attribute_list_t *ai = esp_zb_analog_input_cluster_create(&ai_cfg);
        esp_zb_analog_input_cluster_add_attr(ai,
            ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID, "Diastolic");
        esp_zb_cluster_list_add_analog_input_cluster(cl, ai,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint        = EP_DIASTOLIC,
            .app_profile_id  = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id   = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0,
        };
        esp_zb_ep_list_add_ep(ep_list, cl, ep_cfg);
    }

    /* EP 3: Analog Input (Pulse Rate) */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        esp_zb_analog_input_cluster_cfg_t ai_cfg = {
            .present_value  = 0.0f,
            .out_of_service = false,
            .status_flags   = 0,
        };
        esp_zb_attribute_list_t *ai = esp_zb_analog_input_cluster_create(&ai_cfg);
        esp_zb_analog_input_cluster_add_attr(ai,
            ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID, "Pulse Rate");
        esp_zb_cluster_list_add_analog_input_cluster(cl, ai,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint        = EP_PULSE,
            .app_profile_id  = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id   = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0,
        };
        esp_zb_ep_list_add_ep(ep_list, cl, ep_cfg);
    }

    return ep_list;
}

static void set_present_value(uint8_t endpoint, float value)
{
    esp_zb_zcl_set_attribute_val(
        endpoint,
        ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
        &value,
        false);
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialised");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Device is factory-new, starting network steering");
                led_status_set(LED_ZB_STEERING);
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Rejoining existing network");
                led_status_set(LED_ZB_JOINED);
            }
        } else {
            ESP_LOGW(TAG, "Startup failed (%s), retrying steering",
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm(
                (esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network, PAN ID 0x%04hx, channel %d",
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
            led_status_set(LED_ZB_JOINED);
        } else {
            ESP_LOGW(TAG, "Steering failed (%s), retrying in 1s",
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm(
                (esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;

    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s",
                 esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static void zigbee_task(void *arg)
{
    esp_zb_cfg_t zb_cfg = {
        .esp_zb_role        = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = false,
        .nwk_cfg.zed_cfg = {
            .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN,
            .keep_alive = 4000,
        },
    };
    esp_zb_init(&zb_cfg);

    esp_zb_ep_list_t *ep_list = create_ep_list();
    esp_zb_device_register(ep_list);

    esp_zb_set_primary_network_channel_set(ZB_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));

    while (1) {
        esp_zb_stack_main_loop_iteration();

        bp_measurement_t meas;
        if (xQueueReceive(s_meas_queue, &meas, 0) == pdTRUE) {
            ESP_LOGI(TAG, "updating Zigbee attributes: %.0f/%.0f mmHg, pulse %.0f bpm",
                     meas.systolic, meas.diastolic, meas.pulse);

            esp_zb_lock_acquire(portMAX_DELAY);
            set_present_value(EP_SYSTOLIC,  meas.systolic);
            set_present_value(EP_DIASTOLIC, meas.diastolic);
            if (meas.has_pulse) {
                set_present_value(EP_PULSE, meas.pulse);
            }
            esp_zb_lock_release();
        }
    }
}

void zigbee_bp_device_start(void)
{
    s_meas_queue = xQueueCreate(4, sizeof(bp_measurement_t));
    assert(s_meas_queue);

    esp_zb_platform_config_t config = {
        .radio_config = { .radio_mode = ZB_RADIO_MODE_NATIVE },
        .host_config  = { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(zigbee_task, "zigbee_task", 4096, NULL, 5, NULL);
}

void zigbee_bp_device_update(const bp_measurement_t *meas)
{
    if (s_meas_queue) {
        xQueueSend(s_meas_queue, meas, 0);
    }
}

void zigbee_bp_set_ble_connecting(bool ble_connecting)
{
    uint32_t interval_ms = ble_connecting ? 60000 : 7500;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_set_default_long_poll_interval(interval_ms);
    esp_zb_lock_release();
}
