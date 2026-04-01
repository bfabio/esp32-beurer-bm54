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

/* Endpoints */
#define EP_SYSTOLIC   1
#define EP_DIASTOLIC  2
#define EP_PULSE      3

/* Zigbee channel mask - use all channels; coordinator will pick */
#define ZB_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

/* Queue used to pass measurements from NimBLE task to Zigbee task */
static QueueHandle_t s_meas_queue;

/* ------------------------------------------------------------------ */
/* Endpoint / cluster creation helpers                                  */
/* ------------------------------------------------------------------ */

static esp_zb_ep_list_t *create_ep_list(void)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    /* Shared Basic cluster config (only added to EP 1) */
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version   = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source  = 0x03, /* Battery */
    };

    /* ---- EP 1: Basic + PressureMeasurement (Systolic) ---- */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(&basic_cfg);
        esp_zb_basic_cluster_add_attr(basic,
            ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, "\x06" "Beurer");
        esp_zb_basic_cluster_add_attr(basic,
            ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, "\x04" "BM54");
        esp_zb_cluster_list_add_basic_cluster(cl, basic,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        /* ZHA reads measured_value as hPa. We store mmHg converted to hPa
         * (1 mmHg = 1.33322 hPa) so that ZHA's built-in unit conversion to
         * mmHg in the HA UI produces the correct value. */
        esp_zb_pressure_meas_cluster_cfg_t pm_cfg = {
            .measured_value = 0,
            .min_value      = 80,   /* ~60 mmHg */
            .max_value      = 400,  /* ~300 mmHg */
        };
        esp_zb_attribute_list_t *pm = esp_zb_pressure_meas_cluster_create(&pm_cfg);
        esp_zb_cluster_list_add_pressure_meas_cluster(cl, pm,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint        = EP_SYSTOLIC,
            .app_profile_id  = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id   = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0,
        };
        esp_zb_ep_list_add_ep(ep_list, cl, ep_cfg);
    }

    /* ---- EP 2: PressureMeasurement (Diastolic) ---- */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        esp_zb_pressure_meas_cluster_cfg_t pm_cfg = {
            .measured_value = 0,
            .min_value      = 53,   /* ~40 mmHg */
            .max_value      = 267,  /* ~200 mmHg */
        };
        esp_zb_attribute_list_t *pm = esp_zb_pressure_meas_cluster_create(&pm_cfg);
        esp_zb_cluster_list_add_pressure_meas_cluster(cl, pm,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint        = EP_DIASTOLIC,
            .app_profile_id  = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id   = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0,
        };
        esp_zb_ep_list_add_ep(ep_list, cl, ep_cfg);
    }

    /* ---- EP 3: Analog Input (Pulse Rate) ---- */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        esp_zb_analog_input_cluster_cfg_t ai_cfg = {
            .present_value  = 0.0f,
            .out_of_service = false,
            .status_flags   = 0,
        };
        esp_zb_attribute_list_t *ai = esp_zb_analog_input_cluster_create(&ai_cfg);
        esp_zb_analog_input_cluster_add_attr(ai,
            ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID, "\x0a" "Pulse Rate");
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

/* ------------------------------------------------------------------ */
/* Attribute update                                                     */
/* ------------------------------------------------------------------ */

/* EP1 / EP2: PressureMeasurement — store mmHg directly as int16.
 * ZHA labels the unit as hPa but the number is the correct mmHg value.
 * In the HA UI, change the entity unit to mmHg without letting HA convert
 * (the value is already in mmHg). */
static void set_pressure_value(uint8_t endpoint, float mmhg)
{
    int16_t val = (int16_t)roundf(mmhg);
    esp_zb_zcl_set_attribute_val(
        endpoint,
        ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID,
        &val,
        false);
}

/* EP3: AnalogInput — pulse rate, not consumed by ZHA, sent for completeness. */
static void set_pulse_value(float bpm)
{
    esp_zb_zcl_set_attribute_val(
        EP_PULSE,
        ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
        &bpm,
        false);
}

/* ------------------------------------------------------------------ */
/* Zigbee stack callbacks                                               */
/* ------------------------------------------------------------------ */

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

/* ------------------------------------------------------------------ */
/* Zigbee task                                                          */
/* ------------------------------------------------------------------ */

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

    /* Configure reporting for EP1/EP2 (PressureMeasurement) */
    uint8_t pressure_eps[] = { EP_SYSTOLIC, EP_DIASTOLIC };
    for (int i = 0; i < 2; i++) {
        esp_zb_zcl_reporting_info_t rpt = {
            .direction    = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
            .ep           = pressure_eps[i],
            .cluster_id   = ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
            .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            .attr_id      = ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID,
            .u.send_info = {
                .min_interval     = 0,
                .max_interval     = 300,
                .delta            = { .u16 = 1 }, /* 1 hPa ≈ 0.75 mmHg */
                .def_min_interval = 0,
                .def_max_interval = 300,
            },
            .dst = {
                .short_addr = 0x0000,
                .endpoint   = 1,
                .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            },
            .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        };
        esp_err_t err = esp_zb_zcl_update_reporting_info(&rpt);
        ESP_LOGI(TAG, "reporting config ep=%d: %s", pressure_eps[i], esp_err_to_name(err));
    }

    /* Configure reporting for EP3 (AnalogInput, pulse rate) */
    {
        esp_zb_zcl_reporting_info_t rpt = {
            .direction    = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
            .ep           = EP_PULSE,
            .cluster_id   = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
            .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            .attr_id      = ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
            .u.send_info = {
                .min_interval     = 0,
                .max_interval     = 300,
                .delta            = { .f32 = 1.0f },
                .def_min_interval = 0,
                .def_max_interval = 300,
            },
            .dst = {
                .short_addr = 0x0000,
                .endpoint   = 1,
                .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            },
            .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        };
        esp_err_t err = esp_zb_zcl_update_reporting_info(&rpt);
        ESP_LOGI(TAG, "reporting config ep=%d (pulse): %s", EP_PULSE, esp_err_to_name(err));
    }

    esp_zb_set_primary_network_channel_set(ZB_CHANNEL_MASK);


    ESP_ERROR_CHECK(esp_zb_start(false));

    /* Main Zigbee loop - drains the measurement queue.
     * BM54 sends all stored records in chronological order. We update
     * the attributes for each one; the last value (most recent measurement)
     * is what ZHA will read. One data point per BLE session in HA history. */
    while (1) {
        esp_zb_stack_main_loop_iteration();

        bp_measurement_t meas;
        if (xQueueReceive(s_meas_queue, &meas, 0) == pdTRUE) {
            ESP_LOGI(TAG, "updating Zigbee attributes: %.0f/%.0f mmHg, pulse %.0f bpm",
                     meas.systolic, meas.diastolic, meas.pulse);

            esp_zb_lock_acquire(portMAX_DELAY);
            set_pressure_value(EP_SYSTOLIC,  meas.systolic);
            set_pressure_value(EP_DIASTOLIC, meas.diastolic);
            if (meas.has_pulse) {
                set_pulse_value(meas.pulse);
            }
            esp_zb_lock_release();
        }
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

void zigbee_bp_device_start(void)
{
    s_meas_queue = xQueueCreate(1, sizeof(bp_measurement_t));
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
        xQueueOverwrite(s_meas_queue, meas);
    }
}

void zigbee_bp_set_ble_connecting(bool ble_connecting)
{
    /*
     * During BLE connection: set ZED long-poll to 60s to minimise
     * 802.15.4 activity that interferes with BLE handshake.
     * Normal operation: restore to 7.5s default.
     */
    uint32_t interval_ms = ble_connecting ? 60000 : 7500;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_set_default_long_poll_interval(interval_ms);
    esp_zb_lock_release();
}
