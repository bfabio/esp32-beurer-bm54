#include "ble_bp_client.h"

#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_sm.h"
#include "host/util/util.h"
#include "bp_parser.h"
#include "led_status.h"
#include "zigbee_bp_device.h"

static const char *TAG = "ble_bp";

/* BM54 Bluetooth address (little-endian byte order for NimBLE) */
#define BM54_ADDR_TYPE  BLE_ADDR_PUBLIC
static const uint8_t BM54_ADDR[6] = { 0xA1, 0x43, 0x73, 0xED, 0x7F, 0x0C };

/* Blood Pressure service / characteristic UUIDs */
static const ble_uuid16_t s_bp_svc_uuid  = BLE_UUID16_INIT(0x1810);
static const ble_uuid16_t s_bp_meas_uuid = BLE_UUID16_INIT(0x2A35);
static const ble_uuid16_t s_cccd_uuid    = BLE_UUID16_INIT(0x2902);

/* GATT discovery state */
static uint16_t s_conn_handle  = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_svc_end_hdl  = 0;
static uint16_t s_meas_val_hdl = 0; /* characteristic value handle  */
static uint16_t s_cccd_hdl     = 0;

static bp_measurement_cb_t s_cb = NULL;

/* ------------------------------------------------------------------ */
/* Forward declarations                                                 */
/* ------------------------------------------------------------------ */
static void     start_scan(void);
static int      gap_event_cb(struct ble_gap_event *event, void *arg);
static int      disc_svc_cb(uint16_t conn_handle,
                             const struct ble_gatt_error *error,
                             const struct ble_gatt_svc   *service,
                             void *arg);
static int      disc_chr_cb(uint16_t conn_handle,
                             const struct ble_gatt_error *error,
                             const struct ble_gatt_chr   *chr,
                             void *arg);
static int      disc_dsc_cb(uint16_t conn_handle,
                             const struct ble_gatt_error *error,
                             uint16_t chr_def_handle,
                             const struct ble_gatt_dsc   *dsc,
                             void *arg);
static void     write_cccd_indicate(void);
static int      write_cccd_cb(uint16_t conn_handle,
                               const struct ble_gatt_error *error,
                               struct ble_gatt_attr *attr,
                               void *arg);

/* ------------------------------------------------------------------ */
/* Passkey entry (MITM bonding)                                        */
/* ------------------------------------------------------------------ */

/*
 * Runs in a dedicated task so the NimBLE host task is not blocked while
 * waiting for UART input.  The user types the 6-digit passkey displayed
 * on the BM54 screen and presses Enter.
 */
static void passkey_entry_task(void *arg)
{
    uint16_t conn_handle = (uint16_t)(uintptr_t)arg;
    char buf[8];
    int  idx = 0;
    int  c;

    ESP_LOGI(TAG, ">>> Type the 6-digit passkey shown on BM54, then Enter:");

    while (idx < 6) {
        c = getchar();
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (c >= '0' && c <= '9') {
            buf[idx++] = (char)c;
            putchar(c);
            fflush(stdout);
        }
    }
    buf[6] = '\0';
    putchar('\n');

    struct ble_sm_io io = {
        .action  = BLE_SM_IOACT_INPUT,
        .passkey = (uint32_t)atoi(buf),
    };
    ESP_LOGI(TAG, "injecting passkey %06" PRIu32, io.passkey);
    int rc = ble_sm_inject_io(conn_handle, &io);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_sm_inject_io failed: %d", rc);
    }
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/* Helpers                                                              */
/* ------------------------------------------------------------------ */

static bool addr_is_target(const ble_addr_t *addr)
{
    return addr->type == BM54_ADDR_TYPE &&
           memcmp(addr->val, BM54_ADDR, sizeof(BM54_ADDR)) == 0;
}

/* ------------------------------------------------------------------ */
/* Scan                                                                 */
/* ------------------------------------------------------------------ */

static int scan_event_cb(struct ble_gap_event *event, void *arg)
{
    if (event->type != BLE_GAP_EVENT_DISC) {
        return 0;
    }

    if (!addr_is_target(&event->disc.addr)) {
        return 0;
    }

    ESP_LOGI(TAG, "BM54 found, connecting");
    ble_gap_disc_cancel();

    /* Reduce 802.15.4 activity during the connect+bond cycle */
    zigbee_bp_set_ble_connecting(true);

    ble_addr_t peer = event->disc.addr;
    int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &peer,
                             5000, NULL, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "connect failed: %d", rc);
        start_scan();
    }
    return 0;
}

static void start_scan(void)
{
    struct ble_gap_disc_params params = {
        .passive       = 0,
        .filter_policy = BLE_HCI_SCAN_FILT_NO_WL,
        .itvl          = BLE_GAP_SCAN_ITVL_MS(160),
        .window        = BLE_GAP_SCAN_WIN_MS(80),
        .filter_duplicates = 1,
    };

    led_status_set(LED_READY);
    ESP_LOGI(TAG, "scanning for BM54...");
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &params,
                          scan_event_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "scan start failed: %d", rc);
    }
}

/* ------------------------------------------------------------------ */
/* GAP connection events                                                */
/* ------------------------------------------------------------------ */

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGE(TAG, "connect failed, status=%d", event->connect.status);
            zigbee_bp_set_ble_connecting(false);
            start_scan();
            break;
        }
        s_conn_handle = event->connect.conn_handle;
        led_status_set(LED_RECEIVING);
        ESP_LOGI(TAG, "connected, handle=%d", s_conn_handle);

        /* Trigger bonding / encryption */
        ble_gap_security_initiate(s_conn_handle);
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        led_status_set(LED_READY);
        zigbee_bp_set_ble_connecting(false);
        ESP_LOGW(TAG, "disconnected, reason=%d", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        /* Keep s_meas_val_hdl and s_cccd_hdl: skip rediscovery on next connection */
        vTaskDelay(pdMS_TO_TICKS(2000));
        start_scan();
        break;

    case BLE_GAP_EVENT_ENC_CHANGE: {
        ESP_LOGI(TAG, "ENC_CHANGE status=%d cccd_hdl=%d meas_hdl=%d",
                 event->enc_change.status, s_cccd_hdl, s_meas_val_hdl);
        if (event->enc_change.status == 0) {
            struct ble_gap_conn_desc desc;
            if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
                ESP_LOGI(TAG, "  sec: encrypted=%d authenticated=%d bonded=%d key_size=%d",
                         desc.sec_state.encrypted, desc.sec_state.authenticated,
                         desc.sec_state.bonded, desc.sec_state.key_size);
            }
        }
        if (event->enc_change.status != 0) {
            ESP_LOGE(TAG, "encryption/bonding failed: %d", event->enc_change.status);
            ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            break;
        }
        if (s_cccd_hdl != 0) {
            /* Handles already known from previous connection - skip discovery */
            ESP_LOGI(TAG, "bonded, reusing known handles (val=%d cccd=%d)",
                     s_meas_val_hdl, s_cccd_hdl);
            write_cccd_indicate();
        } else {
            ESP_LOGI(TAG, "bonded, discovering services");
            ble_gattc_disc_svc_by_uuid(s_conn_handle,
                                        (const ble_uuid_t *)&s_bp_svc_uuid,
                                        disc_svc_cb, NULL);
        }
        break;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
        uint16_t attr_handle = event->notify_rx.attr_handle;
        bool     indication  = event->notify_rx.indication;
        uint16_t len         = OS_MBUF_PKTLEN(event->notify_rx.om);
        ESP_LOGI(TAG, "NOTIFY_RX: handle=%d indication=%d len=%d (known meas_hdl=%d)",
                 attr_handle, indication, len, s_meas_val_hdl);

        /*
         * Accept the indication if:
         *  a) handle matches what we discovered, OR
         *  b) we haven't discovered yet but the payload looks like a BP measurement
         *     (≥7 bytes) — learn the handle on the fly.
         */
        bool handle_ok = (s_meas_val_hdl != 0)
                         ? (attr_handle == s_meas_val_hdl)
                         : (len >= 7);

        if (handle_ok) {
            if (s_meas_val_hdl == 0) {
                ESP_LOGI(TAG, "learning meas handle = %d from first indication", attr_handle);
                s_meas_val_hdl = attr_handle;
            }
            uint8_t buf[32];
            if (len > sizeof(buf)) len = sizeof(buf);
            os_mbuf_copydata(event->notify_rx.om, 0, len, buf);

            bp_measurement_t meas;
            if (bp_parse(buf, len, &meas) && s_cb) {
                s_cb(&meas);
            }
        }
        break;
    }

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        ESP_LOGI(TAG, "PASSKEY_ACTION action=%d", event->passkey.params.action);
        if (event->passkey.params.action == BLE_SM_IOACT_INPUT) {
            xTaskCreate(passkey_entry_task, "passkey",
                        2048, (void *)(uintptr_t)event->passkey.conn_handle,
                        5, NULL);
        }
        break;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        /* Bond already exists; delete old one and re-pair */
        struct ble_gap_conn_desc desc;
        ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        ble_store_util_delete_peer(&desc.peer_id_addr);
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    default:
        break;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* GATT discovery                                                       */
/* ------------------------------------------------------------------ */

static int disc_svc_cb(uint16_t conn_handle,
                        const struct ble_gatt_error *error,
                        const struct ble_gatt_svc *service,
                        void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        if (s_svc_end_hdl == 0) {
            ESP_LOGE(TAG, "Blood Pressure service not found");
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;
    }
    if (error->status != 0) {
        ESP_LOGE(TAG, "service discovery error: %d", error->status);
        return 0;
    }

    ESP_LOGI(TAG, "Blood Pressure service found, handles %d-%d",
             service->start_handle, service->end_handle);
    s_svc_end_hdl = service->end_handle;

    ble_gattc_disc_chrs_by_uuid(conn_handle,
                                 service->start_handle,
                                 service->end_handle,
                                 (const ble_uuid_t *)&s_bp_meas_uuid,
                                 disc_chr_cb, NULL);
    return 0;
}

static int disc_chr_cb(uint16_t conn_handle,
                        const struct ble_gatt_error *error,
                        const struct ble_gatt_chr *chr,
                        void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        if (s_meas_val_hdl == 0) {
            ESP_LOGE(TAG, "BP Measurement characteristic not found");
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;
    }
    if (error->status != 0) {
        ESP_LOGE(TAG, "characteristic discovery error: %d", error->status);
        return 0;
    }

    ESP_LOGI(TAG, "BP Measurement found: def=%d val=%d",
             chr->def_handle, chr->val_handle);
    s_meas_val_hdl = chr->val_handle;

    /* Discover descriptors for this characteristic.
     * NimBLE starts at chr_val_handle + 1 internally. */
    ble_gattc_disc_all_dscs(conn_handle,
                             chr->val_handle,
                             s_svc_end_hdl,
                             disc_dsc_cb, NULL);
    return 0;
}

static int disc_dsc_cb(uint16_t conn_handle,
                        const struct ble_gatt_error *error,
                        uint16_t chr_def_handle,
                        const struct ble_gatt_dsc *dsc,
                        void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        if (s_cccd_hdl != 0) {
            write_cccd_indicate();
        } else {
            ESP_LOGE(TAG, "CCCD not found");
        }
        return 0;
    }
    if (error->status != 0) {
        ESP_LOGE(TAG, "descriptor discovery error: %d", error->status);
        return 0;
    }

    if (ble_uuid_cmp(&dsc->uuid.u, &s_cccd_uuid.u) == 0 && s_cccd_hdl == 0) {
        ESP_LOGI(TAG, "CCCD found at handle %d", dsc->handle);
        s_cccd_hdl = dsc->handle;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Enable indications on 0x2A35                                         */
/* ------------------------------------------------------------------ */

static void write_cccd_indicate(void)
{
    /* CCCD value 0x0002 enables indications */
    static const uint8_t val[2] = { 0x02, 0x00 };

    ESP_LOGI(TAG, "enabling indications on BP Measurement");
    int rc = ble_gattc_write_flat(s_conn_handle, s_cccd_hdl,
                                   val, sizeof(val),
                                   write_cccd_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "CCCD write failed: %d", rc);
    }
}

static int write_cccd_cb(uint16_t conn_handle,
                          const struct ble_gatt_error *error,
                          struct ble_gatt_attr *attr,
                          void *arg)
{
    if (error->status != 0) {
        ESP_LOGE(TAG, "CCCD write error: %d", error->status);
        return 0;
    }
    ESP_LOGI(TAG, "indications enabled - waiting for BM54 measurement");
    return 0;
}

/* ------------------------------------------------------------------ */
/* NimBLE host task                                                     */
/* ------------------------------------------------------------------ */

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE reset, reason=%d", reason);
}

static void on_sync(void)
{
    /* Verify the controller address */
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
    start_scan();
}

static void nimble_host_task(void *arg)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();          /* blocks until nimble_port_stop() */
    nimble_port_freertos_deinit();
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

void ble_bp_client_init(bp_measurement_cb_t cb)
{
    s_cb = cb;
}

void ble_bp_client_start(void)
{
    nimble_port_init();

    ble_hs_cfg.reset_cb  = on_reset;
    ble_hs_cfg.sync_cb   = on_sync;

    /* Passkey Entry bonding - BM54 displays passkey, we type it */
    ble_hs_cfg.sm_io_cap  = BLE_HS_IO_KEYBOARD_ONLY;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm    = 1;
    ble_hs_cfg.sm_sc      = 1;
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    /* Persist bond keys in NVS */
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    nimble_port_freertos_init(nimble_host_task);
}
