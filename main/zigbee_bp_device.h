#pragma once

#include "bp_parser.h"

/*
 * Start the Zigbee end-device task.
 * The device joins the network and exposes three Analog Input endpoints:
 *   EP 1 - Systolic   (mmHg)
 *   EP 2 - Diastolic  (mmHg)
 *   EP 3 - Pulse Rate (bpm)
 */
/*
 * If factory_new is true, Zigbee NVRAM is erased at startup so the device
 * enters network steering as if freshly provisioned.
 */
void zigbee_bp_device_start(bool factory_new);

/*
 * Update Zigbee attributes from a new BP measurement.
 * Thread-safe: can be called from the NimBLE task.
 */
void zigbee_bp_device_update(const bp_measurement_t *meas);

/*
 * Temporarily increase the ZED long-poll interval to reduce 802.15.4
 * interference during BLE connection attempts.
 * Call ble_connecting=true before connecting, false after disconnect.
 */
void zigbee_bp_set_ble_connecting(bool ble_connecting);
