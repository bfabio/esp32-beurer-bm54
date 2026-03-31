#pragma once

#include "bp_parser.h"

void zigbee_bp_device_start(void);
void zigbee_bp_device_update(const bp_measurement_t *meas);
void zigbee_bp_set_ble_connecting(bool ble_connecting);
