#pragma once

#include "bp_parser.h"

/* Called from NimBLE task context whenever a measurement indication arrives. */
typedef void (*bp_measurement_cb_t)(const bp_measurement_t *meas);

/*
 * Initialise the NimBLE stack and register the measurement callback.
 * Call once before ble_bp_client_start().
 */
void ble_bp_client_init(bp_measurement_cb_t cb);

/*
 * Start the NimBLE host task and begin scanning for the BM54.
 * Returns immediately; scanning/connection happen asynchronously.
 */
void ble_bp_client_start(void);
