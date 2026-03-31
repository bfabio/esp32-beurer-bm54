#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float systolic;   /* mmHg */
    float diastolic;  /* mmHg */
    float map;        /* mmHg - mean arterial pressure */
    float pulse;      /* bpm  */
    bool  has_pulse;
    bool  valid;
} bp_measurement_t;

/*
 * Parse a raw Blood Pressure Measurement characteristic value (UUID 0x2A35).
 * Returns true and fills *out on success.
 * Values in kPa are converted to mmHg automatically.
 */
bool bp_parse(const uint8_t *data, uint16_t len, bp_measurement_t *out);
