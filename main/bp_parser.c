#include "bp_parser.h"

#include <math.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "bp_parser";

/*
 * Decode an IEEE 11073-20601 SFLOAT (16-bit float).
 *
 * Format: [15:12] exponent (signed 4-bit), [11:0] mantissa (signed 12-bit)
 * Value  = mantissa * 10^exponent
 *
 * Special values defined by the spec:
 *   0x07FF  NaN
 *   0x0800  NRes  (not at this resolution)
 *   0x07FE  +INFINITY
 *   0x0802  -INFINITY
 *   0x0801  Reserved
 */
static float sfloat_decode(uint16_t raw)
{
    switch (raw) {
    case 0x07FF: return NAN;
    case 0x0800: return NAN;
    case 0x07FE: return INFINITY;
    case 0x0802: return -INFINITY;
    default:     break;
    }

    int16_t mantissa = (int16_t)(raw & 0x0FFF);
    int8_t  exponent = (int8_t) (raw >> 12);

    /* sign-extend 12-bit mantissa */
    if (mantissa & 0x0800) {
        mantissa |= (int16_t)0xF000;
    }
    /* sign-extend 4-bit exponent */
    if (exponent & 0x08) {
        exponent |= (int8_t)0xF0;
    }

    return (float)mantissa * powf(10.0f, (float)exponent);
}

bool bp_parse(const uint8_t *data, uint16_t len, bp_measurement_t *out)
{
    if (!data || !out || len < 7) {
        ESP_LOGW(TAG, "invalid args or too short (%u bytes)", len);
        return false;
    }

    memset(out, 0, sizeof(*out));

    uint8_t flags = data[0];
    bool kpa          = (flags & 0x01) != 0;
    bool has_ts       = (flags & 0x02) != 0;
    bool has_pulse    = (flags & 0x04) != 0;

    uint16_t sys_raw  = (uint16_t)(data[1] | ((uint16_t)data[2] << 8));
    uint16_t dia_raw  = (uint16_t)(data[3] | ((uint16_t)data[4] << 8));
    uint16_t map_raw  = (uint16_t)(data[5] | ((uint16_t)data[6] << 8));

    out->systolic  = sfloat_decode(sys_raw);
    out->diastolic = sfloat_decode(dia_raw);
    out->map       = sfloat_decode(map_raw);

    if (kpa) {
        /* 1 kPa = 7.50062 mmHg */
        out->systolic  *= 7.50062f;
        out->diastolic *= 7.50062f;
        out->map       *= 7.50062f;
    }

    if (isnan(out->systolic) || isnan(out->diastolic)) {
        ESP_LOGW(TAG, "NaN in systolic/diastolic - measurement not ready?");
        return false;
    }

    int offset = 7;

    if (has_ts) {
        /* 7 bytes: year(2) + month(1) + day(1) + hour(1) + min(1) + sec(1) */
        if (len < (uint16_t)(offset + 7)) {
            ESP_LOGW(TAG, "too short for timestamp");
            return false;
        }
        /* timestamp parsing skipped - not needed for sensor reporting */
        offset += 7;
    }

    out->has_pulse = has_pulse;
    if (has_pulse) {
        if (len < (uint16_t)(offset + 2)) {
            ESP_LOGW(TAG, "too short for pulse rate");
            return false;
        }
        uint16_t pulse_raw = (uint16_t)(data[offset] | ((uint16_t)data[offset + 1] << 8));
        out->pulse = sfloat_decode(pulse_raw);
    }

    out->valid = true;

    ESP_LOGI(TAG, "systolic=%.0f diastolic=%.0f MAP=%.0f pulse=%.0f bpm (flags=0x%02x)",
             out->systolic, out->diastolic, out->map,
             out->has_pulse ? out->pulse : 0.0f, flags);

    return true;
}
