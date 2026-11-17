#include "frame.h"

#include <stdio.h>

uint8_t frame_crc8(const char *data)
{
    uint8_t crc = 0;
    while (*data) {
        crc ^= (uint8_t)*data++;
        for (uint8_t bit = 0; bit < 8; bit++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

static long round_scaled(float x, float scale)
{
    float y = x * scale;
    return (long)(y >= 0.0f ? y + 0.5f : y - 0.5f);
}

size_t frame_build(char *buf, size_t n, float temp_c, float duty_pct)
{
    // same limits as TEMP_RANGE_C / HUMIDITY_RANGE_PCT in sensor-data-logger/src/sources.py
    if (!(temp_c >= -40.0f && temp_c <= 85.0f)) return 0;     // also rejects NaN
    if (!(duty_pct >= 0.0f && duty_pct <= 100.0f)) return 0;

    long t = round_scaled(temp_c, 100.0f);   // centi-degrees
    long d = round_scaled(duty_pct, 10.0f);  // tenths of a percent
    const char *sign = t < 0 ? "-" : "";
    long ta = t < 0 ? -t : t;

    char payload[24];
    int len = snprintf(payload, sizeof payload, "%s%ld.%02ld,%ld.%ld", sign, ta / 100, ta % 100, d / 10, d % 10);
    if (len <= 0 || (size_t)len >= sizeof payload) return 0;

    int out = snprintf(buf, n, "$%s*%02X", payload, frame_crc8(payload));
    if (out <= 0 || (size_t)out >= n) return 0;
    return (size_t)out;
}
