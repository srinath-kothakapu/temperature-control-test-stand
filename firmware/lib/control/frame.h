// UART frame for the sensor-data-logger toolkit:
//   $<temperature_c>,<duty_pct>*<CRC>\n
// The logger's second column is named humidity_pct; on this stand it carries heater duty (0..100 %).
// CRC-8, polynomial 0x07, initial value 0, over the text between '$' and '*'.
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t frame_crc8(const char *data);

// Writes "$T,D*CC" (no newline) into buf. Temperature with 2 decimals, duty with 1 decimal.
// Formats with integer arithmetic, so it does not need printf float support.
// Returns the length, or 0 if the values are outside the logger's accepted range
// (-40..85 C, 0..100 %) or buf is too small.
size_t frame_build(char *buf, size_t n, float temp_c, float duty_pct);

#ifdef __cplusplus
}
#endif
