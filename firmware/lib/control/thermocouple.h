// Decoding of MAX31855 / MAX6675 raw SPI words. Pure C, no hardware access, unit-tested on the host.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TC_OK = 0,
    TC_FAULT_OPEN = 1 << 0,      // thermocouple not connected
    TC_FAULT_SHORT_GND = 1 << 1, // MAX31855 only
    TC_FAULT_SHORT_VCC = 1 << 2, // MAX31855 only
    TC_FAULT_BUS = 1 << 3,       // all-zeros or all-ones word: board missing or SPI wiring problem
} tc_fault_t;

typedef struct {
    float temp_c;          // thermocouple (hot junction) temperature
    float internal_c;      // cold-junction temperature (MAX31855 only, else NAN-free 0)
    uint8_t faults;        // OR of tc_fault_t
} tc_reading_t;

// MAX31855: 32-bit word, MSB first.
tc_reading_t tc_decode_max31855(uint32_t raw);

// MAX6675: 16-bit word, MSB first.
tc_reading_t tc_decode_max6675(uint16_t raw);

static inline bool tc_ok(const tc_reading_t *r) { return r->faults == TC_OK; }

#ifdef __cplusplus
}
#endif
