#include "thermocouple.h"

// MAX31855 word layout (datasheet, "Memory Map"):
//   D31..D18  14-bit signed thermocouple temperature, 0.25 C per LSB
//   D16       fault (any of D2..D0)
//   D15..D4   12-bit signed internal temperature, 0.0625 C per LSB
//   D2 SCV    short to VCC
//   D1 SCG    short to GND
//   D0 OC     open circuit
tc_reading_t tc_decode_max31855(uint32_t raw)
{
    tc_reading_t r = {0.0f, 0.0f, TC_OK};

    if (raw == 0x00000000u || raw == 0xFFFFFFFFu) {
        r.faults = TC_FAULT_BUS;
        return r;
    }
    if (raw & (1u << 16)) {
        if (raw & (1u << 0)) r.faults |= TC_FAULT_OPEN;
        if (raw & (1u << 1)) r.faults |= TC_FAULT_SHORT_GND;
        if (raw & (1u << 2)) r.faults |= TC_FAULT_SHORT_VCC;
        if (r.faults == TC_OK) r.faults = TC_FAULT_BUS;  // fault bit without a reason: treat as bad read
    }

    int32_t tc = (int32_t)(raw >> 18) & 0x3FFF;
    if (tc & 0x2000) tc -= 0x4000;            // sign-extend 14 bits
    int32_t in = (int32_t)(raw >> 4) & 0x0FFF;
    if (in & 0x0800) in -= 0x1000;            // sign-extend 12 bits

    r.temp_c = (float)tc * 0.25f;
    r.internal_c = (float)in * 0.0625f;
    return r;
}

// MAX6675 word layout (datasheet, Figure 2):
//   D15       dummy sign bit, always 0
//   D14..D3   12-bit unsigned temperature, 0.25 C per LSB
//   D2        thermocouple input open
//   D1        device ID, always 0
tc_reading_t tc_decode_max6675(uint16_t raw)
{
    tc_reading_t r = {0.0f, 0.0f, TC_OK};

    if (raw == 0xFFFFu || (raw & 0x8002u)) {  // bits that must be 0 are set: no device / bad wiring
        r.faults = TC_FAULT_BUS;
        return r;
    }
    if (raw & (1u << 2)) {
        r.faults = TC_FAULT_OPEN;
        return r;
    }
    r.temp_c = (float)((raw >> 3) & 0x0FFF) * 0.25f;
    return r;
}
