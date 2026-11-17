// Latched safety supervisor for the heater. Pure C, unit-tested on the host.
// Once tripped, the heater stays off until the board is reset.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TRIP_NONE = 0,
    TRIP_SENSOR,     // SENSOR_FAULT_LIMIT consecutive bad thermocouple reads
    TRIP_OVERTEMP,   // reading at or above the cutoff
    TRIP_RUNAWAY,    // heater near full power but the reading does not rise: TC off the heater
    TRIP_MAX_RUN,    // maximum run time reached (normal end of a test)
} trip_t;

typedef struct {
    // limits
    float cutoff_c;
    uint8_t fault_limit;
    float runaway_duty_pct;
    uint32_t runaway_window_ms;
    float runaway_min_rise_c;
    uint32_t max_run_ms;
    // state
    trip_t trip;
    uint8_t bad_reads;
    bool high_duty;          // currently in a high-duty stretch
    uint32_t high_since_ms;
    float high_start_c;
} safety_t;

void safety_init(safety_t *s, float cutoff_c, uint8_t fault_limit, float runaway_duty_pct,
                 uint32_t runaway_window_ms, float runaway_min_rise_c, uint32_t max_run_ms);

// Call once per sample. sensor_ok=false means temp_c is not valid.
// duty_pct is the duty that was applied during the previous period.
// heat_ms is the time since the heating phase started (0 during the baseline).
// Returns the (latched) trip reason.
trip_t safety_update(safety_t *s, bool sensor_ok, float temp_c, float duty_pct,
                     uint32_t now_ms, uint32_t heat_ms);

const char *safety_trip_name(trip_t t);

#ifdef __cplusplus
}
#endif
