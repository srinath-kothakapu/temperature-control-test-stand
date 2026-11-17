#include "safety.h"

void safety_init(safety_t *s, float cutoff_c, uint8_t fault_limit, float runaway_duty_pct,
                 uint32_t runaway_window_ms, float runaway_min_rise_c, uint32_t max_run_ms)
{
    s->cutoff_c = cutoff_c;
    s->fault_limit = fault_limit;
    s->runaway_duty_pct = runaway_duty_pct;
    s->runaway_window_ms = runaway_window_ms;
    s->runaway_min_rise_c = runaway_min_rise_c;
    s->max_run_ms = max_run_ms;
    s->trip = TRIP_NONE;
    s->bad_reads = 0;
    s->high_duty = false;
    s->high_since_ms = 0;
    s->high_start_c = 0.0f;
}

static trip_t latch(safety_t *s, trip_t t)
{
    if (s->trip == TRIP_NONE) s->trip = t;
    return s->trip;
}

trip_t safety_update(safety_t *s, bool sensor_ok, float temp_c, float duty_pct,
                     uint32_t now_ms, uint32_t heat_ms)
{
    if (s->trip != TRIP_NONE) return s->trip;

    if (!sensor_ok) {
        if (s->bad_reads < 255) s->bad_reads++;
        if (s->bad_reads >= s->fault_limit) return latch(s, TRIP_SENSOR);
        return TRIP_NONE;  // a single glitch: caller holds the heater off for this sample
    }
    s->bad_reads = 0;

    if (temp_c >= s->cutoff_c) return latch(s, TRIP_OVERTEMP);

    // Runaway check: a long stretch of high duty must produce a temperature rise.
    if (duty_pct >= s->runaway_duty_pct) {
        if (!s->high_duty) {
            s->high_duty = true;
            s->high_since_ms = now_ms;
            s->high_start_c = temp_c;
        } else if (now_ms - s->high_since_ms >= s->runaway_window_ms) {
            if (temp_c - s->high_start_c < s->runaway_min_rise_c) return latch(s, TRIP_RUNAWAY);
            // it is heating: start a new window from here
            s->high_since_ms = now_ms;
            s->high_start_c = temp_c;
        }
    } else {
        s->high_duty = false;
    }

    if (heat_ms >= s->max_run_ms) return latch(s, TRIP_MAX_RUN);
    return TRIP_NONE;
}

const char *safety_trip_name(trip_t t)
{
    switch (t) {
    case TRIP_NONE: return "NONE";
    case TRIP_SENSOR: return "SENSOR";
    case TRIP_OVERTEMP: return "OVERTEMP";
    case TRIP_RUNAWAY: return "RUNAWAY";
    case TRIP_MAX_RUN: return "MAX_RUN";
    }
    return "?";
}
