// Host tests for the firmware control library (gcc, no board needed).
//   make -C tests/c && ./tests/c/test_control
//
// Every test case has an ID and lists the requirement(s) from tests/requirements.csv that it
// verifies. For each case the program prints one line
//   RESULT,<case id>,<requirement ids separated by ;>,<PASS|FAIL>,<number of checks>
// which tools/trace.py reads to build docs/traceability.md.
//
// Levels: TC = unit test of one module, IT = integration test of modules together,
//         ST = system test of the whole control loop against a simulated plant.
//
// The ST cases use stand_step() below. It repeats the order of operations in loop() of
// firmware/src/main.cpp (read, safety supervisor, control, heater lock) with the real values from
// config.h. main.cpp itself needs the board and is NOT covered by these tests.
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "controller.h"
#include "frame.h"
#include "safety.h"
#include "thermocouple.h"

static int case_checks = 0, case_failures = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        case_checks++;                                                       \
        if (!(cond)) {                                                       \
            case_failures++;                                                 \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
        }                                                                    \
    } while (0)
#define CHECK_NEAR(a, b, tol) CHECK(fabs((double)(a) - (double)(b)) <= (tol))

// ================================================================ unit tests

static uint32_t max31855_word(int tc_quarters, int internal_sixteenths, unsigned fault_bits)
{
    uint32_t w = ((uint32_t)(tc_quarters & 0x3FFF) << 18) | ((uint32_t)(internal_sixteenths & 0x0FFF) << 4);
    if (fault_bits) w |= (1u << 16) | (fault_bits & 0x7u);
    return w;
}

static void tc_01_max31855_decoding(void)  // REQ-01
{
    // Examples from the MAX31855 datasheet temperature table
    tc_reading_t r = tc_decode_max31855(max31855_word(1600, 0, 0));  // +400.00 C
    CHECK(tc_ok(&r));
    CHECK_NEAR(r.temp_c, 400.0, 1e-6);

    r = tc_decode_max31855(max31855_word(100, 400, 0));  // +25.00 C, internal +25.0 C
    CHECK_NEAR(r.temp_c, 25.0, 1e-6);
    CHECK_NEAR(r.internal_c, 25.0, 1e-6);

    r = tc_decode_max31855(max31855_word(1, 0, 0));  // +0.25 C
    CHECK_NEAR(r.temp_c, 0.25, 1e-6);

    r = tc_decode_max31855(max31855_word(-1, -1, 0));  // -0.25 C, internal -0.0625 C
    CHECK(tc_ok(&r));
    CHECK_NEAR(r.temp_c, -0.25, 1e-6);
    CHECK_NEAR(r.internal_c, -0.0625, 1e-6);

    r = tc_decode_max31855(max31855_word(-1000, 0, 0));  // -250.00 C
    CHECK_NEAR(r.temp_c, -250.0, 1e-6);
}

static void tc_02_max6675_decoding(void)  // REQ-02
{
    tc_reading_t r = tc_decode_max6675((uint16_t)(100 << 3));  // 25.00 C
    CHECK(tc_ok(&r));
    CHECK_NEAR(r.temp_c, 25.0, 1e-6);
    r = tc_decode_max6675((uint16_t)(4095 << 3));  // 1023.75 C full scale
    CHECK_NEAR(r.temp_c, 1023.75, 1e-6);
}

static void tc_03_sensor_faults(void)  // REQ-03
{
    CHECK(tc_decode_max31855(max31855_word(100, 400, 0x1)).faults == TC_FAULT_OPEN);
    CHECK(tc_decode_max31855(max31855_word(100, 400, 0x2)).faults == TC_FAULT_SHORT_GND);
    CHECK(tc_decode_max31855(max31855_word(100, 400, 0x4)).faults == TC_FAULT_SHORT_VCC);
    CHECK(tc_decode_max31855(0x00000000u).faults == TC_FAULT_BUS);
    CHECK(tc_decode_max31855(0xFFFFFFFFu).faults == TC_FAULT_BUS);
    CHECK(tc_decode_max31855(1u << 16).faults == TC_FAULT_BUS);  // fault flag but no reason bit
    CHECK(tc_decode_max6675((uint16_t)((100 << 3) | 0x4)).faults == TC_FAULT_OPEN);
    CHECK(tc_decode_max6675(0xFFFF).faults == TC_FAULT_BUS);
    CHECK(tc_decode_max6675(0x8000 | (100 << 3)).faults == TC_FAULT_BUS);
    // a faulty read must never look like a valid one
    tc_reading_t r = tc_decode_max31855(max31855_word(100, 400, 0x1));
    CHECK(!tc_ok(&r));
}

static void tc_04_frame_format(void)  // REQ-04
{
    // CRC-8/0x07 standard check value
    CHECK(frame_crc8("123456789") == 0xF4);
    // Example frame from the sensor-data-logger README
    CHECK(frame_crc8("23.45,41.2") == 0xBE);

    char buf[40];
    CHECK(frame_build(buf, sizeof buf, 23.45f, 41.2f) > 0);
    CHECK(strcmp(buf, "$23.45,41.2*BE") == 0);

    frame_build(buf, sizeof buf, 50.0f, 100.0f);
    CHECK(strncmp(buf, "$50.00,100.0*", 13) == 0);
    frame_build(buf, sizeof buf, -0.25f, 0.0f);
    CHECK(strncmp(buf, "$-0.25,0.0*", 11) == 0);
    frame_build(buf, sizeof buf, 7.0f, 3.25f);  // rounds 3.25 -> 3.3 (half away from zero)
    CHECK(strncmp(buf, "$7.00,3.3*", 10) == 0);
}

static void tc_05_frame_range(void)  // REQ-05
{
    char buf[40];
    CHECK(frame_build(buf, sizeof buf, 85.5f, 0.0f) == 0);   // above logger range
    CHECK(frame_build(buf, sizeof buf, -41.0f, 0.0f) == 0);
    CHECK(frame_build(buf, sizeof buf, 20.0f, 100.5f) == 0);
    CHECK(frame_build(buf, sizeof buf, 20.0f, -0.5f) == 0);
    CHECK(frame_build(buf, sizeof buf, NAN, 0.0f) == 0);
    CHECK(frame_build(buf, sizeof buf, 20.0f, NAN) == 0);
    CHECK(frame_build(buf, 8, 20.0f, 10.0f) == 0);           // buffer too small
    CHECK(frame_build(buf, sizeof buf, 85.0f, 100.0f) > 0);  // both limits are inclusive
    CHECK(frame_build(buf, sizeof buf, -40.0f, 0.0f) > 0);
}

static void tc_06_onoff_hysteresis(void)  // REQ-06
{
    onoff_t c;
    onoff_init(&c, 0.5f, 100.0f);
    CHECK(onoff_update(&c, 50.0f, 40.0f) == 100.0f);
    CHECK(onoff_update(&c, 50.0f, 50.2f) == 100.0f);  // in band, keeps heating
    CHECK(onoff_update(&c, 50.0f, 50.6f) == 0.0f);    // above band, off
    CHECK(onoff_update(&c, 50.0f, 49.8f) == 0.0f);    // in band, stays off
    CHECK(onoff_update(&c, 50.0f, 49.4f) == 100.0f);  // below band, on again
}

static void tc_07_pid_terms(void)  // REQ-07
{
    pid_ctrl_t c;
    pid_init(&c, 2.0f, 0.5f, 0.0f, 1.0f, 0.0f, 100.0f);
    float u = pid_update(&c, 50.0f, 45.0f, 1.0f);  // e = 5: P = 10, I = 2.5
    CHECK_NEAR(u, 12.5, 1e-5);
    u = pid_update(&c, 50.0f, 45.0f, 1.0f);        // I = 5.0
    CHECK_NEAR(u, 15.0, 1e-5);

    // Derivative on measurement: rising temperature reduces output, setpoint jump does not kick
    pid_init(&c, 0.0f, 0.0f, 10.0f, 1.0f, -100.0f, 100.0f);
    pid_update(&c, 50.0f, 40.0f, 1.0f);
    u = pid_update(&c, 50.0f, 41.0f, 1.0f);        // dT/dt = 1 C/s -> D = -10
    CHECK_NEAR(u, -10.0, 1e-5);
    u = pid_update(&c, 80.0f, 41.0f, 1.0f);        // setpoint step, temperature flat -> D = 0
    CHECK_NEAR(u, 0.0, 1e-5);
}

static void tc_08_pid_antiwindup(void)  // REQ-08
{
    pid_ctrl_t c;
    pid_init(&c, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 100.0f);
    // Large error for a long time: output saturates, integral must not keep growing
    for (int i = 0; i < 1000; i++) pid_update(&c, 100.0f, 0.0f, 1.0f);
    CHECK(c.integral <= 100.0f);
    float i_sat = c.integral;
    for (int i = 0; i < 1000; i++) pid_update(&c, 100.0f, 0.0f, 1.0f);
    CHECK_NEAR(c.integral, i_sat, 1e-4);
    // As soon as the error flips sign, the output comes off the limit
    float u = pid_update(&c, 100.0f, 101.0f, 1.0f);
    CHECK(u < 100.0f);

    // P alone saturates the output. The integral must stay near 0 meanwhile, so when the error
    // becomes small the output drops at once. (Without conditional integration the integral would
    // sit at its limit, because the separate clamp only caps it at out_max, and the output would
    // stay at 100 % long after the error is gone.)
    pid_init(&c, 10.0f, 0.5f, 0.0f, 1.0f, 0.0f, 100.0f);
    for (int i = 0; i < 1000; i++) pid_update(&c, 100.0f, 0.0f, 1.0f);  // error 100 C, P = 1000 %
    CHECK(c.integral < 1.0f);
    u = pid_update(&c, 100.0f, 99.0f, 1.0f);                             // error 1 C
    CHECK(u < 20.0f);
}

static void tc_09_pid_output_limits(void)  // REQ-09
{
    pid_ctrl_t c;
    pid_init(&c, 8.0f, 0.05f, 2.0f, 0.2f, 0.0f, DUTY_MAX_PCT);
    float lo = 1e9f, hi = -1e9f;
    for (int i = 0; i < 400; i++) {                       // huge positive error, then huge negative
        float u = pid_update(&c, 1000.0f, 0.0f, 0.25f);
        if (u < lo) lo = u;
        if (u > hi) hi = u;
    }
    for (int i = 0; i < 400; i++) {
        float u = pid_update(&c, -1000.0f, 0.0f, 0.25f);
        if (u < lo) lo = u;
        if (u > hi) hi = u;
    }
    CHECK(lo >= 0.0f);
    CHECK(hi <= DUTY_MAX_PCT);
    CHECK(!isnan(lo) && !isnan(hi));
}

static void tc_10_overtemperature(void)  // REQ-10
{
    safety_t s;
    safety_init(&s, TEMP_CUTOFF_C, SENSOR_FAULT_LIMIT, RUNAWAY_DUTY_PCT,
                RUNAWAY_WINDOW_S * 1000u, RUNAWAY_MIN_RISE_C, MAX_RUN_S * 1000u);
    CHECK(safety_update(&s, true, TEMP_CUTOFF_C - 0.25f, 50, 0, 1000) == TRIP_NONE);  // one LSB below
    CHECK(safety_update(&s, true, TEMP_CUTOFF_C, 50, 250, 1250) == TRIP_OVERTEMP);    // at the limit
    CHECK(safety_update(&s, true, 25.0f, 0, 500, 1500) == TRIP_OVERTEMP);             // latched
    CHECK(safety_update(&s, true, 25.0f, 0, 750, 1750) == TRIP_OVERTEMP);
}

static void tc_11_sensor_fault_tolerance(void)  // REQ-11
{
    safety_t s;
    safety_init(&s, 80.0f, 3, 90.0f, 60000, 2.0f, 600000);
    CHECK(safety_update(&s, false, 0, 0, 0, 0) == TRIP_NONE);
    CHECK(safety_update(&s, false, 0, 0, 250, 0) == TRIP_NONE);
    CHECK(safety_update(&s, false, 0, 0, 500, 0) == TRIP_SENSOR);
    CHECK(safety_update(&s, true, 25, 0, 750, 0) == TRIP_SENSOR);  // stays tripped

    // a good read in between resets the counter
    safety_init(&s, 80.0f, 3, 90.0f, 60000, 2.0f, 600000);
    safety_update(&s, false, 0, 0, 0, 0);
    safety_update(&s, false, 0, 0, 250, 0);
    CHECK(safety_update(&s, true, 25, 0, 500, 0) == TRIP_NONE);
    CHECK(safety_update(&s, false, 0, 0, 750, 0) == TRIP_NONE);
}

static void tc_12_runaway(void)  // REQ-12
{
    safety_t s;
    // runaway: full power for 60 s with only 1 C rise
    safety_init(&s, 80.0f, 3, 90.0f, 60000, 2.0f, 600000);
    CHECK(safety_update(&s, true, 25.0f, 100, 0, 0) == TRIP_NONE);
    CHECK(safety_update(&s, true, 25.5f, 100, 30000, 30000) == TRIP_NONE);
    CHECK(safety_update(&s, true, 26.0f, 100, 60000, 60000) == TRIP_RUNAWAY);

    // full power with a real rise is fine; window restarts
    safety_init(&s, 80.0f, 3, 90.0f, 60000, 2.0f, 600000);
    safety_update(&s, true, 25.0f, 100, 0, 0);
    CHECK(safety_update(&s, true, 30.0f, 100, 60000, 60000) == TRIP_NONE);
    CHECK(safety_update(&s, true, 31.0f, 100, 120000, 120000) == TRIP_RUNAWAY);

    // dropping below the duty threshold resets the window
    safety_init(&s, 80.0f, 3, 90.0f, 60000, 2.0f, 600000);
    safety_update(&s, true, 25.0f, 100, 0, 0);
    safety_update(&s, true, 25.0f, 50, 30000, 30000);
    safety_update(&s, true, 25.0f, 100, 40000, 40000);
    CHECK(safety_update(&s, true, 25.5f, 100, 90000, 90000) == TRIP_NONE);

    // the duty threshold itself counts as "working hard" (>=), as in config.h
    safety_init(&s, 80.0f, 3, 90.0f, 60000, 2.0f, 600000);
    safety_update(&s, true, 25.0f, 90.0f, 0, 0);
    CHECK(safety_update(&s, true, 25.0f, 90.0f, 60000, 60000) == TRIP_RUNAWAY);
    // just below the threshold never trips
    safety_init(&s, 80.0f, 3, 90.0f, 60000, 2.0f, 600000);
    safety_update(&s, true, 25.0f, 89.9f, 0, 0);
    CHECK(safety_update(&s, true, 25.0f, 89.9f, 600000, 60000) == TRIP_NONE);
}

static void tc_13_max_run(void)  // REQ-13
{
    safety_t s;
    safety_init(&s, 80.0f, 3, 90.0f, 60000, 2.0f, 600000);
    CHECK(safety_update(&s, true, 40.0f, 30, 0, 599999) == TRIP_NONE);
    CHECK(safety_update(&s, true, 40.0f, 30, 250, 600000) == TRIP_MAX_RUN);
    CHECK(strcmp(safety_trip_name(TRIP_MAX_RUN), "MAX_RUN") == 0);
    CHECK(safety_update(&s, true, 40.0f, 30, 500, 600250) == TRIP_MAX_RUN);
}

// ================================================================ integration test

static void it_01_decode_to_frame(void)  // REQ-01, REQ-04
{
    // thermocouple word -> decoded temperature -> UART frame -> CRC recomputed from the payload
    tc_reading_t r = tc_decode_max31855(max31855_word(200, 400, 0));  // 50.00 C
    CHECK(tc_ok(&r));
    char buf[40];
    CHECK(frame_build(buf, sizeof buf, r.temp_c, 37.5f) > 0);
    CHECK(strncmp(buf, "$50.00,37.5*", 12) == 0);
    char *star = strchr(buf, '*');
    CHECK(star != NULL);
    if (star) {
        char payload[32];
        size_t n = (size_t)(star - buf) - 1;
        memcpy(payload, buf + 1, n);
        payload[n] = '\0';
        char crc[3];
        snprintf(crc, sizeof crc, "%02X", frame_crc8(payload));
        CHECK(strcmp(star + 1, crc) == 0);
    }
}

// ================================================================ system tests

typedef struct {
    pid_ctrl_t pid;
    safety_t safety;
    float applied_duty;
    int locked;
} stand_t;

static void stand_init(stand_t *st)
{
    pid_init(&st->pid, PID_KP, PID_KI, PID_KD, PID_D_FILTER, 0.0f, DUTY_MAX_PCT);
    safety_init(&st->safety, TEMP_CUTOFF_C, SENSOR_FAULT_LIMIT, RUNAWAY_DUTY_PCT,
                (uint32_t)RUNAWAY_WINDOW_S * 1000u, RUNAWAY_MIN_RISE_C, (uint32_t)MAX_RUN_S * 1000u);
    st->applied_duty = 0.0f;
    st->locked = 0;
}

// One pass of loop() in firmware/src/main.cpp (PID mode). elapsed_ms counts from reset.
static float stand_step(stand_t *st, int sensor_ok, float temp_c, float setpoint_c, uint32_t elapsed_ms)
{
    const uint32_t baseline_ms = (uint32_t)BASELINE_S * 1000u;
    const int heating = elapsed_ms >= baseline_ms;
    const uint32_t heat_ms = heating ? elapsed_ms - baseline_ms : 0;
    trip_t trip = safety_update(&st->safety, sensor_ok, temp_c, st->applied_duty, elapsed_ms, heat_ms);
    float duty = 0.0f;
    if (trip != TRIP_NONE) st->locked = 1;
    else if (sensor_ok && heating) duty = pid_update(&st->pid, setpoint_c, temp_c, SAMPLE_PERIOD_MS / 1000.0f);
    st->applied_duty = st->locked ? 0.0f : duty;
    return st->applied_duty;
}

// First-order plant with dead time, T' = (ambient + K*u_delayed - T) / tau. Not a measurement.
typedef struct {
    float K, tau, ambient, T;
    float queue[64];
    int delay, k;
} plant_t;

static void plant_init(plant_t *p, float K, float tau, float ambient, int delay_samples)
{
    memset(p, 0, sizeof *p);
    p->K = K; p->tau = tau; p->ambient = ambient; p->T = ambient; p->delay = delay_samples;
}

static void plant_step(plant_t *p, float duty)
{
    const float dt = SAMPLE_PERIOD_MS / 1000.0f;
    float u = p->queue[p->k % p->delay];
    p->queue[p->k % p->delay] = duty;
    p->k++;
    p->T += dt / p->tau * (p->ambient + p->K * u - p->T);
}

static void st_01_pid_reaches_setpoint(void)  // REQ-14
{
    plant_t pl;
    plant_init(&pl, 0.5f, 120.0f, 22.0f, 40);  // 10 s dead time
    pid_ctrl_t c;
    pid_init(&c, 6.0f, 0.06f, 0.0f, 0.2f, 0.0f, 100.0f);
    const float dt = 0.25f;
    float u = 0, umin = 1e9f, umax = -1e9f;
    for (int k = 0; k < (int)(1800 / dt); k++) {
        u = pid_update(&c, 50.0f, pl.T, dt);
        if (u < umin) umin = u;
        if (u > umax) umax = u;
        plant_step(&pl, u);
    }
    CHECK_NEAR(pl.T, 50.0, 0.05);
    CHECK(umin >= 0.0f && umax <= 100.0f);
    CHECK_NEAR(u, (50.0 - 22.0) / 0.5, 0.5);  // steady-state duty = 56 %
}

static void st_02_overtemperature_in_loop(void)  // REQ-10
{
    // Setpoint above the cutoff and a strong heater: the supervisor has to switch the heater off
    // at the first reading at or above the cutoff, and keep it off.
    // Not asserted: how far the plate coasts afterwards. That depends on heater power and dead
    // time of the real stand (see docs/verification-notes.md, finding F-1).
    stand_t st; plant_t pl;
    stand_init(&st);
    plant_init(&pl, 1.4f, 120.0f, 22.0f, 40);
    int first_hot = -1, duty_at_first_hot = -1, duty_after = 0;
    for (int k = 0; k < (int)(1500 * 1000 / SAMPLE_PERIOD_MS); k++) {
        uint32_t t_ms = (uint32_t)k * SAMPLE_PERIOD_MS;
        float reading = pl.T;
        float duty = stand_step(&st, 1, reading, 90.0f, t_ms);
        if (first_hot < 0 && reading >= TEMP_CUTOFF_C) {
            first_hot = k;
            duty_at_first_hot = (duty == 0.0f);
        } else if (first_hot >= 0 && duty != 0.0f) {
            duty_after++;
        }
        plant_step(&pl, duty);
    }
    CHECK(first_hot > 0);                        // the plate did reach the cutoff
    CHECK(duty_at_first_hot == 1);               // heater off in that very sample
    CHECK(st.safety.trip == TRIP_OVERTEMP);
    CHECK(duty_after == 0);                      // and never on again
}

// Thermocouple falls off the heater at detach_ms: the reading stays at the last good value while
// the real heater keeps running. Returns the trip time in ms, or -1.
static int thermocouple_detach_run(uint32_t detach_ms, int *duty_after_trip, trip_t *trip_out)
{
    stand_t st; plant_t pl;
    stand_init(&st);
    plant_init(&pl, 0.5f, 120.0f, 22.0f, 40);
    float last_good = pl.T;
    int tripped_at = -1;
    *duty_after_trip = 0;
    for (int k = 0; k < (int)(900 * 1000 / SAMPLE_PERIOD_MS); k++) {
        uint32_t t_ms = (uint32_t)k * SAMPLE_PERIOD_MS;
        if (t_ms < detach_ms) last_good = pl.T;
        float duty = stand_step(&st, 1, last_good, 50.0f, t_ms);
        if (st.safety.trip != TRIP_NONE && tripped_at < 0) tripped_at = (int)t_ms;
        if (tripped_at >= 0 && duty != 0.0f) (*duty_after_trip)++;
        plant_step(&pl, duty);
    }
    *trip_out = st.safety.trip;
    return tripped_at;
}

static void st_03_thermocouple_falls_off(void)  // REQ-12
{
    const int window_ms = RUNAWAY_WINDOW_S * 1000;
    int after;
    trip_t trip;

    // (a) detached when heating starts: trips one window later
    uint32_t t0 = (uint32_t)BASELINE_S * 1000u;
    int at = thermocouple_detach_run(t0, &after, &trip);
    CHECK(trip == TRIP_RUNAWAY);
    CHECK(at >= (int)t0 + window_ms);
    CHECK(at <= (int)t0 + window_ms + 2 * SAMPLE_PERIOD_MS);
    CHECK(after == 0);

    // (b) detached 60 s into heating, when a window of real temperature rise is already running:
    // that window ends normally and the next one catches the frozen reading, so within two windows
    uint32_t t1 = t0 + 60u * 1000u;
    at = thermocouple_detach_run(t1, &after, &trip);
    CHECK(trip == TRIP_RUNAWAY);
    CHECK(at > (int)t1);
    CHECK(at <= (int)t1 + 2 * window_ms + 2 * SAMPLE_PERIOD_MS);
    CHECK(after == 0);
}

static void st_04_sensor_unplugged(void)  // REQ-11
{
    stand_t st; plant_t pl;
    stand_init(&st);
    plant_init(&pl, 0.5f, 120.0f, 22.0f, 40);
    uint32_t k = 0;
    float duty = 0;
    for (; k < (uint32_t)(60 * 1000 / SAMPLE_PERIOD_MS); k++) {  // 30 s baseline + 30 s of heating
        duty = stand_step(&st, 1, pl.T, 50.0f, k * SAMPLE_PERIOD_MS);
        plant_step(&pl, duty);
    }
    CHECK(duty > 0.0f);                                          // it was heating
    duty = stand_step(&st, 0, 0.0f, 50.0f, k++ * SAMPLE_PERIOD_MS);
    CHECK(duty == 0.0f && st.safety.trip == TRIP_NONE);         // 1st bad read: off, not tripped
    duty = stand_step(&st, 0, 0.0f, 50.0f, k++ * SAMPLE_PERIOD_MS);
    CHECK(duty == 0.0f && st.safety.trip == TRIP_NONE);         // 2nd
    duty = stand_step(&st, 0, 0.0f, 50.0f, k++ * SAMPLE_PERIOD_MS);
    CHECK(duty == 0.0f && st.safety.trip == TRIP_SENSOR);       // 3rd: tripped
    for (int i = 0; i < 400; i++) {                              // reads come back, heater stays off
        duty = stand_step(&st, 1, pl.T, 50.0f, k++ * SAMPLE_PERIOD_MS);
        CHECK(duty == 0.0f);
    }
}

static void st_05_max_run_time(void)  // REQ-13
{
    stand_t st; plant_t pl;
    stand_init(&st);
    plant_init(&pl, 0.5f, 120.0f, 22.0f, 40);
    const uint32_t end_ms = ((uint32_t)BASELINE_S + (uint32_t)MAX_RUN_S) * 1000u;
    int duty_after_end = 0, was_heating = 0;
    for (uint32_t t_ms = 0; t_ms < end_ms + 300000u; t_ms += SAMPLE_PERIOD_MS) {
        float duty = stand_step(&st, 1, pl.T, 50.0f, t_ms);
        if (t_ms < end_ms - 10000u && duty > 0.0f) was_heating = 1;
        if (t_ms > end_ms + SAMPLE_PERIOD_MS && duty != 0.0f) duty_after_end++;
        plant_step(&pl, duty);
    }
    CHECK(was_heating);
    CHECK(st.safety.trip == TRIP_MAX_RUN);
    CHECK(duty_after_end == 0);
}

// ================================================================ runner

typedef struct {
    const char *id;
    const char *reqs;
    void (*fn)(void);
} test_case_t;

static const test_case_t cases[] = {
    {"TC-01", "REQ-01", tc_01_max31855_decoding},
    {"TC-02", "REQ-02", tc_02_max6675_decoding},
    {"TC-03", "REQ-03", tc_03_sensor_faults},
    {"TC-04", "REQ-04", tc_04_frame_format},
    {"TC-05", "REQ-05", tc_05_frame_range},
    {"TC-06", "REQ-06", tc_06_onoff_hysteresis},
    {"TC-07", "REQ-07", tc_07_pid_terms},
    {"TC-08", "REQ-08", tc_08_pid_antiwindup},
    {"TC-09", "REQ-09", tc_09_pid_output_limits},
    {"TC-10", "REQ-10", tc_10_overtemperature},
    {"TC-11", "REQ-11", tc_11_sensor_fault_tolerance},
    {"TC-12", "REQ-12", tc_12_runaway},
    {"TC-13", "REQ-13", tc_13_max_run},
    {"IT-01", "REQ-01;REQ-04", it_01_decode_to_frame},
    {"ST-01", "REQ-14", st_01_pid_reaches_setpoint},
    {"ST-02", "REQ-10", st_02_overtemperature_in_loop},
    {"ST-03", "REQ-12", st_03_thermocouple_falls_off},
    {"ST-04", "REQ-11", st_04_sensor_unplugged},
    {"ST-05", "REQ-13", st_05_max_run_time},
};

int main(void)
{
    int failed_cases = 0;
    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        case_checks = case_failures = 0;
        cases[i].fn();
        printf("RESULT,%s,%s,%s,%d\n", cases[i].id, cases[i].reqs, case_failures ? "FAIL" : "PASS", case_checks);
        if (case_failures) failed_cases++;
    }
    printf("%d test cases, %d failed\n", (int)(sizeof cases / sizeof cases[0]), failed_cases);
    return failed_cases ? 1 : 0;
}
