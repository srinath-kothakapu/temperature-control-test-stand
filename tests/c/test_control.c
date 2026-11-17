// Host unit tests for the firmware control library (runs with gcc, no board needed).
//   make -C tests/c
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "controller.h"
#include "frame.h"
#include "safety.h"
#include "thermocouple.h"

static int failures = 0, checks = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        checks++;                                                            \
        if (!(cond)) {                                                       \
            failures++;                                                      \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
        }                                                                    \
    } while (0)
#define CHECK_NEAR(a, b, tol) CHECK(fabs((double)(a) - (double)(b)) <= (tol))

// ---------------------------------------------------------------- thermocouple decoding

static uint32_t max31855_word(int tc_quarters, int internal_sixteenths, unsigned fault_bits)
{
    uint32_t w = ((uint32_t)(tc_quarters & 0x3FFF) << 18) | ((uint32_t)(internal_sixteenths & 0x0FFF) << 4);
    if (fault_bits) w |= (1u << 16) | (fault_bits & 0x7u);
    return w;
}

static void test_max31855(void)
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

    r = tc_decode_max31855(max31855_word(100, 400, 0x1));
    CHECK(r.faults == TC_FAULT_OPEN);
    r = tc_decode_max31855(max31855_word(100, 400, 0x2));
    CHECK(r.faults == TC_FAULT_SHORT_GND);
    r = tc_decode_max31855(max31855_word(100, 400, 0x4));
    CHECK(r.faults == TC_FAULT_SHORT_VCC);

    CHECK(tc_decode_max31855(0x00000000u).faults == TC_FAULT_BUS);
    CHECK(tc_decode_max31855(0xFFFFFFFFu).faults == TC_FAULT_BUS);
    CHECK(tc_decode_max31855(1u << 16).faults == TC_FAULT_BUS);  // fault flag but no reason bit
}

static void test_max6675(void)
{
    tc_reading_t r = tc_decode_max6675((uint16_t)(100 << 3));  // 25.00 C
    CHECK(tc_ok(&r));
    CHECK_NEAR(r.temp_c, 25.0, 1e-6);
    r = tc_decode_max6675((uint16_t)(4095 << 3));  // 1023.75 C full scale
    CHECK_NEAR(r.temp_c, 1023.75, 1e-6);
    CHECK(tc_decode_max6675((uint16_t)((100 << 3) | 0x4)).faults == TC_FAULT_OPEN);
    CHECK(tc_decode_max6675(0xFFFF).faults == TC_FAULT_BUS);
    CHECK(tc_decode_max6675(0x8000 | (100 << 3)).faults == TC_FAULT_BUS);
}

// ---------------------------------------------------------------- frames

static void test_frames(void)
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

    CHECK(frame_build(buf, sizeof buf, 85.5f, 0.0f) == 0);   // above logger range
    CHECK(frame_build(buf, sizeof buf, -41.0f, 0.0f) == 0);
    CHECK(frame_build(buf, sizeof buf, 20.0f, 100.5f) == 0);
    CHECK(frame_build(buf, sizeof buf, NAN, 0.0f) == 0);
    CHECK(frame_build(buf, 8, 20.0f, 10.0f) == 0);           // buffer too small
}

// ---------------------------------------------------------------- on/off

static void test_onoff(void)
{
    onoff_t c;
    onoff_init(&c, 0.5f, 100.0f);
    CHECK(onoff_update(&c, 50.0f, 40.0f) == 100.0f);
    CHECK(onoff_update(&c, 50.0f, 50.2f) == 100.0f);  // in band, keeps heating
    CHECK(onoff_update(&c, 50.0f, 50.6f) == 0.0f);    // above band, off
    CHECK(onoff_update(&c, 50.0f, 49.8f) == 0.0f);    // in band, stays off
    CHECK(onoff_update(&c, 50.0f, 49.4f) == 100.0f);  // below band, on again
}

// ---------------------------------------------------------------- PID

static void test_pid_terms(void)
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

static void test_pid_antiwindup(void)
{
    pid_ctrl_t c;
    pid_init(&c, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 100.0f);
    // Large error for a long time: output saturates, integral must not keep growing
    for (int i = 0; i < 1000; i++) pid_update(&c, 100.0f, 0.0f, 1.0f);
    CHECK(c.integral <= 100.0f);
    float i_sat = c.integral;
    for (int i = 0; i < 1000; i++) pid_update(&c, 100.0f, 0.0f, 1.0f);
    CHECK_NEAR(c.integral, i_sat, 1e-4);
    // As soon as the error flips sign, the output comes off the limit immediately-ish
    float u = pid_update(&c, 100.0f, 101.0f, 1.0f);
    CHECK(u < 100.0f);
}

// Closed loop against a simple first-order-plus-dead-time model. This only checks that the
// controller code behaves (no steady-state error, output within limits). It says nothing about
// how the real heater behaves.
static void test_pid_closed_loop(void)
{
    const float K = 0.5f, tau = 120.0f, ambient = 22.0f, dt = 0.25f;
    enum { DELAY = 40 };  // 10 s dead time
    float queue[DELAY] = {0};
    float T = ambient;
    pid_ctrl_t c;
    pid_init(&c, 6.0f, 0.06f, 0.0f, 0.2f, 0.0f, 100.0f);
    float u = 0, umin = 1e9f, umax = -1e9f;
    for (int k = 0; k < (int)(1800 / dt); k++) {
        u = pid_update(&c, 50.0f, T, dt);
        if (u < umin) umin = u;
        if (u > umax) umax = u;
        float u_delayed = queue[k % DELAY];
        queue[k % DELAY] = u;
        T += dt / tau * (ambient + K * u_delayed - T);
    }
    CHECK_NEAR(T, 50.0, 0.05);
    CHECK(umin >= 0.0f && umax <= 100.0f);
    CHECK_NEAR(u, (50.0 - ambient) / K, 0.5);  // steady-state duty = 56 %
}

// ---------------------------------------------------------------- safety

static void test_safety(void)
{
    safety_t s;
    safety_init(&s, 80.0f, 3, 90.0f, 60000, 2.0f, 600000);

    // two bad reads are tolerated, the third trips, and it stays tripped
    CHECK(safety_update(&s, false, 0, 0, 0, 0) == TRIP_NONE);
    CHECK(safety_update(&s, false, 0, 0, 250, 0) == TRIP_NONE);
    CHECK(safety_update(&s, false, 0, 0, 500, 0) == TRIP_SENSOR);
    CHECK(safety_update(&s, true, 25, 0, 750, 0) == TRIP_SENSOR);

    // a good read in between resets the counter
    safety_init(&s, 80.0f, 3, 90.0f, 60000, 2.0f, 600000);
    safety_update(&s, false, 0, 0, 0, 0);
    safety_update(&s, false, 0, 0, 250, 0);
    CHECK(safety_update(&s, true, 25, 0, 500, 0) == TRIP_NONE);
    CHECK(safety_update(&s, false, 0, 0, 750, 0) == TRIP_NONE);

    // over-temperature
    safety_init(&s, 80.0f, 3, 90.0f, 60000, 2.0f, 600000);
    CHECK(safety_update(&s, true, 79.75f, 50, 0, 1000) == TRIP_NONE);
    CHECK(safety_update(&s, true, 80.0f, 50, 250, 1250) == TRIP_OVERTEMP);

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

    // max run time
    safety_init(&s, 80.0f, 3, 90.0f, 60000, 2.0f, 600000);
    CHECK(safety_update(&s, true, 40.0f, 30, 0, 599999) == TRIP_NONE);
    CHECK(safety_update(&s, true, 40.0f, 30, 250, 600000) == TRIP_MAX_RUN);
    CHECK(strcmp(safety_trip_name(TRIP_MAX_RUN), "MAX_RUN") == 0);
}

int main(void)
{
    test_max31855();
    test_max6675();
    test_frames();
    test_onoff();
    test_pid_terms();
    test_pid_antiwindup();
    test_pid_closed_loop();
    test_safety();
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
