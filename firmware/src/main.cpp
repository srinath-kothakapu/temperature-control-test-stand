// Temperature control test stand: STM32F103C8 "Blue Pill", K-type thermocouple on MAX31855
// (or MAX6675), logic-level MOSFET driving a 12 V resistive heater with PWM.
//
// Sequence after reset:
//   1. BASELINE_S seconds with the heater off (records the starting temperature)
//   2. step: open-loop fixed duty, on/off, or PID towards SETPOINT_C (chosen at build time)
//   3. after MAX_RUN_S the heater switches off and logging continues (cool-down)
//
// Output: one CRC-checked frame per sample, "$<temp_c>,<duty_pct>*<CRC>", read by the
// sensor-data-logger toolkit. Status lines start with '#' (the logger skips and counts them).
//
// Safety (latched until reset): over-temperature cutoff, sensor fault, thermal runaway
// (thermocouple fell off the heater), max run time, independent hardware watchdog.

#include <Arduino.h>
#include <IWatchdog.h>
#include <SPI.h>

#include "config.h"
#include "controller.h"
#include "frame.h"
#include "safety.h"
#include "thermocouple.h"

#if defined(MODE_OPENLOOP)
static const char *MODE_NAME = "openloop";
#elif defined(MODE_ONOFF)
static const char *MODE_NAME = "onoff";
#else
static const char *MODE_NAME = "pid";
#endif

static HardwareTimer *pwm = nullptr;
static uint32_t pwm_channel = 0;
static uint32_t pwm_period_ticks = 1;
static bool heater_locked = false;

static onoff_t onoff;
static pid_ctrl_t pid;
static safety_t safety;

static uint32_t t_start_ms = 0;
static uint32_t next_sample_ms = 0;
static float applied_duty = 0.0f;
static trip_t reported_trip = TRIP_NONE;

// ---------------------------------------------------------------- heater output

static void heater_init()
{
    // Gate held low by the 10k pull-down on the board until the timer takes over.
    pinMode(PIN_HEATER, OUTPUT);
    digitalWrite(PIN_HEATER, LOW);

    PinName pin = digitalPinToPinName(PIN_HEATER);
    TIM_TypeDef *instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
    pwm_channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
    pwm = new HardwareTimer(instance);
    pwm->setPWM(pwm_channel, PIN_HEATER, PWM_FREQ_HZ, 0);
    pwm_period_ticks = pwm->getOverflow(TICK_FORMAT);
}

static void heater_set(float duty_pct)
{
    if (heater_locked) return;
    if (duty_pct < 0.0f) duty_pct = 0.0f;
    if (duty_pct > DUTY_MAX_PCT) duty_pct = DUTY_MAX_PCT;
    uint32_t ticks = (uint32_t)(duty_pct * (float)pwm_period_ticks / 100.0f + 0.5f);
    pwm->setCaptureCompare(pwm_channel, ticks, TICK_COMPARE_FORMAT);
}

// Hard off: stop the timer and drive the pin as a plain GPIO. Only a reset undoes this.
static void heater_lock_off()
{
    if (pwm) {
        pwm->setCaptureCompare(pwm_channel, 0, TICK_COMPARE_FORMAT);
        pwm->pause();
    }
    pinMode(PIN_HEATER, OUTPUT);
    digitalWrite(PIN_HEATER, LOW);
    heater_locked = true;
}

// ---------------------------------------------------------------- thermocouple

static const SPISettings TC_SPI(1000000, MSBFIRST, SPI_MODE0);

static tc_reading_t read_thermocouple()
{
    SPI.beginTransaction(TC_SPI);
    digitalWrite(PIN_TC_CS, LOW);
    delayMicroseconds(1);
#ifdef SENSOR_MAX6675
    uint16_t raw = (uint16_t)SPI.transfer(0) << 8;
    raw |= SPI.transfer(0);
#else
    uint32_t raw = 0;
    for (int i = 0; i < 4; i++) raw = (raw << 8) | SPI.transfer(0);
#endif
    digitalWrite(PIN_TC_CS, HIGH);
    SPI.endTransaction();
#ifdef SENSOR_MAX6675
    return tc_decode_max6675(raw);
#else
    return tc_decode_max31855(raw);
#endif
}

// ---------------------------------------------------------------- control law

static float control(float temp_c, float dt_s)
{
#if defined(MODE_OPENLOOP)
    (void)temp_c;
    (void)dt_s;
    return OPENLOOP_DUTY_PCT;
#elif defined(MODE_ONOFF)
    (void)dt_s;
    return onoff_update(&onoff, SETPOINT_C, temp_c);
#else
    return pid_update(&pid, SETPOINT_C, temp_c, dt_s);
#endif
}

static void print_fixed(float x, int decimals)
{
    Serial.print(x, decimals);  // Print::print(double) works without newlib printf float support
}

// ---------------------------------------------------------------- setup / loop

void setup()
{
    heater_init();  // first, so the heater is defined-off as early as possible

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);  // off (active low)
    pinMode(PIN_TC_CS, OUTPUT);
    digitalWrite(PIN_TC_CS, HIGH);
    SPI.begin();

    Serial.begin(SERIAL_BAUD);

    bool wdt_reset = IWatchdog.isReset(true);
    IWatchdog.begin(WATCHDOG_TIMEOUT_US);

    onoff_init(&onoff, ONOFF_HYST_C, DUTY_MAX_PCT);
    pid_init(&pid, PID_KP, PID_KI, PID_KD, PID_D_FILTER, 0.0f, DUTY_MAX_PCT);
    safety_init(&safety, TEMP_CUTOFF_C, SENSOR_FAULT_LIMIT, RUNAWAY_DUTY_PCT,
                (uint32_t)RUNAWAY_WINDOW_S * 1000u, RUNAWAY_MIN_RISE_C, (uint32_t)MAX_RUN_S * 1000u);

    Serial.print("#BOOT mode=");
    Serial.print(MODE_NAME);
#if defined(MODE_OPENLOOP)
    Serial.print(" duty=");
    print_fixed(OPENLOOP_DUTY_PCT, 1);
#else
    Serial.print(" setpoint=");
    print_fixed(SETPOINT_C, 2);
#endif
#if defined(MODE_PID)
    Serial.print(" kp=");
    print_fixed(PID_KP, 4);
    Serial.print(" ki=");
    print_fixed(PID_KI, 4);
    Serial.print(" kd=");
    print_fixed(PID_KD, 4);
#endif
    Serial.print(" baseline_s=");
    Serial.print(BASELINE_S);
    Serial.print(" period_ms=");
    Serial.print(SAMPLE_PERIOD_MS);
    Serial.print(" cutoff=");
    print_fixed(TEMP_CUTOFF_C, 1);
    if (wdt_reset) Serial.print(" WATCHDOG_RESET");
    Serial.println();

    t_start_ms = millis();
    next_sample_ms = t_start_ms;
}

void loop()
{
    uint32_t now = millis();
    if ((int32_t)(now - next_sample_ms) < 0) return;
    next_sample_ms += SAMPLE_PERIOD_MS;

    const float dt_s = SAMPLE_PERIOD_MS / 1000.0f;
    const uint32_t elapsed = now - t_start_ms;
    const uint32_t baseline_ms = (uint32_t)BASELINE_S * 1000u;
    const bool heating = elapsed >= baseline_ms;
    const uint32_t heat_ms = heating ? elapsed - baseline_ms : 0;

    tc_reading_t r = read_thermocouple();
    bool ok = tc_ok(&r);

    trip_t trip = safety_update(&safety, ok, r.temp_c, applied_duty, now, heat_ms);

    float duty = 0.0f;
    if (trip != TRIP_NONE) {
        if (!heater_locked) heater_lock_off();
        if (trip != reported_trip) {
            Serial.print(trip == TRIP_MAX_RUN ? "#DONE " : "#TRIP ");
            Serial.println(safety_trip_name(trip));
            reported_trip = trip;
        }
    } else if (ok && heating) {
        duty = control(r.temp_c, dt_s);
    }
    // !ok without a trip yet: single bad read, heater off for this sample
    heater_set(duty);
    applied_duty = heater_locked ? 0.0f : duty;

    if (ok) {
        char line[40];
        if (frame_build(line, sizeof line, r.temp_c, applied_duty)) Serial.println(line);
    } else {
        Serial.print("#SENSOR_FAULT 0x");
        Serial.println(r.faults, HEX);
    }

    // LED: blinks while running, steady on after a trip
    digitalWrite(PIN_LED, (trip != TRIP_NONE) ? LOW : ((elapsed / 500u) & 1u ? LOW : HIGH));

    IWatchdog.reload();
}
