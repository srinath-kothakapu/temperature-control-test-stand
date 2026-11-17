// Test-stand configuration. Every value can be overridden with -D in platformio.ini.
#pragma once

// ---------------------------------------------------------------- control mode
// Exactly one of these is set per PlatformIO environment:
//   MODE_OPENLOOP  fixed duty step, for identifying the plant (gain, time constant, dead time)
//   MODE_ONOFF     bang-bang control with hysteresis
//   MODE_PID       PID with anti-windup and derivative on measurement
#if !defined(MODE_OPENLOOP) && !defined(MODE_ONOFF) && !defined(MODE_PID)
#define MODE_PID
#endif

// ---------------------------------------------------------------- sensor
// Default is MAX31855 (14-bit, 0.25 C). Build with -DSENSOR_MAX6675 for the MAX6675 board.
#ifndef SAMPLE_PERIOD_MS
#ifdef SENSOR_MAX6675
#define SAMPLE_PERIOD_MS 300   // MAX6675 needs about 220 ms per conversion
#else
#define SAMPLE_PERIOD_MS 250   // MAX31855 converts in about 100 ms
#endif
#endif

// ---------------------------------------------------------------- step test
#ifndef BASELINE_S
#define BASELINE_S 30          // heater off this long after reset, to record the starting temperature
#endif
#ifndef SETPOINT_C
#define SETPOINT_C 50.0f       // closed-loop target (on/off and PID)
#endif
#ifndef OPENLOOP_DUTY_PCT
#define OPENLOOP_DUTY_PCT 30.0f  // open-loop step size
#endif

// ---------------------------------------------------------------- on/off
#ifndef ONOFF_HYST_C
#define ONOFF_HYST_C 0.5f      // heater on below SP-H, off above SP+H
#endif

// ---------------------------------------------------------------- PID
// Starting values only. Tune them from your own open-loop run:
//   python analysis/step_response.py openloop data/openloop.csv
#ifndef PID_KP
#define PID_KP 8.0f            // % duty per C
#endif
#ifndef PID_KI
#define PID_KI 0.05f           // % duty per (C*s)
#endif
#ifndef PID_KD
#define PID_KD 0.0f            // % duty * s per C (derivative on measurement)
#endif
#ifndef PID_D_FILTER
#define PID_D_FILTER 0.2f      // 0..1, smoothing of the derivative term (1 = no filter)
#endif

// ---------------------------------------------------------------- actuator
#ifndef PWM_FREQ_HZ
#define PWM_FREQ_HZ 500
#endif
#ifndef DUTY_MAX_PCT
#define DUTY_MAX_PCT 100.0f    // lower this to limit heater power
#endif

// ---------------------------------------------------------------- safety (latched until reset)
#ifndef TEMP_CUTOFF_C
#define TEMP_CUTOFF_C 80.0f    // heater off at or above this. Keep below the logger limit of 85 C
#endif
#ifndef SENSOR_FAULT_LIMIT
#define SENSOR_FAULT_LIMIT 3   // consecutive bad sensor reads before tripping
#endif
#ifndef RUNAWAY_DUTY_PCT
#define RUNAWAY_DUTY_PCT 90.0f // "heater is working hard" threshold
#endif
#ifndef RUNAWAY_WINDOW_S
#define RUNAWAY_WINDOW_S 90    // ...for this long...
#endif
#ifndef RUNAWAY_MIN_RISE_C
#define RUNAWAY_MIN_RISE_C 2.0f // ...must raise the reading by at least this much, else the TC is off the heater
#endif
#ifndef MAX_RUN_S
#define MAX_RUN_S 1800         // heater off after this, logging continues (cool-down curve)
#endif
#ifndef WATCHDOG_TIMEOUT_US
#define WATCHDOG_TIMEOUT_US 2000000  // independent watchdog: resets the MCU (heater off) if the loop hangs
#endif

// ---------------------------------------------------------------- pins (Blue Pill)
#define PIN_TC_CS   PA4        // SPI1: SCK PA5, MISO PA6 (MOSI unused)
#define PIN_HEATER  PA8        // TIM1_CH1 PWM -> MOSFET gate
#define PIN_LED     PC13       // on-board LED, active low
#define SERIAL_BAUD 115200     // USART1: TX PA9, RX PA10 via USB-UART adapter
