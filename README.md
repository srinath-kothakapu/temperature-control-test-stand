# Temperature Control Test Stand

![CI](https://github.com/srinath-kothakapu/temperature-control-test-stand/actions/workflows/ci.yml/badge.svg)

A small 12 V test stand for closed-loop temperature control. A K-type thermocouple on a MAX31855 is read over SPI by an STM32F103 ("Blue Pill"). The STM32 drives a power-resistor heater with PWM through a logic-level MOSFET. The firmware has three build modes: an open-loop step to identify the plant, on/off control, and PID. Each run is logged over UART with my [sensor-data-logger](https://github.com/srinath-kothakapu/sensor-data-logger) toolkit, and a Python script reports rise time, overshoot, settling time and steady-state error from the log.

> **Status:** the firmware and analysis code are written and unit-tested on the host (see [Tests](#tests)). The physical stand has **not been built and measured yet**. The Results section will only contain numbers from real runs.

<!-- Add a photo of the wired stand here: ![Test stand wiring](docs/wiring.jpg) -->

## What it covers

- **Thermal measurement:** K-type thermocouple, MAX31855 cold-junction-compensated converter, SPI driver with fault decoding (open circuit, short to GND/VCC, missing board).
- **Actuator:** 500 Hz hardware PWM on TIM1, a logic-level MOSFET, a 12 V / 14 W resistive heater.
- **Control:** on/off with hysteresis, and PID with derivative on measurement, a derivative filter and anti-windup by conditional integration. Starting gains come from an open-loop FOPDT fit (SIMC rules).
- **Safety:** a fuse, a bimetal hardware cutoff, a gate pull-down, and latched firmware trips (over-temperature, sensor fault, thermal runaway, max run time). An independent watchdog resets the MCU if the loop hangs.
- **Test and evaluation:** CRC-checked UART frames, CSV logging, step-response metrics and plots, a written test procedure and run log.

## How it fits together

```text
 K-type TC ─> MAX31855 ──SPI──> STM32F103 ──PWM──> MOSFET ──> 12 V heater on Al plate
                                   │                              │
                                   │      heat (thermal plant) <──┘
                                   │
                                   └──UART "$temp,duty*CRC"──> sensor-data-logger ──> CSV
                                                                                   │
                                                  analysis/step_response.py <──────┘
                                                  (metrics, FOPDT fit, plots)
```

After a reset the firmware keeps the heater off for 30 s, recording a baseline. Then it applies the step: a fixed duty in open-loop mode, or setpoint 50 °C for on/off and PID. After 30 min the heater switches off and logging continues, so the cool-down is recorded too.

The logger expects frames of the form `$<a>,<b>*<CRC-8>`. The firmware sends the temperature in °C as the first value and the heater duty in % as the second, so the logger is used unchanged. In its CSV the duty appears in the `humidity_pct` column.

## Hardware

The parts list, wiring table, schematic and safety notes are in **[docs/hardware.md](docs/hardware.md)**.

| Signal | Blue Pill pin |
|---|---|
| MAX31855 SCK / DO / CS | PA5 / PA6 / PA4 |
| Heater PWM (TIM1_CH1) → MOSFET gate | PA8 |
| UART to PC (USART1 TX / RX) | PA9 / PA10 |
| Status LED | PC13 |

## Build and flash

Requires [PlatformIO](https://platformio.org/) and an ST-Link.

```bash
pio run -e openloop -t upload   # fixed 30 % duty step
pio run -e onoff -t upload      # on/off, setpoint 50 °C, ±0.5 °C hysteresis
pio run -e pid -t upload        # PID, setpoint 50 °C
```

All parameters live in [`firmware/include/config.h`](firmware/include/config.h) and can be overridden without editing code:

```bash
PLATFORMIO_BUILD_FLAGS="-DSETPOINT_C=60.0f -DPID_KP=6.5f -DPID_KI=0.04f" pio run -e pid -t upload
```

## Run a test and analyse it

The full procedure, including bring-up checks and a run log, is in **[docs/test-procedure.md](docs/test-procedure.md)**. In short:

```bash
# start logging, then press reset on the Blue Pill
python ../sensor-data-logger/src/logger.py --port COM5 --duration 1900 --watchdog 10 --out data/pid_run1.csv

# open-loop run: plant model and suggested PI gains
python analysis/step_response.py openloop data/openloop_30pct.csv --save results/openloop.png

# closed-loop runs: metrics and a combined plot
python analysis/step_response.py closed data/onoff_50C.csv data/pid_run1.csv \
    --setpoint 50 --label "on/off" --label "PID" --save results/step_response.png
```

The script detects the step at the first sample where the duty becomes non-zero. It reports:

| Metric | Definition |
|---|---|
| Rise time | 10 % → 90 % of the step from the baseline temperature to the setpoint |
| Overshoot | Peak above setpoint, in °C and as % of the step |
| Settling time | Time after the step until the temperature stays inside ±1 °C (`--band`) to the end of the log |
| Steady-state error / ripple | Mean error and peak-to-peak over the last 120 s (`--ss-window`), only if it settled |
| Data quality | Sample count, median interval, largest gap in the log |

## Results

> **Not measured yet.** After the runs in [docs/test-procedure.md](docs/test-procedure.md), this section gets the plot and the numbers printed by `analysis/step_response.py`, along with the room temperature, gains and date of each run.

<!-- ![Measured step response](results/step_response.png) -->

| | On/off | PID |
|---|---|---|
| Setpoint | | |
| Rise time 10–90 % | | |
| Overshoot | | |
| Settling time (±1 °C) | | |
| Steady-state error | | |
| Steady-state ripple (p-p) | | |
| Gains | – | |

Plant model from the open-loop run: K = _ °C/%, τ = _ s, θ = _ s.

## Design notes

- **Derivative on measurement:** the D term acts on −dT/dt instead of de/dt, so a setpoint step does not cause an output spike. A first-order filter (`PID_D_FILTER`) limits noise from the 0.25 °C quantisation. D is off by default; a thermal plant with modest dead time usually needs only PI.
- **Anti-windup:** the integral is frozen while the output is saturated and the error would push it further, so the long full-power heat-up does not wind it up and cause a large overshoot.
- **Fixed sample period:** the loop runs on a fixed 250 ms schedule (`next_sample_ms += period`), so dt is constant and matches what the PID assumes.
- **Latched safety:** every trip turns the heater off by stopping the timer and driving the pin low as a GPIO. Only a reset clears it. The runaway check catches the dangerous case where the thermocouple has come off the plate: the reading stays cold, so the controller would otherwise keep heating at full power.
- **Pure-C control library:** the thermocouple decoding, controllers, safety supervisor and frame builder (`firmware/lib/control/`) have no hardware dependencies, so they are unit-tested on a PC with gcc.

## Tests

```bash
make -C tests/c                 # firmware library: decoding, frames, on/off, PID, safety (gcc)
python -m pytest -q tests       # analysis: FOPDT fit, metrics, CLI
```

The C tests check the MAX31855/MAX6675 decoding against datasheet values. They check that the frame CRC matches the logger's (`$23.45,41.2*BE` from the logger's README), PID terms and anti-windup, every safety trip, and that the PID reaches the setpoint against a simple simulated plant. The Python tests check the analysis against synthetic curves with known answers. All this test data is synthetic and checks only the code; it is not a result. CI also builds all three firmware modes with PlatformIO.

## Repository structure

```text
firmware/
  src/main.cpp              Hardware layer: SPI, PWM, UART, watchdog, sequencing
  include/config.h          All tunable parameters and pins
  lib/control/              Pure C: thermocouple decoding, on/off + PID, safety, frames
analysis/step_response.py   Step-response metrics, FOPDT fit, SIMC gains, plots
tests/c/                    Host unit tests for the firmware library
tests/test_step_response.py Tests for the analysis
docs/hardware.md            Parts, wiring, schematic, safety
docs/test-procedure.md      Step-by-step tests and run log
data/                       Logger CSVs from real runs
results/                    Plots from real runs
platformio.ini              Build environments: openloop, onoff, pid
```

## Limitations

- Not yet run on hardware; see Status.
- The MAX31855 has 0.25 °C resolution and about ±2 °C accuracy. Absolute temperatures are only as good as that; step-response shapes and relative comparisons are more reliable.
- The thermocouple measures one point on the plate, not the plate's average temperature.
- Setpoint and mode are fixed at build time. There is no runtime command interface, because the logger owns the serial port during a run.
- Timestamps come from the PC (logger side), so USB-serial buffering adds some jitter.
