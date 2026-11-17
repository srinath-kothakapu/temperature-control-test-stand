# Temperature Control Test Stand

![CI](https://github.com/srinath-kothakapu/temperature-control-test-stand/actions/workflows/ci.yml/badge.svg)

This is a small 12 V stand for trying out temperature control on real hardware. A K-type thermocouple sits on an aluminium plate with a power resistor bolted to it. An STM32 Blue Pill reads the thermocouple through a MAX31855 (SPI) and heats the plate with PWM through a logic-level MOSFET. I log every run with my [sensor-data-logger](https://github.com/srinath-kothakapu/sensor-data-logger) and then calculate rise time, overshoot and settling time from the log.

I wanted one project that covers the whole loop: sensor, actuator, control, safety and an honest measurement at the end.

**Status:** the firmware and the analysis script are done and tested on the PC. The stand itself is not built yet, so there are no measured results below. I will add them after the runs.

<!-- photo of the wired stand: ![Test stand](docs/wiring.jpg) -->

## How it works

```text
 K-type TC -> MAX31855 --SPI--> STM32F103 --PWM--> MOSFET --> 12 V heater on Al plate
                                   |
                                   +--UART "$temp,duty*CRC"--> sensor-data-logger --> CSV
                                                                                   |
                                                  analysis/step_response.py <------+
```

After reset the heater stays off for 30 s, so I get the starting temperature in the log. Then the step comes. What the step is depends on which firmware I flash:

- `openloop`: fixed 30 % duty. I use this run to measure the plate (gain, time constant, dead time).
- `onoff`: simple on/off control to 50 °C with ±0.5 °C hysteresis.
- `pid`: PID to 50 °C.

After 30 min the heater switches off and the logging continues, so the cool-down is in the log as well.

The logger was written for temperature and humidity (`$temp,hum*CRC`). I didn't want to change it, so the firmware sends the heater duty in the humidity field. In the CSV the duty is in the `humidity_pct` column.

## Hardware

The parts list, wiring and safety notes are in [docs/hardware.md](docs/hardware.md). Short version of the pins:

| Signal | Pin |
|---|---|
| MAX31855 SCK / DO / CS | PA5 / PA6 / PA4 |
| Heater PWM to MOSFET gate (TIM1_CH1) | PA8 |
| UART to PC (TX / RX) | PA9 / PA10 |
| Status LED | PC13 |

Safety was important to me because the thing gets hot. Everything is 12 V from a closed power supply. There is a fuse, a bimetal thermal switch in series with the heater, and a pull-down on the MOSFET gate. In the firmware the heater switches off (and stays off until reset) when:

- the temperature reaches 80 °C
- the thermocouple gives 3 bad reads in a row
- the heater runs at more than 90 % for 90 s and the temperature doesn't rise by at least 2 °C (this means the thermocouple has fallen off the plate)
- the maximum run time is over

The independent watchdog resets the MCU if the loop hangs. And I don't leave it running alone.

## Build and flash

You need PlatformIO and an ST-Link.

```bash
pio run -e openloop -t upload
pio run -e onoff -t upload
pio run -e pid -t upload
```

All settings are in [firmware/include/config.h](firmware/include/config.h). You can change them without editing the code:

```bash
PLATFORMIO_BUILD_FLAGS="-DSETPOINT_C=60.0f -DPID_KP=6.5f -DPID_KI=0.04f" pio run -e pid -t upload
```

## Running a test

The full steps are in [docs/test-procedure.md](docs/test-procedure.md). Basically: start the logger, then press reset on the Blue Pill.

```bash
python ../sensor-data-logger/src/logger.py --port COM5 --duration 1900 --watchdog 10 --out data/pid_run1.csv
```

Then analyse it:

```bash
# open-loop run: fits a first-order model with dead time and suggests PI gains (SIMC)
python analysis/step_response.py openloop data/openloop_30pct.csv --save results/openloop.png

# closed-loop runs: metrics and one plot with both runs
python analysis/step_response.py closed data/onoff_50C.csv data/pid_run1.csv \
    --setpoint 50 --label "on/off" --label "PID" --save results/step_response.png
```

The script finds the step itself (the first sample where the duty is above 0). It prints:

- rise time (10 % to 90 % of the step)
- overshoot (in °C and in % of the step)
- settling time (when it stays inside ±1 °C until the end)
- steady-state error and ripple over the last 120 s

It also prints how many samples there are and the biggest gap in the log, so I can see if the data is OK.

## Results

Not measured yet. The table gets filled after the real runs, together with the plot, the room temperature and the gains I used.

<!-- ![Measured step response](results/step_response.png) -->

| | On/off | PID |
|---|---|---|
| Setpoint | | |
| Rise time 10–90 % | | |
| Overshoot | | |
| Settling time (±1 °C) | | |
| Steady-state error | | |
| Ripple (p-p) | | |
| Gains | – | |

Plate model from the open-loop run: K = _ °C/%, τ = _ s, dead time = _ s.

## Some decisions

- **D on the measurement, not on the error.** If D acts on the error, a setpoint step gives a big spike in the output. With D on the temperature that doesn't happen. I left Kd at 0 for now, because for a slow thermal plant PI is usually enough. If I use D later, it has a filter, because the MAX31855 only has 0.25 °C steps.
- **Anti-windup.** The heat-up takes long at 100 % duty. Without anti-windup the integral grows the whole time and then you get a big overshoot. So the integral stops when the output is saturated.
- **Fixed sample time of 250 ms.** The PID assumes a constant dt, so the loop runs on a fixed schedule and not "as fast as possible".
- **The control code is plain C without hardware calls** (`firmware/lib/control/`). That way I can test it on the PC with gcc, without the board.

## Tests

```bash
make -C tests/c               # firmware library (gcc on the PC)
python -m pytest -q tests     # analysis script
```

The C tests check the thermocouple decoding against the datasheet values, the CRC against my logger (`$23.45,41.2*BE`), the PID, the anti-windup and every safety trip. One test runs the PID against a simple model of a heater to see that it reaches the setpoint. That is only a check of the code, not a result. The Python tests use generated curves where I know the right answer. GitHub Actions runs all tests and also builds the three firmware versions.

## Folder structure

```text
firmware/src/main.cpp        SPI, PWM, UART, watchdog, sequence
firmware/include/config.h    all settings and pins
firmware/lib/control/        thermocouple decoding, on/off, PID, safety, frames
analysis/step_response.py    metrics, model fit, plots
tests/                       C and Python tests
docs/                        hardware and test procedure
data/                        CSV files from real runs
results/                     plots from real runs
```

## Limitations

- Not built yet (see status).
- The MAX31855 has 0.25 °C resolution and about ±2 °C accuracy, so the absolute values are not very exact. The shape of the step response is more useful than the exact number.
- The thermocouple measures one point on the plate, not the whole plate.
- Setpoint and mode are fixed at build time. There is no serial command interface, because the logger uses the port during a run.
- The timestamps come from the PC, so USB buffering adds a bit of jitter.
