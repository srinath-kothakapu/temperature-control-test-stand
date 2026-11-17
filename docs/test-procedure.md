# Test procedure

Run these steps in order. Stay with the stand for every powered test.

The logger is my [sensor-data-logger](https://github.com/srinath-kothakapu/sensor-data-logger) toolkit, cloned next to this repo:

```text
projects/
  sensor-data-logger/
  temperature-control-test-stand/   <- this repo
```

Replace `COM5` with your serial port (`/dev/ttyUSB0` on Linux).

## 0. Bring-up without the heater

1. Leave the +12 V supply **unplugged**. Flash the open-loop build: `pio run -e openloop -t upload`.
2. Open the serial monitor (`pio device monitor`). You should see one `#BOOT ...` line, then about 4 frames per second like `$23.25,0.0*XX`.
3. Hold the thermocouple tip in your fingers: the reading should rise by a few degrees.
4. Unplug the thermocouple: `#SENSOR_FAULT 0x1` appears, then `#TRIP SENSOR`, and the LED stays on. Press reset to clear it.
5. Check the reading at room temperature against a reference thermometer, and write down the difference.

## 1. Gate and heater check

1. Bolt the thermocouple to the plate, then plug in the +12 V supply. Keep a hand near the supply's plug.
2. After the 30 s baseline, the open-loop build switches to 30 % duty. Measure the voltage across the heater with a multimeter. It should show roughly 30 % of 12 V on average (a meter reads the PWM average), and the reading in the serial monitor should start to rise.
3. Unplug the supply and let the plate cool before step 2.

## 2. Open-loop step (plant identification)

1. Let the plate cool to room temperature.
2. Start the logger first, then press reset on the Blue Pill, so the 30 s baseline is captured:
   ```bash
   python ../sensor-data-logger/src/logger.py --port COM5 --baud 115200 --duration 1900 --watchdog 10 --out data/openloop_30pct.csv
   ```
3. Wait for the run to finish. After 30 min the firmware prints `#DONE MAX_RUN`, switches the heater off and keeps logging. Lines that start with `#` show up in the logger's "rejected (no_crc)" count; that is expected.
4. Analyse it:
   ```bash
   python analysis/step_response.py openloop data/openloop_30pct.csv --save results/openloop.png
   ```
   This prints K, τ, the dead time and suggested PI gains. If it warns that the temperature was still rising, run longer: `PLATFORMIO_BUILD_FLAGS="-DMAX_RUN_S=3600" pio run -e openloop -t upload`.

## 3. On/off control

```bash
pio run -e onoff -t upload
python ../sensor-data-logger/src/logger.py --port COM5 --duration 1900 --watchdog 10 --out data/onoff_50C.csv
# press reset on the Blue Pill
```

## 4. PID control

Flash with the gains suggested in step 2, for example:

```bash
PLATFORMIO_BUILD_FLAGS="-DPID_KP=<kp>f -DPID_KI=<ki>f" pio run -e pid -t upload
python ../sensor-data-logger/src/logger.py --port COM5 --duration 1900 --watchdog 10 --out data/pid_run1.csv
# press reset on the Blue Pill
```

On Windows PowerShell, set the flags first: `$env:PLATFORMIO_BUILD_FLAGS="-DPID_KP=..."`.

## 5. Compare and plot

```bash
python analysis/step_response.py closed data/onoff_50C.csv data/pid_run1.csv \
    --setpoint 50 --label "on/off" --label "PID" --save results/step_response.png
```

Copy the printed numbers into the Results table in the README, and the `#BOOT` line of each run into the run log below.

## Run log

Fill one row per real run. If a run went wrong, keep its row and say what happened.

| Date | File | Mode / gains | Setpoint | Room temp | Notes |
|---|---|---|---|---|---|
| | | | | | |
