# Hardware: parts, wiring, safety

Everything on this stand runs from 12 V DC. No mains wiring is part of the build: the 12 V comes from a closed, CE-marked power supply.

## Parts list

| # | Part | Notes |
|---|---|---|
| 1 | STM32F103C8T6 "Blue Pill" + ST-Link V2 | Same board and flashing setup as my FreeRTOS environmental monitor |
| 2 | MAX31855 thermocouple breakout (K-type) | A MAX6675 board also works: build with `-DSENSOR_MAX6675` |
| 3 | K-type thermocouple, ring/washer type (M4) or glass-fibre insulated | The washer type can be bolted to the heater plate. Must be **ungrounded** (see notes) |
| 4 | Logic-level N-MOSFET, e.g. IRLB8721PbF (TO-220) | Has to switch fully at 3.3 V gate drive. Check the datasheet's R<sub>DS(on)</sub> curve at V<sub>GS</sub> ≈ 3.3 V for your load current |
| 5 | 100 Ω resistor (gate series), 10 kΩ resistor (gate pull-down) | The pull-down keeps the heater off while the MCU is in reset or unprogrammed |
| 6 | Heater: 10 Ω, 25 W aluminium-housed power resistor | At 12 V: 1.2 A, 14.4 W. Run it well below its rating, bolted to a heatsink plate |
| 7 | Aluminium plate, about 100 × 100 × 3 mm, thermal paste, M3/M4 screws | Thermal mass for the controlled system; plate mass sets the time constant |
| 8 | 12 V / 2 A power supply (closed brick) | |
| 9 | Inline fuse holder + 2 A fast fuse | In the +12 V line, as close to the supply as possible |
| 10 | Normally-closed bimetal thermal switch, about 85 °C (e.g. KSD301 type), bolted to the plate | Hardware cutoff in series with the heater: works even if the firmware or MOSFET fails |
| 11 | USB-UART adapter, 3.3 V logic (CP2102 / CH340) | For the logger |
| 12 | Base board (wood or aluminium), stand-offs, screw terminals, wire rated ≥ 2 A | Everything bolted down; the heater plate stands off the base |

## Wiring

| From | To | Notes |
|---|---|---|
| MAX31855 VIN | Blue Pill 3.3 V | |
| MAX31855 GND | Blue Pill GND | |
| MAX31855 CLK | PA5 (SPI1 SCK) | |
| MAX31855 DO | PA6 (SPI1 MISO) | |
| MAX31855 CS | PA4 | |
| PA8 (TIM1_CH1 PWM) | MOSFET gate, through 100 Ω | |
| MOSFET gate | MOSFET source, through 10 kΩ | Pull-down |
| MOSFET source | 12 V supply GND **and** Blue Pill GND | Join the grounds at this one point |
| MOSFET drain | Heater terminal 1 | |
| Heater terminal 2 | Thermal switch → fuse → +12 V | Series chain on the high side |
| USB-UART RX | PA9 (USART1 TX) | |
| USB-UART TX | PA10 (USART1 RX) | |
| USB-UART GND | Blue Pill GND | |
| PC13 | on-board LED | Blinks while running, steady on after a safety trip |

```text
 +12 V ──[2 A fuse]──[thermal switch NC ~85 °C]──┐
                                                 │
                                           [10 Ω 25 W heater]
                                                 │
                                                 D
 PA8 ──[100 Ω]──┬────────────────────────── G  MOSFET (IRLB8721)
                │                                S
              [10 k]                             │
                │                                │
 GND (Blue Pill)┴────────────────────────────────┴── 12 V supply GND

 MAX31855:  VIN-3V3  GND-GND  CLK-PA5  DO-PA6  CS-PA4     thermocouple on the heater plate
 USB-UART:  RX-PA9   TX-PA10  GND-GND                     -> PC, sensor-data-logger
```

No flyback diode is needed: the heater is a resistive load. If you swap in a fan or anything with a coil, add one.

## Notes from the datasheets

- **The MAX31855 needs an ungrounded thermocouple.** If the thermocouple junction touches a grounded metal part, the chip reports a short-to-GND fault (SCG) and the firmware trips with `SENSOR`. Keep the plate floating, or use an insulated probe.
- The MAX31855 resolution is 0.25 °C, so do not report results finer than that. Its accuracy is about ±2 °C over its range; check it against a reference thermometer at room temperature before you trust absolute values.
- The firmware reads the MAX31855 every 250 ms (the conversion takes about 100 ms). The MAX6675 is slower (about 220 ms), so it is read every 300 ms.

## Safety layers

1. **Low voltage only:** 12 V DC from a closed supply.
2. **Fuse** in the +12 V line.
3. **Bimetal thermal switch** in series with the heater: an independent hardware cutoff.
4. **Gate pull-down:** the heater is off while the MCU is in reset or unprogrammed.
5. **Firmware cutoffs** (latched, only a reset clears them): over-temperature at 80 °C, sensor fault (3 bad reads), thermal runaway (heater at 90 % duty or more for 90 s without a 2 °C rise, meaning the thermocouple has come off), and maximum run time.
6. **Independent watchdog (IWDG):** if the control loop hangs, the MCU resets and the heater goes off.
7. **Mechanics:** everything bolted down, the heater plate on stand-offs away from the base, nothing flammable nearby.
8. **Never left running unattended.** Unplug the 12 V supply after each test session.
