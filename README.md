# Flyback Driver

[![View PCB on KiCanvas](https://hack.club/pcb-badge)](https://kicanvas.org/?github=https://github.com/varvarakarenski/Flyback-Driver/tree/main/pcb)

This is a driver for a High Voltage Flyback Transformer.
We are planning on using it in the next stage in our research on EHD propulsion. You can read our preprint from last year here: https://doi.org/10.31224/6625

## Features:

- Variable duty cycle
- Variable frequency
- Current/Voltage Sensing
- Current/Voltage Regulating 6V to 25V, digitally adjustable via 12-bit DAC about 5mV resolution
- Rotary encoder for control
- OLED Display
- Indicator Lights
- Soft Startup
- Hardware overcurrent protection, independent of firmware
- Latching fault shutdown
- Adjustable trip threshold
- RCD clamp with tunable clamp voltage

## PCB

Made in KiCad. The full schematic is in [assets/schematic.pdf](assets/schematic.pdf).

Main parts:

- **MCU:** STM32F103C8T6
- **Flyback switch:** IRFP460 (500V, 20A) driven by a UCC27511A gate driver
- **Supply regulation:** LM5117 buck controller with two MCP4725 12-bit DACs for digital voltage/current setpoints
- **Sensing:** INA226 current/voltage monitor
- **Protection:** LM393 comparator + 74HC132 latch for hardware overcurrent shutdown
- **Logic rails:** LM2596 and TPS62130 bucks

Custom footprints are in [pcb/libraries](pcb/libraries).

## Firmware

No firmware yet but coming soon

## BOM

The full BOM with reference designators, Digi-Key part numbers and datasheets is in [BOM.csv](BOM.csv). Parts on Digi-Key come to about **$94**.

Not on the Digi-Key order yet:

- 1x 0.91" 128x32 I2C OLED (U4)
- 1x 2x4 2.54mm pin header (J1)
- 1x 1x5 2.54mm pin header (J2)
- 16x test points (TP1–TP16)
