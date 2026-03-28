# BMS — STM32G491RE Battery Management System

A battery management system firmware for the NUCLEO-G491RE development board.

## Hardware
- **MCU**: STM32G491RE (ARM Cortex-M4, 170 MHz)
- **Current/Voltage sensor**: INA228 (I2C)
- **Temperature sensor**: NTC thermistor (ADC)
- **Display**: SSD1306 OLED 128x64 (I2C)
- **Load control**: MOSFET on PB5

## Features
- SOC estimation (Coulomb counting + OCV correction)
- Real-time current, voltage, temperature display
- Time-to-empty calculation
- Over-temperature & over/under-current fault detection
- MOSFET thermal cutoff (>30°C → OFF)
- 5-page UI with button navigation

## Project Structure
```
Core/
├── Src/
│   ├── main.c        # Main application
│   ├── ssd1306.c     # OLED driver
│   └── font5x7.c     # Font data
└── Inc/
    ├── ssd1306.h
    └── font5x7.h
NUCLEO_G491RE_Test.ioc  # STM32CubeMX configuration
```

## Version History
- **v1.0** — Initial BMS with INA228 + NTC + basic OLED UI
- **v2.0** — Added Nucleo BSP, refined fault detection
- **v3.0** — Coulomb counting SOC, TTE estimation, MOSFET thermal control

