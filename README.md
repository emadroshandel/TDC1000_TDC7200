# TDC1000 and TDC7200 for Arduino
This project is a fork of [Yveaux's original repository](https://github.com/Yveaux/TDC1000) for the TDC1000 and TDC7200 integrated circuits, which function as time-to-digital converters for precise ultrasonic signal measurement and timing applications. The library is fully functional and specifically designed for Arduino development.
I created this fork to contribute additional examples and provide more detailed explanations for users who want to better understand and apply the existing libraries.

# Table of Contents
- Hardware Overview
- How It Works
- Hardware Connections
- Key Concepts
- Configuration
- Getting Started
- Interpreting Results
- Troubleshooting
- Technical Details

# Hardware Overview
Required Components:

**TDC1000 - Ultrasonic Sensing Analog Front End**

- Drives ultrasonic transducers
- Receives and conditions echo signals
- Generates START and STOP pulses for timing

**TDC7200 - Time-to-Digital Converter**

- Measures time intervals with picosecond precision
- Captures up to 5 STOP events per measurement
- Communicates via SPI

**Ultrasonic Transducers (2x)**

- Recommended: 1 MHz transducers for water
- Mounted facing each other for through-transmission


**Microcontroller**

- Arduino-compatible board
- Requires SPI interface and GPIO pins


**300pF Capacitors (2x)**

Required between transducers and TDC1000 for AC coupling
