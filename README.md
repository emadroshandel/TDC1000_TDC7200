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

# How It Works
The Measurement Sequence
This system performs two sequential measurements for bidirectional flow detection:
**Measurement 1: Channel 1 (A → B)**
```
TX1 (Transducer A) ──ultrasound──> RX2 (Transducer B)
```
**Measurement 2: Channel 2 (B → A)**

```
TX2 (Transducer B) ──ultrasound──> RX1 (Transducer A)
```
**Step-by-Step Process**
For each measurement direction:

- Configure TDC1000 for the desired channel (Channel1 or Channel2)
- Arm TDC7200 by calling startMeasurement() - this makes INT pin go HIGH
- Trigger TDC1000 with a 20µs pulse to start ultrasonic burst

Hardware sequence happens automatically:

- TDC1000 sends START pulse to TDC7200 (timer starts)
- TDC1000 transmits an ultrasonic burst from the TX transducer
- Sound travels through water
- RX transducer receives echo
- TDC1000 generates STOP pulses (one per zero-crossing of received signal)
- TDC7200 captures the timing of each STOP pulse
- After capturing all STOPs, TDC7200 sets INT pin LOW
- Poll INT pin until it goes LOW (measurement complete)
- Read results from TDC7200 via SPI

**Understanding the INT Pin**

CRITICAL CONCEPT: The TDC7200's INT pin is an OUTPUT from the chip, not an interrupt input:

- HIGH = Measurement in progress (waiting for STOP pulses)
- LOW = Measurement complete (all STOPs captured)
  
```
// I configure it as INPUT and poll it:
pinMode(PIN_TDC7200_INT, INPUT_PULLUP);

// Wait for measurement completion
while (digitalRead(PIN_TDC7200_INT) == HIGH) {
    // Still waiting for STOP pulses...
}
// INT went LOW = measurement done!
```

No interrupt handler needed! Simple polling works because measurements are fast (30-120µs typically).
# Hardware Connections
Signal Flow Diagram
Microcontroller              TDC1000                    TDC7200
     |                          |                          |
     |---SPI Config------------>|                          |
     |---SPI Config----------------------------------->|
     |                          |                          |
     |---TRIGGER (20µs)-------->|                          |
     |                          |                          |
     |                          |--START pulse------------>| (Timer starts)
     |                          |                          |
     |                          |--Ultrasonic burst------->| (TX transducer)
     |                          |                          |
     |                          |<--Echo received----------| (RX transducer)
     |                          |                          |
     |                          |--STOP pulse 1----------->| (Capture time 1)
     |                          |--STOP pulse 2----------->| (Capture time 2)
     |                          |--STOP pulse 3----------->| (Capture time 3)
     |                          |--STOP pulse 4----------->| (Capture time 4)
     |                          |--STOP pulse 5----------->| (Capture time 5)
     |                          |                          |
     |<--INT goes LOW---------------------------------| (Done!)
     |                          |                          |
     |---Read measurements via SPI----------------->|

     
### Transducer Setup
     [Transducer A]                    [Transducer B]
     TX1 ─────┐                        ┌───── TX2
              │                        │
     RX2 ──┐  │    <<<< Water >>>>    │  ┌── RX1
           │  │                        │  │
         [300pF]                    [300pF]
           │  │                        │  │
        To TDC1000              To TDC1000
**Important: 300pF capacitors provide AC coupling and are required for proper operation.**


