# TDC1000 and TDC7200 for Arduino
This project is a fork of [Yveaux's original repository](https://github.com/Yveaux/TDC1000) for the TDC1000 and TDC7200 integrated circuits, which function as time-to-digital converters for precise ultrasonic signal measurement and timing applications. The library is fully functional and specifically designed for Arduino development.
I created this fork to contribute additional examples and provide more detailed explanations for users who want to better understand and apply the existing libraries.

# Table of Contents
- Hardware Overview
- How It Works
- Hardware Connections
- Examples overview
- Future works for further contribution 
- Acknowledgement 
- References

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
## Channel Selection and Transducer Configuration
```
Physical Setup
     Transducer A                    Transducer B
     ┌─────────┐                    ┌─────────┐
     │   TX1   │────────────────────│   RX1   │
     │         │                    │         │
     │   RX2   │────────────────────│   TX2   │
     └─────────┘                    └─────────┘
         ↓                              ↓
      300pF cap                      300pF cap
         ↓                              ↓
      TDC1000                        TDC1000
    Channel pins                   Channel pins
```
**TDC1000 Measurement Modes**
The TDC1000 supports three TOF (Time-of-Flight) measurement modes:

Mode 0: Fluid Level and Identification

Single transducer acts as both TX and RX:
```
┌─────────┐
│  TX/RX  │ ──> ultrasound ──> [boundary/fluid surface]
└─────────┘ <── echo ←────────
```

Single transducer transmits, then switches to receive its own echo
Used for: Level measurement, fluid identification, boundary detection
Channel selection: Not applicable (single transducer)

Mode 1: Flow Sensing (Manual Channel Switching) 

```
Two separate transducers:
┌─────────┐                    ┌─────────┐
│   TX1   │ ──ultrasound────> │   RX2   │  Channel 1
│ (Xdcr A)│                    │ (Xdcr B)│
│         │                    │         │
│   RX1   │ <──ultrasound───── │   TX2   │  Channel 2
└─────────┘                    └─────────┘
```

- Transducer 1 transmits, Transducer 2 receives (through-transmission)
- Requires manual channel switching via software (setMeasureTOF())
- Used for: Bidirectional flow measurement, distance measurement
- Advantages: Full software control, predictable timing, easier debugging
- Channel selection: Software-controlled 

Mode 2: Flow Sensing (Automatic Channel Switching)
Same physical setup as Mode 1:
```
┌─────────┐                    ┌─────────┐
│   TX1   │ ──ultrasound────> │   RX2   │  Channel 1 (auto)
│ (Xdcr A)│                    │ (Xdcr B)│
│         │                    │         │
│   RX1   │ <──ultrasound───── │   TX2   │  Channel 2 (auto)
└─────────┘                    └─────────┘
```

- Same as Mode 1, but TDC1000 automatically swaps channels after each measurement
- Can be configured to swap: automatically after each measurement, or controlled by the external CHSEL pin
- Used for: Rapid bidirectional flow measurements
- Advantages: Faster updates, less software overhead
- Disadvantages: Less control, more complex timing coordination

# Hardware Connections
## Signal Flow Diagram 
```
Microcontroller              TDC1000                    TDC7200
     |                          |                          |
     |---SPI Config------------>|                          |
     |---SPI Config----------------------------------->|
     |                          |                          |
     |---TRIGGER (20µs)-------->|                          |
     |                          |                          |
     |                          |--START pulse------------>| (Timer starts)
     |                          |                          |
     |                          |--Ultrasonic burst---------------------------------->| (TX transducer)
     |                          |                          |
     |                          |<--Echo received-------------------------------------| (RX transducer)
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
```
     
## Transducer Setup for Modes 1 and 2
     [Transducer A]                    [Transducer B]
     TX1 ─────┐                        ┌───── TX2
              │                        │
     RX2 ──┐  │    <<<< Water >>>>    │  ┌── RX1
           │  │                        │  │
         [300pF]                    [300pF]
           │  │                        │  │
        To TDC1000              To TDC1000
**Important: 300pF capacitors provide AC coupling and are required for proper operation.**

# Getting started 
## Mode 0

## Mode 1 and 2
**The Measurement Sequence**

In modes 1 and 2, the system performs two sequential measurements for bidirectional flow detection:

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


**TDC7200 Measurement Modes**
I used Measurement Mode 2, which measuresthe time between consecutive stops in example (TDC1000_TDC7200_Integration):

Measurement Mode 2 Output:
```
  TIME1 = STOP1 - START      ← This is absolute TOF
  TIME2 = STOP2 - STOP1      ← Time between stops
  TIME3 = STOP3 - STOP2      ← Time between stops
  TIME4 = STOP4 - STOP3      ← Time between stops
  TIME5 = STOP5 - STOP4      ← Time between stops
```
For flow measurement, I use TIME1 (first STOP time).

### Flow Measurement Physics:

Time-of-Flight in Flowing Water

The presence of flow affects transit time asymmetrically:

```
Static Water (v = 0):
  TOF_AB = L / c
  TOF_BA = L / c
  TOF_AB = TOF_BA

With Flow (v ≠ 0):
  TOF_AB = L / (c + v)  ← Faster with flow
  TOF_BA = L / (c - v)  ← Slower against flow
  TOF_AB ≠ TOF_BA
Where:

L = distance between transducers (meters)
c = speed of sound in water (~1482 m/s at 20°C)
v = flow velocity (m/s)
```

**Flow Velocity Calculation"**
```
Simplified approximation (valid when v << c):
Δt = TOF_AB - TOF_BA

v ≈ (c² × Δt) / (2 × L)
More accurate formula (better for higher flows):
v = (c² / 2L) × [(TOF_BA - TOF_AB) / (TOF_BA × TOF_AB)]
```

**Flow Direction:**

```C++
double tof_diff = tof_ch1 - tof_ch2;

if (tof_diff > 0) {
    // TOF_AB > TOF_BA
    // Flow is AGAINST A→B direction
    // Therefore: Flow is in B→A direction
}
else if (tof_diff < 0) {
    // TOF_AB < TOF_BA
    // Flow is WITH A→B direction
    // Therefore: Flow is in A→B direction
}
else {
    // TOF_AB ≈ TOF_BA
    // Static water (no flow)
}
```
# Contributing
Contributions welcome! Areas for improvement:

- Temperature compensation
- Multi-path flow calculation
- Calibration routines
- Additional measurement modes
- Signal quality metrics

# License
GNU GENERAL PUBLIC LICENSE; Version 3, 29 June 2007

# Acknowledgments
- Thanks to [Yveaux's original repository](https://github.com/Yveaux/TDC1000)
- Texas Instruments for TDC1000/TDC7200 datasheets
- Arduino community for development tools

# References

- [TDC1000 Datasheet](https://github.com/Yveaux/TDC1000)
- [TDC7200 Datasheet](https://www.ti.com/lit/ds/symlink/tdc7200.pdf)
- [Ultrasonic Flow Measurement Theory](https://en.wikipedia.org/wiki/Ultrasonic_flow_meter)


