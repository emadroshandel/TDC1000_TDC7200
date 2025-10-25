# TDC1000 and TDC7200 for Arduino
This project is a fork of [Yveaux's original repository](https://github.com/Yveaux/TDC1000) for the TDC1000 and TDC7200 integrated circuits, which function as time-to-digital converters for precise ultrasonic signal measurement and timing applications. The library is fully functional and specifically designed for Arduino development.
I created this fork to contribute additional examples and provide more detailed explanations for users who want to better understand and apply the existing libraries.

# Table of Contents
- Hardware Overview
- How It Works
- Hardware Connections
- Examples overview
- Possible improvements
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

**Ultrasonic Transducers**

- For the water level application, a single transducer is enough
- For water flow applications, two Transducers are required 


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

This repository includes two fully functional and thoroughly commented examples. Each aims to comprehensively demonstrate the measurement capabilities of the TDC1000 and TDC7200 across a range of scenarios.

## Mode 0
The Water Level Monitor example showcases a practical application of the TDC1000 and TDC7200, configured for a single-transducer system to measure liquid levels.
This application uses ultrasonic echo reflection to detect the water level by:

- Sending an ultrasonic pulse downward through the air
- Receiving the echo reflected from the water surface
- Analyzing the echo frequency to determine the water level
- Tracking frequency changes over time

### Key TDC1000 Settings

**RX Configuration**

```C++
Water Level (Mode 0):
usafe.setRx(true);  // Multi-echo enabled

// Flow Measurement (Mode 1):
usafe.setRx(false);  // Single echo
```

Why multi-echo?

- Allows receiving multiple reflections (surface, bottom, walls)
- Provides richer frequency information
- Necessary for Mode 0 operation

**Sensitivity Settings**
```C++
// Water Level - Higher gain needed
usafe.setRxSensitivity(
    TDC1000::RxDacEchoThreshold::m410mV,
    TDC1000::RxPgaGain::g21dB,           
    TDC1000::RxLnaFbMode::resistive
);
```

Why higher gain?

- Echo is weaker (signal travels twice the distance)
- Energy lost at reflection
- Air attenuation

**TX Configuration**

```C++
// Water Level - Fewer pulses
usafe.setTx(TDC1000_CLKIN_FREQ_DIV, 8, 0, true);  // 8 cycles

// Flow - More pulses
usafe.setTx(TDC1000_CLKIN_FREQ_DIV, 10, 0, true);  // 10 cycles
```

Why fewer pulses?

- Shorter burst = better time resolution
- Echo arrives quickly (short distance)
- Reduces ringing interference

**Timing Windows**

```C++
//Water Level - Shorter times
usafe.setTofMeasuementShort(
    TDC1000::T0::ClkInDiv1,
    TDC1000::TxAutoZeroPeriod::T0x64,    
    TDC1000::TxBlankPeriod::T0x32,       
    TDC1000::TxEchoTimeoutPeriod::disabled  
);

// Flow - Longer times
usafe.setTofMeasuementShort(
    TDC1000::T0::ClkInDiv1,
    TDC1000::TxAutoZeroPeriod::T0x128,   // 16µs
    TDC1000::TxBlankPeriod::T0x128,      // 16µs
    TDC1000::TxEchoTimeoutPeriod::T0x1024  // 128µs
);
```

Why shorter times?

- Echo returns quickly in level sensing
- Shorter blanking = better for close surfaces
- Timeout disabled (level varies widely)

**Mode Selection**
```C++
// Water Level
usafe.setMeasureTOF(
    TDC1000::TxRxChannel::Channel1,
    TDC1000::TofMode::Mode0  // With setRx(true) = echo mode
);

// Flow
usafe.setMeasureTOF(
    TDC1000::TxRxChannel::Channel1,  // or Channel2
    TDC1000::TofMode::Mode1  // Manual channel switching
);
```

### Water level measurement

To use this for actual water level measurement, you must calibrate for your specific setup:

**Step 1: Establish Baseline Measurements**

- Prepare container: Clean, empty, dry
- Mount transducer: Fixed position, facing down
- Start system: Run water level monitor code
- Record empty frequency: Note the frequency (e.g., 179.2 kHz)

**Step 2: Measure Known Levels**

Fill the container to known levels and record frequencies:

```
Level | Height (cm) | Frequency (kHz) | Notes
------|-------------|-----------------|--------
Empty |      0      |     179.2       | Baseline
 25%  |      5      |     181.4       |
 50%  |     10      |     183.1       |
 75%  |     15      |     185.3       |
Full  |     20      |     186.8       | Maximum
Tips:
```

- Wait 10 seconds at each level for water to settle
- Record 10 measurements and average
- Mark the container with lines for repeatability

**Step 3: Create Calibration Function**

Option A: Lookup Table (Simple)

```C++
double estimateLevel(double freq_khz) {
    if (freq_khz < 180.0) return 0.0;       // Empty
    if (freq_khz < 182.0) return 25.0;      // Quarter
    if (freq_khz < 184.0) return 50.0;      // Half
    if (freq_khz < 186.0) return 75.0;      // Three-quarters
    return 100.0;                           // Full
}
```

Option B: Linear Interpolation (more continuous)

```C++
double estimateLevel(double freq_khz) {
    // Based on calibration data
    const double freq_empty = 179.2;  // Your empty frequency
    const double freq_full = 186.8;   // Your full frequency
    const double height_range = 20.0; // Container height in cm
    
    // Linear mapping
    double percent = (freq_khz - freq_empty) / (freq_full - freq_empty);
    percent = constrain(percent, 0.0, 1.0);  // Clamp to 0-100%
    
    return percent * height_range;  // Returns height in cm
}
```

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
I used Measurement Mode 2, which measures the time between consecutive stops in example (TDC1000_TDC7200_Integration):

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
  TOF_BA = L / (c - v)  ← Slower against the flow
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
## Mode 1 Hardware-based Channel Selection
This version uses the CHsel hardware pin to select channels instead of software configuration via SPI. The TDC1000 is configured once in "External" mode, then channel selection is controlled by toggling the CHsel pin.

CHsel Pin Operation:
```
CHsel = LOW  → Channel 1 (TX1→RX2, A→B)
CHsel = HIGH → Channel 2 (TX2→RX1, B→A)
 ```

Advantages of CHsel method:
- Hardware-controlled, more deterministic timing
- One less configuration step per measurement
- Specific situation when some external signal wants to set the direction of the transmitter and receiver

Example, "CHsel_Utilization.ino" shows the way that I have configured the device for hardware-level channel selection. 

# Possible improvements
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


