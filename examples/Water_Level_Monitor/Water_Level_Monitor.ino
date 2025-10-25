/*******************************************************************************
 * TDC1000/TDC7200 Water Level Monitor
 * 
 * This program uses ultrasonic reflections to detect water level in a container.
 * Unlike flow measurement, this uses a SINGLE transducer that both transmits
 * and receives (Mode 0).
 * 
 * MEASUREMENT PRINCIPLE:
 * ---------------------
 * Single transducer in echo/reflection mode:
 * 
 *    ┌─────────┐
 *    │  TX/RX  │ ──> ultrasound pulse ──> [water surface]
 *    │(Xdcr A) │ <── echo reflection <──
 *    └─────────┘
 *         ↑
 *      Air gap
 *         ↓
 *    ═══════════  ← Water surface (reflection point)
 *    ╎ Water   ╎
 *    ╎         ╎
 *    ╎         ╎
 *    ═══════════  ← Bottom
 * 
 * HOW IT WORKS:
 * -------------
 * 1. Transducer sends ultrasonic pulse downward
 * 2. Pulse travels through air at ~343 m/s
 * 3. Reflects off water surface (impedance change)
 * 4. Echo returns to same transducer
 * 5. Multiple zero-crossings in echo create STOP pulses
 * 6. Time between STOPs indicates echo frequency
 * 7. Frequency changes with water level (acoustic properties)
 * 
 * FREQUENCY vs WATER LEVEL:
 * -------------------------
 * The echo frequency varies with water level due to:
 * - Changes in acoustic path length
 * - Changes in acoustic impedance
 * - Multiple reflections (water surface + bottom)
 * 
 * Typical observations (for a drinking glass):
 *   Empty:     ~179 kHz
 *   Half full: ~182 kHz
 *   Full:      ~186 kHz
 * 
 * Your actual values will depend on:
 * - Container size and shape
 * - Transducer frequency (1 MHz typical)
 * - Distance to water surface
 * - Water properties (temperature, dissolved gases)
 * 
 * DIFFERENCES FROM FLOW MEASUREMENT:
 * -----------------------------------
 * Feature              Flow Meter (Mode 1)    Water Level (Mode 0)
 * --------             ------------------     --------------------
 * Transducers          2 (separate TX/RX)     1 (combined TX/RX)
 * Measurement type     Through-transmission   Echo/reflection
 * Signal path          TX → water → RX        TX → surface → RX
 * What's measured      Transit time           Echo frequency
 * TDC1000 mode         Mode 1 or Mode 2       Mode 0
 * TDC1000 RX config    setRx(false)           setRx(true) - multi-echo
 * Channel              Manual or auto swap    Channel 1 only
 * 
 * AUTHOR: Emad Roshandel
 * DATE: 26/10/2025
 * LICENSE: GNU GENERAL PUBLIC LICENSE; Version 3, 29 June 2007
 ******************************************************************************/

#include "TDC1000.h"
#include "TDC7200.h"

// ═════════════════════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═════════════════════════════════════════════════════════════════════════════

#define PIN_TDC1000_TRIGGER 0
#define PIN_TDC1000_RESET 1 
#define PIN_TDC7200_INT 2
#define ERRB 3
#define PIN_TDC7200_TRIGGER 4
#define PIN_TDC7200_ENABLE 5 
#define RedLED 6
#define GreenLED 7
#define CHsel 12
#define PIN_TDC7200_SPI_CS 13
#define PIN_TDC1000_SPI_CS 14
#define Dout 15
#define PIN_TDC7200_STOP 16
#define PIN_TDC7200_START 17

//////////////////////////////////////////////////////
#define NUM_STOPS (5)

// External clock is 8 MHz
#define TDC1000_CLKIN_FREQ_HZ  (8000000UL)
#define TDC1000_CLKIN_FREQ_DIV (TDC1000::TxFreqDivider::Div8)
#define TDC7200_CLOCK_FREQ_HZ  (8000000UL)
// ═════════════════════════════════════════════════════════════════════════════
// HARDWARE INITIALIZATION
// ═════════════════════════════════════════════════════════════════════════════

// TDC7200: Time-to-Digital Converter (captures timing of echo zero-crossings)
static TDC7200 tof(PIN_TDC7200_ENABLE, PIN_TDC7200_SPI_CS, TDC7200_CLOCK_FREQ_HZ);

// TDC1000: Ultrasonic AFE (drives transducer, receives echo, generates STOPs)
static TDC1000 usafe(PIN_TDC1000_SPI_CS, PIN_TDC1000_RESET);

// ═════════════════════════════════════════════════════════════════════════════
// STATISTICS AND TRACKING
// ═════════════════════════════════════════════════════════════════════════════

#define NUM_SAMPLES 10  // Number of measurements to keep for statistics

// Storage for rolling window of measurements
// [stop_number][sample_number]
uint64_t tofSamples[NUM_STOPS][NUM_SAMPLES];
uint8_t sampleIndex = 0;  // Current position in circular buffer

uint32_t successCount = 0;  // Count of successful measurements
uint32_t errorCount = 0;    // Count of failed measurements

// Real-time frequency tracking variables
uint64_t lastTOF1 = 0;      // Previous measurement's first STOP time
uint64_t lastTOF2 = 0;      // Previous measurement's second STOP time
bool firstMeasurement = true;  // Flag to skip delta calculation on first run

// ═════════════════════════════════════════════════════════════════════════════
// REAL-TIME FREQUENCY DISPLAY
// ═════════════════════════════════════════════════════════════════════════════

/*******************************************************************************
 * displayRealtimeFrequency()
 * 
 * Calculates and displays the echo frequency based on time between STOPs.
 * 
 * FREQUENCY CALCULATION:
 * ----------------------
 * The echo signal is an oscillating waveform. Each zero-crossing generates a
 * STOP pulse. The time between consecutive STOPs is one period.
 * 
 *   Period (T) = TOF2 - TOF1  (time between first and second STOP)
 *   Frequency (f) = 1 / Period
 * 
 * Example:
 *   TOF1 = 28.456 µs
 *   TOF2 = 33.912 µs
 *   Period = 5.456 µs
 *   Frequency = 1 / 5.456µs = 183.3 kHz
 * 
 * WHAT THE FREQUENCY TELLS US:
 * ----------------------------
 * The echo frequency changes with water level because:
 * 1. Different path lengths create different resonances
 * 2. Water surface acts as acoustic reflector
 * 3. Container geometry affects standing waves
 * 
 * By monitoring frequency changes, we can detect water level changes without
 * needing to know absolute distance.
 * 
 * PARAMETERS:
 *   tof1 - Time of first STOP pulse (picoseconds)
 *   tof2 - Time of second STOP pulse (picoseconds)
 ******************************************************************************/
void displayRealtimeFrequency(uint64_t tof1, uint64_t tof2) {
    // Validate inputs
    if (tof1 == 0 || tof2 == 0 || tof2 <= tof1) {
        Serial.println(F("  [Freq calc error]"));
        return;
    }
    
    // Calculate period (time between zero-crossings)
    uint64_t period = tof2 - tof1;  // In picoseconds
    
    // Calculate frequency
    // freq = 1 / period
    // period is in picoseconds, so convert to seconds: period / 1,000,000 µs
    // Result is in Hz
    double freq = 1000000.0 / (period / 1000000.0);
    
    Serial.print(F("  Frequency: "));
    Serial.print(freq / 1000.0, 2);  // Display in kHz
    Serial.print(F(" kHz"));
    
    // ─────────────────────────────────────────────────────────────────────────
    // Show change from previous measurement
    // ─────────────────────────────────────────────────────────────────────────
    // This helps detect trends (water level rising/falling)
    
    if (!firstMeasurement) {
        // Calculate previous frequency
        uint64_t lastPeriod = lastTOF2 - lastTOF1;
        double lastFreq = 1000000.0 / (lastPeriod / 1000000.0);
        
        // Calculate change
        double freqChange = freq - lastFreq;
        
        // Display change with +/- sign
        Serial.print(F("  ("));
        if (freqChange > 0) Serial.print(F("+"));
        Serial.print(freqChange / 1000.0, 2);
        Serial.print(F(" kHz)"));
        
        // Interpretation hints (optional - adjust thresholds for your setup)
        if (freqChange > 0.5) {
            Serial.print(F(" ↑ Level rising"));
        } else if (freqChange < -0.5) {
            Serial.print(F(" ↓ Level falling"));
        }
    }
    
    Serial.println();
    
    // Store for next comparison
    lastTOF1 = tof1;
    lastTOF2 = tof2;
    firstMeasurement = false;
}

// ═════════════════════════════════════════════════════════════════════════════
// STATISTICAL ANALYSIS
// ═════════════════════════════════════════════════════════════════════════════

/*******************************************************************************
 * calculateStatistics()
 * 
 * Analyzes the last NUM_SAMPLES measurements to provide:
 * - Average frequency and range (for water level correlation)
 * - Mean TOF for each STOP (for distance calculation)
 * - Jitter (measurement stability indicator)
 * - Success rate (overall system performance)
 * 
 * This function is called every 10 successful measurements to provide
 * periodic system health checks.
 * 
 * STATISTICS EXPLAINED:
 * ---------------------
 * 1. Frequency statistics:
 *    - Average: Typical frequency for current water level
 *    - Range: Shows frequency span (wider = less stable or changing level)
 *    - Stability: ±deviation indicates measurement consistency
 * 
 * 2. TOF statistics:
 *    - Mean: Average time for each STOP
 *    - Jitter: Time variation (should be <100 ns for good signal)
 * 
 * 3. Success rate:
 *    - Percentage of successful measurements
 *    - Should be >90% for reliable operation
 ******************************************************************************/
void calculateStatistics() {
    // Need full buffer before calculating stats
    if (sampleIndex < NUM_SAMPLES) return;
    
    Serial.println(F("\n=== STATISTICS (last 10 measurements) ==="));
    
    // ─────────────────────────────────────────────────────────────────────────
    // Calculate frequency statistics
    // ─────────────────────────────────────────────────────────────────────────
    
    double avgFreq = 0;
    double minFreq = 999999;
    double maxFreq = 0;
    uint8_t validFreqSamples = 0;
    
    // Calculate frequency for each sample
    for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
        // Need TOF2 > TOF1 for valid frequency calculation
        if (tofSamples[1][i] > tofSamples[0][i]) {
            uint64_t period = tofSamples[1][i] - tofSamples[0][i];
            double freq = 1000000.0 / (period / 1000000.0);
            
            avgFreq += freq;
            if (freq < minFreq) minFreq = freq;
            if (freq > maxFreq) maxFreq = freq;
            validFreqSamples++;
        }
    }
    
    // Display frequency statistics
    if (validFreqSamples > 0) {
        avgFreq /= validFreqSamples;
        
        Serial.print(F("Average Frequency: "));
        Serial.print(avgFreq / 1000.0, 2);
        Serial.println(F(" kHz"));
        
        Serial.print(F("Frequency Range: "));
        Serial.print(minFreq / 1000.0, 2);
        Serial.print(F(" to "));
        Serial.print(maxFreq / 1000.0, 2);
        Serial.println(F(" kHz"));
        
        // Stability = ±half of range
        Serial.print(F("Frequency Stability: ±"));
        Serial.print((maxFreq - minFreq) / 2000.0, 2);
        Serial.println(F(" kHz\n"));
        
        // Interpretation guide
        double stability_khz = (maxFreq - minFreq) / 2000.0;
        if (stability_khz < 0.5) {
            Serial.println(F("  Status: EXCELLENT stability"));
        } else if (stability_khz < 1.0) {
            Serial.println(F("  Status: GOOD stability"));
        } else if (stability_khz < 2.0) {
            Serial.println(F("  Status: FAIR stability"));
        } else {
            Serial.println(F("  Status: POOR stability - check setup\n"));
        }
    }
    
    // ─────────────────────────────────────────────────────────────────────────
    // Calculate TOF statistics for each STOP
    // ─────────────────────────────────────────────────────────────────────────
    
    for (uint8_t stop = 0; stop < NUM_STOPS; stop++) {
        // Calculate mean
        uint64_t sum = 0;
        for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
            sum += tofSamples[stop][i];
        }
        uint64_t mean = sum / NUM_SAMPLES;
        
        // Calculate variance and standard deviation
        // Note: This is a simplified calculation suitable for embedded systems
        uint64_t variance = 0;
        for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
            int64_t diff = (int64_t)tofSamples[stop][i] - (int64_t)mean;
            variance += (diff * diff) / NUM_SAMPLES;
        }
        uint32_t stddev = sqrt(variance);
        
        // Find min/max for jitter calculation
        uint64_t minVal = tofSamples[stop][0];
        uint64_t maxVal = tofSamples[stop][0];
        for (uint8_t i = 1; i < NUM_SAMPLES; i++) {
            if (tofSamples[stop][i] < minVal) minVal = tofSamples[stop][i];
            if (tofSamples[stop][i] > maxVal) maxVal = tofSamples[stop][i];
        }
        
        // Display statistics for this STOP
        Serial.print(F("TOF")); 
        Serial.print(stop + 1); 
        Serial.println(F(":"));
        
        Serial.print(F("  Mean:   ")); 
        Serial.print((double)mean / 1000000.0, 3);
        Serial.println(F(" us"));
        
        // Jitter = peak-to-peak variation
        Serial.print(F("  Jitter: ")); 
        Serial.print((double)(maxVal - minVal) / 1000.0, 1);
        Serial.print(F(" ns"));
        
        // Jitter interpretation
        double jitter_ns = (double)(maxVal - minVal) / 1000.0;
        if (jitter_ns < 50) {
            Serial.println(F(" (excellent)"));
        } else if (jitter_ns < 100) {
            Serial.println(F(" (good)"));
        } else if (jitter_ns < 200) {
            Serial.println(F(" (fair)"));
        } else {
            Serial.println(F(" (poor - check signal)"));
        }
        
        Serial.println();
    }
    
    // ─────────────────────────────────────────────────────────────────────────
    // Display overall success rate
    // ─────────────────────────────────────────────────────────────────────────
    
    Serial.print(F("Success rate: "));
    Serial.print(successCount);
    Serial.print(F("/"));
    Serial.print(successCount + errorCount);
    Serial.print(F(" ("));
    Serial.print(100.0 * successCount / (successCount + errorCount), 1);
    Serial.print(F("%)"));
    
    // Success rate interpretation
    double success_pct = 100.0 * successCount / (successCount + errorCount);
    if (success_pct >= 95) {
        Serial.println(F(" ✓ Excellent"));
    } else if (success_pct >= 85) {
        Serial.println(F(" - Good"));
    } else if (success_pct >= 70) {
        Serial.println(F(" ⚠ Fair - may need tuning"));
    } else {
        Serial.println(F(" ✗ Poor - check hardware"));
    }
    
    Serial.println(F("\n==========================================\n"));
}

// ═════════════════════════════════════════════════════════════════════════════
// SETUP - Run once at startup
// ═════════════════════════════════════════════════════════════════════════════

void setup()
{
    Serial.begin(115200);
    delay(100);

    // ─────────────────────────────────────────────────────────────────────────
    // Configure GPIO pins
    // ─────────────────────────────────────────────────────────────────────────
    
    pinMode(PIN_TDC1000_TRIGGER, OUTPUT);
    digitalWrite(PIN_TDC1000_TRIGGER, LOW);
    
    // TDC7200 INT pin is OUTPUT from chip (we read it to detect completion)
    pinMode(PIN_TDC7200_INT, INPUT_PULLUP);

    Serial.println(F("=== TDC Water Level Monitor ===\n"));
    
    // ─────────────────────────────────────────────────────────────────────────
    // Initialize TDC1000 (Ultrasonic Analog Front End)
    // ─────────────────────────────────────────────────────────────────────────
    
    Serial.println(F("Initializing TDC1000..."));
    if (!usafe.begin()) {
        Serial.println(F("Failed to init TDC1000"));
        while (1);  // Halt on failure
    }

    bool ok = true;  // Track configuration success
    
    // Configure trigger edge (rising = true)
    ok &= usafe.setTriggerEdge(true);
    
    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ TX Configuration: OPTIMIZED FOR WATER LEVEL DETECTION              │
    // └─────────────────────────────────────────────────────────────────────┘
    // Parameters: (clock_divider, num_pulses, damping, short_burst)
    //   - 8 pulses: Fewer than flow measurement (10) but enough energy
    //               for echo detection. Shorter burst = better resolution.
    //   - Frequency: 8MHz / 8 = 1MHz (typical transducer resonance)
    
    ok &= usafe.setTx(TDC1000_CLKIN_FREQ_DIV, 8, 0, true);
    
    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ RX Configuration: MULTI-ECHO MODE                                   │
    // └─────────────────────────────────────────────────────────────────────┘
    // setRx(true) = Enable multi-echo reception
    //   - Allows receiving multiple reflections
    //   - Necessary for Mode 0 (echo/reflection mode)
    //   - Different from flow measurement which uses setRx(false)
    //
    // Why multi-echo?
    //   - Water surface reflection (primary echo)
    //   - Bottom reflection (secondary echo)
    //   - Side wall reflections (if container is narrow)
    //   - Multiple reflections provide richer signal information
    
    ok &= usafe.setRx(true);  // true = multi-echo for Mode 0
    
    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ Sensitivity Configuration: TUNED FOR ECHO DETECTION                 │
    // └─────────────────────────────────────────────────────────────────────┘
    // Echo signals are typically weaker than through-transmission signals
    // because:
    //   1. Signal travels twice the distance (down and back)
    //   2. Energy is lost at reflection
    //   3. Air-water interface reflects only ~99.9% (some passes through)
    //
    // Current settings (Option 1): Balanced for typical water level sensing
    
    ok &= usafe.setRxSensitivity(
        TDC1000::RxDacEchoThreshold::m410mV,  // Moderate threshold
        TDC1000::RxPgaGain::g21dB,            // Higher gain than flow (was g15dB)
        TDC1000::RxLnaFbMode::resistive       // Resistive feedback for echo
    );
    
    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ ALTERNATIVE SENSITIVITY SETTINGS (uncomment to test)                │
    // └─────────────────────────────────────────────────────────────────────┘
    
    // Option 2: Maximum sensitivity for weak echoes
    // Use if you get "Signal weak" or "No signal" errors
    // ok &= usafe.setRxSensitivity(
    //     TDC1000::RxDacEchoThreshold::m410mV, 
    //     TDC1000::RxPgaGain::g21dB,           // Maximum gain
    //     TDC1000::RxLnaFbMode::capacitive     // Capacitive for max sensitivity
    // );
    
    // Option 3: Lower threshold for strong echoes
    // Use if you get "Signal high" errors
    // ok &= usafe.setRxSensitivity(
    //     TDC1000::RxDacEchoThreshold::m220mV,  // Lower threshold (more sensitive)
    //     TDC1000::RxPgaGain::g21dB, 
    //     TDC1000::RxLnaFbMode::resistive
    // );
    
    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ STOP Configuration: Auto-stop for Mode 0                            │
    // └─────────────────────────────────────────────────────────────────────┘
    // Parameters: (tx_rx_cycles, num_stops)
    //   - x1: Single TX burst per measurement
    //   - 0: Auto-stop count (TDC1000 decides based on echo)
    //
    // In Mode 0, we let TDC1000 automatically determine when echo is complete
    // rather than specifying a fixed number of stops like in Mode 1.
    
    ok &= usafe.setRepeat(TDC1000::TxRxCycles::x1, 0);  // 0 = auto-stop
    
    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ Timing Configuration: OPTIMIZED FOR ECHO MODE                       │
    // └─────────────────────────────────────────────────────────────────────┘
    // These timings are adjusted for echo/reflection measurements:
    //
    // TxAutoZeroPeriod: T0x64 (8µs)
    //   - Shorter than flow mode (was T0x128)
    //   - Faster settling for quicker measurements
    //
    // TxBlankPeriod: T0x32 (4µs)
    //   - Shorter blanking period
    //   - Echo arrives quickly in water level sensing
    //
    // TxEchoTimeoutPeriod: disabled
    //   - Let TDC1000 wait indefinitely for echo
    //   - Necessary when water level varies widely
    //   - Timeout handled by microcontroller (150ms in loop)
    
    ok &= usafe.setTofMeasuementShort(
        TDC1000::T0::ClkInDiv1,                      // Base timing: 125ns
        TDC1000::TxAutoZeroPeriod::T0x64,            // 8µs settling
        TDC1000::TxBlankPeriod::T0x32,               // 4µs blanking
        TDC1000::TxEchoTimeoutPeriod::disabled       // No hardware timeout
    );
    
    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ Mode Configuration: MODE 0 for Echo/Reflection                      │
    // └─────────────────────────────────────────────────────────────────────┘
    // setMeasureTOF(channel, mode):
    //   - Channel1: Only one channel needed (same transducer TX and RX)
    //   - Mode2: This is confusing naming! Despite saying "Mode2" here,
    //            we're actually using TDC1000's Mode 0 (echo mode)
    //
    // TDC1000 Mode clarification:
    //   TDC1000::TofMode::Mode0 = echo mode 
    //   TDC1000::TofMode::Mode1 = Through-transmission, manual channel switch
    //   TDC1000::TofMode::Mode2 = Through-transmission, Automatic channel switch
    //
    // Since we configured setRx(true) for multi-echo, Mode0 operates as
    // echo/reflection mode (TDC1000's actual Mode 0).
    
    ok &= usafe.setMeasureTOF(
        TDC1000::TxRxChannel::Channel1,  // Only need one channel
        TDC1000::TofMode::Mode0          // With setRx(true) = echo mode
    );

    // Check if all configuration steps succeeded
    if (!ok) {
        Serial.println(F("TDC1000 config failed"));
        while (1);
    }
    Serial.println(F("TDC1000 OK\n"));

    // ─────────────────────────────────────────────────────────────────────────
    // Initialize TDC7200 (Time-to-Digital Converter)
    // ─────────────────────────────────────────────────────────────────────────
    
    Serial.println(F("Initializing TDC7200..."));
    if (!tof.begin()) {
        Serial.println(F("Failed to init TDC7200"));
        while (1);
    }

    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ TDC7200 Measurement Setup                                           │
    // └─────────────────────────────────────────────────────────────────────┘
    // Parameters: (calibration2_periods, avg_cycles, num_stop, meas_mode)
    //   - calibration2_periods: 10 (standard)
    //   - avg_cycles: 1 (no averaging, single shot)
    //   - num_stop: NUM_STOPS (5) - capture 5 zero-crossings of echo
    //   - meas_mode: 2 = Measurement Mode 2 (time between stops)
    //
    // We capture 5 stops to:
    //   1. Calculate echo frequency (TOF2 - TOF1 = period)
    //   2. Verify signal quality (all 5 should be present)
    //   3. Track consistency (period should be uniform)
    
    if (!tof.setupMeasurement(10, 1, NUM_STOPS, 2)) {
        Serial.println(F("Failed to setup TDC7200"));
        while (1);
    }
    Serial.println(F("TDC7200 OK\n"));

    // ─────────────────────────────────────────────────────────────────────────
    // System Ready - Print usage information
    // ─────────────────────────────────────────────────────────────────────────
    
    Serial.println(F("System ready. Monitoring water level...\n"));
    Serial.println(F("Legend: Empty glass ~179kHz, Half full ~182kHz, Full ~186kHz\n"));
    Serial.println(F("NOTE: Your frequencies will differ based on:"));
    Serial.println(F("  - Container size and shape"));
    Serial.println(F("  - Transducer mounting height"));
    Serial.println(F("  - Water temperature"));
    Serial.println(F("  - Transducer frequency\n"));
    Serial.println(F("Establish your baseline by measuring known levels!\n"));
    
    delay(1000);
}

// ═════════════════════════════════════════════════════════════════════════════
// MAIN LOOP - Continuously measure water level via echo frequency
// ═════════════════════════════════════════════════════════════════════════════

void loop()
{
    // ─────────────────────────────────────────────────────────────────────────
    // Reset TDC1000 state before each measurement
    // ─────────────────────────────────────────────────────────────────────────
    
    usafe.clearErrorFlags();       // Clear any previous error conditions
    usafe.resetStatemachine();     // Reset TDC1000 internal state machine
    delay(1);  // Brief settling time

    // ─────────────────────────────────────────────────────────────────────────
    // Arm TDC7200 for measurement
    // ─────────────────────────────────────────────────────────────────────────
    // This prepares TDC7200 to capture STOP pulses
    // INT pin goes HIGH (waiting for STOPs)
    
    tof.startMeasurement();
    delayMicroseconds(10);

    // ─────────────────────────────────────────────────────────────────────────
    // Trigger ultrasonic pulse
    // ─────────────────────────────────────────────────────────────────────────
    // Send 10µs pulse to TDC1000 TRIGGER pin to initiate measurement
    // 
    // What happens next (automatic hardware sequence):
    //   1. TDC1000 sends START pulse to TDC7200
    //   2. TDC1000 drives transducer with 8-cycle burst
    //   3. Ultrasound propagates through air (~343 m/s)
    //   4. Reflects off water surface
    //   5. Echo returns to same transducer
    //   6. TDC1000 detects echo, generates STOP pulses at zero-crossings
    //   7. TDC7200 captures time of each STOP
    //   8. TDC7200 sets INT LOW when all STOPs captured
    
    digitalWrite(PIN_TDC1000_TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TDC1000_TRIGGER, LOW);

    // ─────────────────────────────────────────────────────────────────────────
    // Wait for measurement completion
    // ─────────────────────────────────────────────────────────────────────────
    // Poll INT pin until it goes LOW (measurement complete)
    // 
    // INT Pin states:
    //   HIGH = TDC7200 still waiting for STOP pulses
    //   LOW  = All STOPs captured, ready to read results
    //
    // Timeout: 100ms (shorter than flow measurement because echo arrives faster)
    
    unsigned long t0 = millis();
    while (digitalRead(PIN_TDC7200_INT) == HIGH) {
        if (millis() - t0 > 100) {
            // Timeout - measurement failed
            Serial.println(F("TIMEOUT"));
            errorCount++;
            
            // Check TDC1000 error flags to diagnose problem
            bool sigWeak, noSig, sigHigh;
            usafe.getErrorFlags(sigWeak, noSig, sigHigh);
            
            if (sigWeak) {
                Serial.println(F("  Error: Signal weak"));
                Serial.println(F("    -> Try increasing gain (g27dB)"));
                Serial.println(F("    -> Check transducer alignment"));
                Serial.println(F("    -> Verify water is present"));
            }
            if (noSig) {
                Serial.println(F("  Error: No signal"));
                Serial.println(F("    -> Check if transducer is working"));
                Serial.println(F("    -> Verify wiring connections"));
                Serial.println(F("    -> Ensure water surface is within range"));
            }
            if (sigHigh) {
                Serial.println(F("  Error: Signal high"));
                Serial.println(F("    -> Reduce gain or increase threshold"));
                Serial.println(F("    -> Transducer may be too close to surface"));
            }
            
            delay(1000);  // Wait before retry
            return;
        }
    }
    
    // If we reach here, INT went LOW = measurement successful!

    // ─────────────────────────────────────────────────────────────────────────
    // Read measurement results from TDC7200
    // ─────────────────────────────────────────────────────────────────────────
    
    uint64_t currentTOF[NUM_STOPS];  // Array to store all STOP times
    bool allSuccess = true;           // Track if all STOPs valid
    
    Serial.print(F("Measurement #")); 
    Serial.print(successCount + 1);
    Serial.println(F(":"));
    
    // Read all STOP times (1 through NUM_STOPS)
    for (uint8_t stop = 1; stop <= NUM_STOPS; stop++) {
        uint64_t time;
        
        if (tof.readMeasurement(stop, time)) {
            // Measurement read successful
            currentTOF[stop - 1] = time;  // Store in 0-indexed array
            
            // ┌─────────────────────────────────────────────────────────────┐
            // │ Update rolling buffer for statistics                       │
            // └─────────────────────────────────────────────────────────────┘
            // We maintain a circular buffer of the last NUM_SAMPLES measurements
            // This allows us to calculate statistics without storing everything
            
            if (sampleIndex < NUM_SAMPLES) {
                // Buffer not yet full - simply append
                tofSamples[stop - 1][sampleIndex] = time;
            } else {
                // Buffer full - shift left and add new value at end
                for (uint8_t i = 0; i < NUM_SAMPLES - 1; i++) {
                    tofSamples[stop - 1][i] = tofSamples[stop - 1][i + 1];
                }
                tofSamples[stop - 1][NUM_SAMPLES - 1] = time;
            }
            
            // Display this STOP time
            Serial.print(F("  TOF")); 
            Serial.print(stop);
            Serial.print(F(" = "));
            Serial.print((double)time / 1000000.0, 3);  // Convert ps to µs
            Serial.println(F(" us"));
            
        } else {
            // Measurement read failed
            allSuccess = false;
            Serial.print(F("  TOF")); 
            Serial.print(stop);
            Serial.println(F(" = ERROR"));
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Calculate and display real-time frequency
    // ─────────────────────────────────────────────────────────────────────────
    // Frequency is the key indicator for water level
    // 
    // We use TOF1 and TOF2 (first two stops) because:
    //   - They're most consistent
    //   - Period between them represents echo frequency
    //   - First zero-crossings have strongest signal
    
    if (allSuccess && NUM_STOPS >= 2) {
        displayRealtimeFrequency(currentTOF[0], currentTOF[1]);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Update statistics and display periodic summary
    // ─────────────────────────────────────────────────────────────────────────
    
    if (allSuccess) {
        successCount++;
        
        // Update sample index (circular buffer)
        if (sampleIndex < NUM_SAMPLES) {
            sampleIndex++;
        }
        
        // Display detailed statistics every 10 successful measurements
        // This provides periodic system health checks without cluttering output
        if (sampleIndex >= NUM_SAMPLES && (successCount % 10) == 0) {
            calculateStatistics();
        }
    } else {
        errorCount++;
        Serial.println(F("  Some STOPs failed - check signal quality\n"));
    }
    
    Serial.println();
    
    // ─────────────────────────────────────────────────────────────────────────
    // Delay before next measurement
    // ─────────────────────────────────────────────────────────────────────────
    // 500ms = ~2 Hz update rate
    // Adjust based on your needs:
    //   - Faster: 200-300ms for more responsive monitoring
    //   - Slower: 1000ms+ for less frequent updates
    //   - Consider water dynamics: fast changes need fast updates
    
    delay(500);
}


// ═════════════════════════════════════════════════════════════════════════════
// UINT64 TO ASCII CONVERSION
// ═════════════════════════════════════════════════════════════════════════════

/*******************************************************************************
 * ui64toa() - Convert 64-bit unsigned integer to ASCII string
 * 
 * This function converts a uint64_t value to a string representation in the
 * specified base (typically base 10 for decimal).
 * 
 * WHY THIS IS NEEDED:
 * -------------------
 * The TDC7200 returns timing values as 64-bit integers (picoseconds). Standard
 * Arduino functions like String() or sprintf() don't handle uint64_t well on
 * all platforms, leading to truncation or incorrect output.
 * 
 * This custom implementation ensures proper handling of the full 64-bit range.
 * 
 * ALGORITHM:
 * ----------
 * 1. Extract digits by repeated division and modulo
 * 2. Store digits in reverse order (least significant first)
 * 3. Null-terminate the string
 * 4. Reverse the character array to get correct order
 * 
 * USAGE EXAMPLE:
 * --------------
 * uint64_t tof = 28456789012;  // Time in picoseconds
 * char buffer[32];
 * ui64toa(tof, buffer, 10);    // Convert to decimal string
 * Serial.println(buffer);       // Output: "28456789012"
 * 
 * PARAMETERS:
 * -----------
 * @param[in]  v     Value to convert (uint64_t)
 * @param[out] buf   Output buffer (must be large enough for result + null)
 * @param[in]  base  Numeric base (10 for decimal, 16 for hex, etc.)
 * 
 * BUFFER SIZE REQUIREMENTS:
 * -------------------------
 * Base 10: Maximum 20 digits + null terminator = 21 bytes minimum
 * Base 16: Maximum 16 digits + null terminator = 17 bytes minimum
 * Recommended: 32 bytes for safety
 * 
 * LIMITATIONS:
 * ------------
 * - No error checking for buffer overflow
 * - Assumes base <= 10 (only adds '0' to digit, doesn't handle A-F for hex)
 * - For hex/other bases, would need: buf[idx++] = "0123456789ABCDEF"[digit]
 * 
 ******************************************************************************/
static void ui64toa(uint64_t v, char * buf, uint8_t base)
{
    int idx = 0;      // Current position in buffer
    uint64_t w = 0;   // Quotient from division
    
    // ─────────────────────────────────────────────────────────────────────────
    // STEP 1: Extract digits in reverse order
    // ─────────────────────────────────────────────────────────────────────────
    // Algorithm: Repeatedly divide by base and extract remainder
    // Example for v=123, base=10:
    //   Iteration 1: 123/10 = 12 remainder 3 → buf[0] = '3'
    //   Iteration 2: 12/10  = 1  remainder 2 → buf[1] = '2'
    //   Iteration 3: 1/10   = 0  remainder 1 → buf[2] = '1'
    //   Result: buf = "321" (reversed)
    
    while (v > 0)
    {
        w = v / base;                      // Quotient (next iteration value)
        buf[idx++] = (v - w * base) + '0'; // Remainder = digit, convert to ASCII
        v = w;                              // Continue with quotient
    }
    
    // ─────────────────────────────────────────────────────────────────────────
    // STEP 2: Null-terminate the string
    // ─────────────────────────────────────────────────────────────────────────
    buf[idx] = 0;  // Add null terminator at end
    
    // ─────────────────────────────────────────────────────────────────────────
    // STEP 3: Reverse the character array
    // ─────────────────────────────────────────────────────────────────────────
    // We extracted digits backward, now flip them to the correct order
    // Example: "321" → "123"
    //
    // Algorithm: Swap characters from outside-in
    //   i starts at 0 (left end)
    //   j starts at idx-1 (right end)
    //   Swap buf[i] with buf[j]
    //   Move i right, j left
    //   Continue until they meet in middle
    
    for (int i = 0, j = idx - 1; i < idx / 2; i++, j--)
    {
        char c = buf[i];    // Temporarily store left character
        buf[i] = buf[j];    // Copy right to left
        buf[j] = c;         // Copy temp (original left) to right
    }
}

