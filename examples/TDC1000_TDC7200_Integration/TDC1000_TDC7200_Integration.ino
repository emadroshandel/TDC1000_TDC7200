/*******************************************************************************
 * TDC1000/TDC7200 Bidirectional Ultrasonic Flow Meter
 * 
 * This program measures time-of-flight (TOF) of ultrasonic pulses through water
 * in both directions to enable flow detection and distance measurement.
 * 
 * MEASUREMENT PRINCIPLE:
 * ---------------------
 * Two measurements are performed sequentially:
 * 
 * 1. Channel 1 (A→B): TX1 transmits, RX2 receives
 *    ┌─────────┐  Ultrasound   ┌─────────┐
 *    │  TX1    │───────────────>│   RX2   │
 *    │ (Xdcr A)│                │ (Xdcr B)│
 *    └─────────┘                └─────────┘
 * 
 * 2. Channel 2 (B→A): TX2 transmits, RX1 receives  
 *    ┌─────────┐  Ultrasound   ┌─────────┐
 *    │  RX1    │<───────────────│   TX2   │
 *    │ (Xdcr A)│                │ (Xdcr B)│
 *    └─────────┘                └─────────┘
 * 
 * In static water: TOF_AB ≈ TOF_BA (should be nearly equal)
 * With flow: The TOF difference indicates flow direction and velocity
 * 
 * HARDWARE COORDINATION:
 * ----------------------
 * The TDC1000 and TDC7200 work together automatically:
 * 
 * TDC1000 (Analog Front End):
 *   - Drives ultrasonic transducers
 *   - Receives and conditions echo signals
 *   - Generates START pulse when transmission begins
 *   - Generates STOP pulses for each zero-crossing of received signal
 * 
 * TDC7200 (Time-to-Digital Converter):
 *   - Starts internal timer on START pulse
 *   - Captures time of each STOP pulse (up to 5)
 *   - Signals completion via INT pin (goes LOW when done)
 * 
 * Microcontroller:
 *   - Configures both chips via SPI
 *   - Triggers measurements
 *   - Polls INT pin to detect completion
 *   - Reads timing results via SPI
 * 
 * AUTHOR: Emad Roshandel
 * DATE: 25/10/2025
 * LICENSE: GNU GENERAL PUBLIC LICENSE
 ******************************************************************************/

#include "TDC1000.h"  
#include "TDC7200.h"  

// Pin definitions for the considered processor
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

#define NUM_STOPS (5)

// External clock is 8 MHz
#define TDC1000_CLKIN_FREQ_HZ  (8000000UL)
#define TDC1000_CLKIN_FREQ_DIV (TDC1000::TxFreqDivider::Div8)
#define TDC7200_CLOCK_FREQ_HZ  (8000000UL)
// ═════════════════════════════════════════════════════════════════════════════
// DEBUG CONFIGURATION
// ═════════════════════════════════════════════════════════════════════════════

#define DEBUG_OUTPUT true       // Enable serial output of measurement details
#define SHOW_RAW_STOPS false    // Show all 5 STOP times (useful for debugging)

// ═════════════════════════════════════════════════════════════════════════════
// HARDWARE INITIALIZATION
// ═════════════════════════════════════════════════════════════════════════════

// TDC7200: Time-to-Digital Converter (measures time intervals with ps precision)
static TDC7200 tof(PIN_TDC7200_ENABLE, PIN_TDC7200_SPI_CS, TDC7200_CLOCK_FREQ_HZ);

// TDC1000: Ultrasonic Sensing Analog Front End (drives transducers, receives echoes)
static TDC1000 usafe(PIN_TDC1000_SPI_CS, PIN_TDC1000_RESET);

// ═════════════════════════════════════════════════════════════════════════════
// STATISTICS TRACKING
// ═════════════════════════════════════════════════════════════════════════════

uint32_t successCount = 0;  // Count of successful bidirectional measurements
uint32_t errorCount = 0;    // Count of failed or partial measurements

// ═════════════════════════════════════════════════════════════════════════════
// MEASUREMENT FUNCTION
// ═════════════════════════════════════════════════════════════════════════════

/*******************************************************************************
 * measureOneDirection()
 * 
 * Performs a single time-of-flight measurement in one direction.
 * This function handles the complete measurement sequence:
 * 1. Configure TDC1000 for the desired channel
 * 2. Arm TDC7200 for measurement
 * 3. Trigger ultrasonic burst
 * 4. Wait for measurement completion
 * 5. Read and validate results
 * 
 * PARAMETERS:
 *   tof_value   - Output: Time-of-flight in picoseconds (if successful)
 *   direction   - String for debug output (e.g., "Channel 1")
 *   useChannel2 - true for Channel 2 (B→A), false for Channel 1 (A→B)
 * 
 * RETURNS:
 *   true  - Measurement successful, tof_value contains valid data
 *   false - Measurement failed (timeout, no signal, etc.)
 * 
 * TIMING:
 *   Typical measurement time: 30-120 microseconds
 *   Timeout: 150 milliseconds
 ******************************************************************************/
bool measureOneDirection(uint64_t &tof_value, const char* direction, bool useChannel2) {
    
    // ─────────────────────────────────────────────────────────────────────────
    // STEP 1: Configure TDC1000 for the desired measurement channel
    // ─────────────────────────────────────────────────────────────────────────
    // Channel 1: TX1→RX2 (Transducer A transmits, B receives)
    // Channel 2: TX2→RX1 (Transducer B transmits, A receives)
    
    if (useChannel2) {
        usafe.setMeasureTOF(TDC1000::TxRxChannel::Channel2, TDC1000::TofMode::Mode1);
    } else {
        usafe.setMeasureTOF(TDC1000::TxRxChannel::Channel1, TDC1000::TofMode::Mode1);
    }
    delay(10);  // Allow TDC1000 internal registers to settle
    
    // ─────────────────────────────────────────────────────────────────────────
    // STEP 2: Clear any previous error state
    // ─────────────────────────────────────────────────────────────────────────
    // This ensures we start with a clean slate for this measurement
    
    usafe.clearErrorFlags();      // Clear TDC1000 error flags (SIG_WEAK, NO_SIG, etc.)
    usafe.resetStatemachine();    // Reset TDC1000 internal state machine
    delay(10);
    
    // ─────────────────────────────────────────────────────────────────────────
    // STEP 3: Arm the TDC7200 for measurement
    // ─────────────────────────────────────────────────────────────────────────
    // This writes to CONFIG1 register with START_MEAS bit set.
    // Result: TDC7200's INT pin goes HIGH (indicating "waiting for STOP pulses")
    // The TDC7200 is now ready and waiting for the START pulse from TDC1000
    
    tof.startMeasurement();
    delayMicroseconds(50);  // Brief delay for TDC7200 to be fully ready
    
    // ─────────────────────────────────────────────────────────────────────────
    // STEP 4: Trigger the ultrasonic measurement
    // ─────────────────────────────────────────────────────────────────────────
    // Send a 20µs HIGH pulse to TDC1000's TRIGGER input
    // This initiates the entire measurement sequence in hardware
    
    digitalWrite(PIN_TDC1000_TRIGGER, HIGH);
    delayMicroseconds(20);
    digitalWrite(PIN_TDC1000_TRIGGER, LOW);
    
    // ─────────────────────────────────────────────────────────────────────────
    // HARDWARE SEQUENCE (happens automatically, no software intervention):
    // ─────────────────────────────────────────────────────────────────────────
    // 1. TDC1000 sends START pulse to TDC7200 → TDC7200 starts internal timer
    // 2. TDC1000 transmits ultrasonic burst from selected TX transducer
    // 3. Ultrasound propagates through water at ~1482 m/s
    // 4. Selected RX transducer receives the echo
    // 5. TDC1000 detects zero-crossings in received signal
    // 6. TDC1000 generates STOP pulse for each zero-crossing
    // 7. TDC7200 captures the time of each STOP pulse
    // 8. After 5 STOPs (or timeout), TDC7200 sets INT pin LOW
    // ─────────────────────────────────────────────────────────────────────────
    
    // ─────────────────────────────────────────────────────────────────────────
    // STEP 5: Wait for measurement completion
    // ─────────────────────────────────────────────────────────────────────────
    // Poll the INT pin until it goes LOW (indicating measurement complete)
    // INT pin states:
    //   HIGH = TDC7200 is still waiting for STOP pulses
    //   LOW  = All expected STOPs received, measurement data ready
    
    unsigned long t0 = millis();
    while (digitalRead(PIN_TDC7200_INT) == HIGH) {
        
        // Check for timeout (measurement taking too long)
        if (millis() - t0 > 150) {
            
            // Timeout occurred - measurement failed
            if (DEBUG_OUTPUT) {
                Serial.print(direction);
                Serial.println(" - TIMEOUT");
                
                // Check TDC1000 error flags to diagnose the problem
                bool sigWeak, noSig, sigHigh;
                usafe.getErrorFlags(sigWeak, noSig, sigHigh);
                
                if (sigWeak) Serial.println("  Error: Signal weak - increase gain");
                if (noSig)   Serial.println("  Error: No signal - check alignment");
                if (sigHigh) Serial.println("  Error: Signal high - reduce gain");
            }
            return false;  // Measurement failed
        }
    }
    
    // If we reach here, INT went LOW → measurement completed successfully!
    
    // ─────────────────────────────────────────────────────────────────────────
    // STEP 6: Read measurement results from TDC7200
    // ─────────────────────────────────────────────────────────────────────────
    // TDC7200 can capture up to 5 STOP pulses per measurement
    // Each STOP represents a zero-crossing of the received signal
    
    uint64_t stops[5];      // Array to store time values (in picoseconds)
    bool stopValid[5];      // Array to track which stops have valid data
    int validCount = 0;
    
    for (uint8_t i = 0; i < 5; i++) {
        stopValid[i] = tof.readMeasurement(i + 1, stops[i]);
        if (stopValid[i] && stops[i] > 0) {
            validCount++;
        }
    }
    
    // ─────────────────────────────────────────────────────────────────────────
    // OPTIONAL: Display all raw STOP times for debugging
    // ─────────────────────────────────────────────────────────────────────────
    
    if (SHOW_RAW_STOPS && DEBUG_OUTPUT) {
        Serial.print(direction);
        Serial.println(" - Raw stops:");
        for (uint8_t i = 0; i < 5; i++) {
            Serial.print("  Stop ");
            Serial.print(i + 1);
            Serial.print(": ");
            if (stopValid[i] && stops[i] > 0) {
                Serial.print((double)stops[i] / 1000000.0, 3);
                Serial.println(" us");
            } else {
                Serial.println("INVALID");
            }
        }
    }
    
    // ─────────────────────────────────────────────────────────────────────────
    // STEP 7: Select best result
    // ─────────────────────────────────────────────────────────────────────────
    // Use the first valid STOP (most reliable in Mode 1)
    // Fallback to second STOP if first is invalid
    
    if (stopValid[0] && stops[0] > 0) {
        tof_value = stops[0];
        return true;
    } else if (stopValid[1] && stops[1] > 0) {
        tof_value = stops[1];
        return true;
    } else {
        // No valid stops captured
        if (DEBUG_OUTPUT) {
            Serial.print(direction);
            Serial.print(" - No valid stops (");
            Serial.print(validCount);
            Serial.println(" total)");
        }
        return false;
    }
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
    
    // TDC1000 trigger: We drive this OUTPUT to start measurements
    pinMode(PIN_TDC1000_TRIGGER, OUTPUT);
    digitalWrite(PIN_TDC1000_TRIGGER, LOW);
    
    // TDC7200 INT: This is an OUTPUT from TDC7200 (not an interrupt input!)
    // We configure it as INPUT_PULLUP and poll it to detect completion
    pinMode(PIN_TDC7200_INT, INPUT_PULLUP);
    
    // Status LEDs
    pinMode(RedLED, OUTPUT);
    pinMode(GreenLED, OUTPUT);
    digitalWrite(GreenLED, LOW);
    digitalWrite(RedLED, LOW);
    
    Serial.println(F("=== TDC System - Sequential Mode 1 Bidirectional ===\n"));
    
    // ─────────────────────────────────────────────────────────────────────────
    // Initialize TDC1000 (Ultrasonic Analog Front End)
    // ─────────────────────────────────────────────────────────────────────────
    
    Serial.println(F("Initializing TDC1000..."));
    
    if (!usafe.begin()) {
        Serial.println(F("Failed to init TDC1000"));
        while (1);  // Halt - cannot proceed without TDC1000
    }

    bool ok = true;  // Track configuration success
    
    // ─────────────────────────────────────────────────────────────────────────
    // TDC1000 Configuration
    // ─────────────────────────────────────────────────────────────────────────
    
    // Configure trigger edge (rising edge = true)
    ok &= usafe.setTriggerEdge(true);
    
    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ TX Configuration: Ultrasonic transmission parameters                │
    // └─────────────────────────────────────────────────────────────────────┘
    // Parameters: (clock_divider, num_pulses, damping, short_burst)
    //   - clock_divider: Divides input clock to set TX frequency
    //                    8MHz / 8 = 1MHz (matches typical transducers)
    //   - num_pulses: 10 = number of ultrasonic pulses in burst
    //                 More pulses = more energy but longer burst
    //   - damping: 0 = no damping (maximize signal strength)
    //   - short_burst: true = use short burst timing
    
    ok &= usafe.setTx(TDC1000_CLKIN_FREQ_DIV, 10, 0, true);
    
    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ RX Configuration: Echo reception mode                               │
    // └─────────────────────────────────────────────────────────────────────┘
    // false = single echo mode (for Mode 1 through-transmission)
    // true  = multiple echo mode (for Mode 2 echo/reflection)
    
    ok &= usafe.setRx(false);
    
    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ Sensitivity Configuration: Critical for signal detection            │
    // └─────────────────────────────────────────────────────────────────────┘
    // These parameters determine what signals are detected as echoes.
    // Tuning guide based on distance:
    //
    // SHORT RANGE (2-12 cm):
    //   RxDacEchoThreshold::m125mV, RxPgaGain::g3dB
    //   High sensitivity for weak signals at short distances
    //
    // MEDIUM RANGE (4-20 cm): ← DEFAULT
    //   RxDacEchoThreshold::m410mV, RxPgaGain::g15dB
    //   Balanced sensitivity for general use
    //
    // LONG RANGE (12-40 cm):
    //   RxDacEchoThreshold::m775mV, RxPgaGain::g3dB
    //   Lower sensitivity to avoid noise, suitable for longer distances
    //
    // If you get "Signal weak" errors: increase gain (g15dB → g21dB)
    // If you get "Signal high" errors: decrease gain or increase threshold
    
    ok &= usafe.setRxSensitivity(
        TDC1000::RxDacEchoThreshold::m410mV,  // Echo detection threshold
        TDC1000::RxPgaGain::g15dB,            // Programmable gain amplifier
        TDC1000::RxLnaFbMode::capacitive      // Low-noise amp feedback mode
    );
    
    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ Stop Configuration: How many zero-crossings to capture              │
    // └─────────────────────────────────────────────────────────────────────┘
    // Parameters: (tx_rx_cycles, num_stops)
    //   - tx_rx_cycles: x1 = single burst per measurement
    //   - num_stops: 5 = capture 5 STOP pulses (5 zero-crossings)
    //
    // More stops provide redundancy and allow selection of best result.
    // TDC7200 will capture times of all 5 stops, we use the first valid one.
    
    ok &= usafe.setRepeat(TDC1000::TxRxCycles::x1, 5);
    
    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ Timing Configuration: Measurement window parameters                 │
    // └─────────────────────────────────────────────────────────────────────┘
    // These parameters define timing windows for the measurement:
    //   - T0: Base timing unit (clock / 1 = 125ns for 8MHz)
    //   - TxAutoZeroPeriod: T0×128 = settling time before TX
    //   - TxBlankPeriod: T0×128 = blanking after TX to avoid ringing
    //   - TxEchoTimeoutPeriod: T0×1024 = max time to wait for echo
    //
    // These values work well for 4-20cm range in water.
    // For longer distances, increase TxEchoTimeoutPeriod.
    
    ok &= usafe.setTofMeasuementShort(
        TDC1000::T0::ClkInDiv1,                    // Base timing unit
        TDC1000::TxAutoZeroPeriod::T0x128,         // 16µs autozero
        TDC1000::TxBlankPeriod::T0x128,            // 16µs blanking
        TDC1000::TxEchoTimeoutPeriod::T0x1024      // 128µs timeout
    );
    
    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ Initial Channel Selection (will be changed during measurements)     │
    // └─────────────────────────────────────────────────────────────────────┘
    // Start with Channel 1, but this gets reconfigured before each measurement
    
    ok &= usafe.setMeasureTOF(
        TDC1000::TxRxChannel::Channel1,
        TDC1000::TofMode::Mode1  // Mode 1 = through-transmission
    );
    
    // Check if all configuration steps succeeded
    if (!ok) {
        Serial.println(F("TDC1000 config failed"));
        while (1);  // Halt - configuration error
    }
    Serial.println(F("TDC1000 OK\n"));

    // ─────────────────────────────────────────────────────────────────────────
    // Initialize TDC7200 (Time-to-Digital Converter)
    // ─────────────────────────────────────────────────────────────────────────
    
    Serial.println(F("Initializing TDC7200..."));
    if (!tof.begin()) {
        Serial.println(F("Failed to init TDC7200"));
        while (1);  // Halt - cannot proceed without TDC7200
    }

    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ TDC7200 Measurement Setup                                           │
    // └─────────────────────────────────────────────────────────────────────┘
    // Parameters: (calibration2_periods, avg_cycles, num_stop, meas_mode)
    //   - calibration2_periods: 10 = clock cycles for calibration
    //   - avg_cycles: 1 = no averaging (single measurement)
    //   - num_stop: 5 = capture 5 STOP events
    //   - meas_mode: 2 = Measurement Mode 2 (time between consecutive stops)
    //
    // Measurement Mode 2 measures time between each consecutive STOP:
    //   TIME1 = STOP1 - START
    //   TIME2 = STOP2 - STOP1
    //   TIME3 = STOP3 - STOP2, etc.
    //
    // For absolute TOF, we use TIME1 (first STOP - START)
    
    if (!tof.setupMeasurement(10, 1, 5, 2)) {  
        Serial.println(F("Failed to setup TDC7200"));
        while (1);
    }
    Serial.println(F("TDC7200 OK\n"));
    
    // Configure overflow behavior (0 = no overflow adjustment needed)
    tof.setupOverflow(0);  

    // ─────────────────────────────────────────────────────────────────────────
    // System Ready - Print startup banner
    // ─────────────────────────────────────────────────────────────────────────
    
    Serial.println(F("==========================================="));
    Serial.println(F("System ready!"));
    Serial.println(F("Mode 1 Sequential: Two separate measurements"));
    Serial.println(F("Measurement 1: Channel 1 (TX1→RX2, A→B)"));
    Serial.println(F("Measurement 2: Channel 2 (TX2→RX1, B→A)"));
    Serial.println(F("===========================================\n"));
    
    // Indicate successful initialization with green LED
    digitalWrite(GreenLED, HIGH);
    delay(1000);
}

// ═════════════════════════════════════════════════════════════════════════════
// MAIN LOOP - Continuously perform bidirectional measurements
// ═════════════════════════════════════════════════════════════════════════════

void loop()
{
    uint64_t tof_ch1, tof_ch2;      // Time-of-flight values (picoseconds)
    bool success_ch1, success_ch2;   // Success flags for each channel
    
    digitalWrite(RedLED, HIGH);  // Indicate measurement in progress
    
    if (DEBUG_OUTPUT) {
        Serial.println(F("\n========================================"));
        Serial.println(F("Starting bidirectional measurement..."));
        Serial.println(F("========================================"));
    }
    
    // ═════════════════════════════════════════════════════════════════════════
    // MEASUREMENT 1: Channel 1 (A → B)
    // ═════════════════════════════════════════════════════════════════════════
    // Transducer A transmits, Transducer B receives
    
    if (DEBUG_OUTPUT) Serial.println(F("\n--- Measuring Channel 1 (TX1→RX2, A→B) ---"));
    success_ch1 = measureOneDirection(tof_ch1, "Channel 1", false);
    
    if (success_ch1 && DEBUG_OUTPUT) {
        Serial.print(F("✓ Channel 1 TOF: "));
        Serial.print((double)tof_ch1 / 1000000.0, 3);  // Convert ps to µs
        Serial.println(F(" us"));
    }
    
    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ Inter-measurement delay                                             │
    // └─────────────────────────────────────────────────────────────────────┘
    // Wait 50ms between measurements to:
    //   1. Allow acoustic energy to dissipate
    //   2. Prevent crosstalk between channels
    //   3. Ensure clean signal separation
    //
    // This delay can be reduced to ~20ms if faster update rate is needed,
    // but may increase risk of interference between channels.
    
    delay(50);
    
    // ═════════════════════════════════════════════════════════════════════════
    // MEASUREMENT 2: Channel 2 (B → A)
    // ═════════════════════════════════════════════════════════════════════════
    // Transducer B transmits, Transducer A receives
    
    if (DEBUG_OUTPUT) Serial.println(F("\n--- Measuring Channel 2 (TX2→RX1, B→A) ---"));
    success_ch2 = measureOneDirection(tof_ch2, "Channel 2", true);
    
    if (success_ch2 && DEBUG_OUTPUT) {
        Serial.print(F("✓ Channel 2 TOF: "));
        Serial.print((double)tof_ch2 / 1000000.0, 3);  // Convert ps to µs
        Serial.println(F(" us"));
    }
    
    // ═════════════════════════════════════════════════════════════════════════
    // ANALYZE RESULTS AND CALCULATE FLOW
    // ═════════════════════════════════════════════════════════════════════════
    
    Serial.println(F("\n========================================"));
    Serial.println(F("           MEASUREMENT RESULTS          "));
    Serial.println(F("========================================"));
    
    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ CASE 1: Both channels successful                                    │
    // └─────────────────────────────────────────────────────────────────────┘
    
    if (success_ch1 && success_ch2) {
        // Convert from picoseconds to microseconds for easier reading
        double tof_ch1_us = (double)tof_ch1 / 1000000.0;
        double tof_ch2_us = (double)tof_ch2 / 1000000.0;
        
        // Calculate average (for distance) and difference (for flow)
        double avg_tof = (tof_ch1_us + tof_ch2_us) / 2.0;
        double tof_diff = tof_ch1_us - tof_ch2_us;  // Positive = flow B→A
        double diff_percent = abs(tof_diff) / avg_tof * 100.0;
        
        // ┌─────────────────────────────────────────────────────────────────┐
        // │ Symmetry Validation                                             │
        // └─────────────────────────────────────────────────────────────────┘
        // In static water with good alignment, both TOF should be similar.
        // Large differences indicate:
        //   - Misalignment between transducers
        //   - Different acoustic coupling on each side
        //   - Air bubbles or obstructions in one direction
        //   - Or actual flow (if difference is consistent)
        
        bool reasonable = (diff_percent < 30.0);  // Less than 30% difference
        
        Serial.print(F("STATUS: "));
        if (reasonable) {
            Serial.println(F("✓✓ WATER DETECTED"));
        } else {
            Serial.print(F("⚠ ASYMMETRIC ("));
            Serial.print(diff_percent, 1);
            Serial.println(F("% difference)"));
        }
        Serial.println();
        
        // Display individual channel results
        Serial.print(F("Channel 1 (A→B): "));
        Serial.print(tof_ch1_us, 3);
        Serial.println(F(" us"));
        
        Serial.print(F("Channel 2 (B→A): "));
        Serial.print(tof_ch2_us, 3);
        Serial.println(F(" us"));
        
        Serial.print(F("Average TOF:     "));
        Serial.print(avg_tof, 3);
        Serial.println(F(" us"));
        
        Serial.print(F("TOF Difference:  "));
        Serial.print(tof_diff, 3);
        Serial.print(F(" us ("));
        Serial.print(diff_percent, 1);
        Serial.println(F("%)"));
        
        // ┌─────────────────────────────────────────────────────────────────┐
        // │ Distance Calculation                                            │
        // └─────────────────────────────────────────────────────────────────┘
        // Speed of sound in water varies with temperature:
        //   15°C: 1465 m/s  →  0.1465 cm/µs
        //   20°C: 1482 m/s  →  0.1482 cm/µs  ← Default
        //   25°C: 1497 m/s  →  0.1497 cm/µs
        //
        // Distance = (Speed of Sound) × (Average TOF)
        //          = 1482 m/s × avg_tof
        //          = 0.1482 cm/µs × avg_tof
        
        double distance_cm = avg_tof * 0.1482;  // For 20°C water
        Serial.print(F("Distance:        "));
        Serial.print(distance_cm, 2);
        Serial.println(F(" cm"));
        
        // ┌─────────────────────────────────────────────────────────────────┐
        // │ Flow Analysis                                                   │
        // └─────────────────────────────────────────────────────────────────┘
        // Flow affects TOF in opposite ways for each direction:
        //
        // With flow velocity v in A→B direction:
        //   TOF_AB = L / (c + v)  ← faster (with flow)
        //   TOF_BA = L / (c - v)  ← slower (against flow)
        //
        // Where: L = distance, c = speed of sound in water, v = flow velocity
        //
        // Time difference: Δt = TOF_AB - TOF_BA
        //   Δt > 0: Flow in B→A direction (TOF_AB longer)
        //   Δt < 0: Flow in A→B direction (TOF_AB shorter)
        //   Δt ≈ 0: Static water
        
        if (reasonable) {
            if (abs(tof_diff) > 0.15) {  // More than 150ns difference
                Serial.println();
                Serial.print(F("Flow detected: "));
                if (tof_diff > 0) {
                    Serial.println(F("Flowing B→A direction"));
                } else {
                    Serial.println(F("Flowing A→B direction"));
                }
                
                // ┌─────────────────────────────────────────────────────────┐
                // │ Flow Velocity Calculation (Simplified)                 │
                // └─────────────────────────────────────────────────────────┘
                // Approximation: Δt ≈ 2×L×v / c²
                // Solving for v: v ≈ (c² × Δt) / (2×L)
                //
                // Where:
                //   c = speed of sound in water (1482 m/s)
                //   L = distance between transducers (meters)
                //   Δt = time difference (seconds)
                //   v = flow velocity (m/s)
                //
                // Note: This is a simplified formula that assumes v << c
                // For more accurate results with higher flows, use:
                // v = (c² / 2L) × [(TOF_BA - TOF_AB) / (TOF_BA × TOF_AB)]
                
                double c = 1482.0;  // Speed of sound in water (m/s at 20°C)
                double L = distance_cm / 100.0;  // Convert cm to meters
                double dt = abs(tof_diff) / 1000000.0;  // Convert µs to seconds
                
                if (L > 0.01) {  // Avoid division by very small numbers
                    double flow_velocity = (c * c * dt) / (2.0 * L);
                    Serial.print(F("Flow velocity:   "));
                    Serial.print(flow_velocity * 100.0, 2);  // Convert m/s to cm/s
                    Serial.println(F(" cm/s"));
                }
            } else {
                Serial.println(F("Flow: Static water (< 150ns diff)"));
            }
        } else {
            // Large asymmetry detected - provide troubleshooting guidance
            Serial.println();
            Serial.println(F("⚠ WARNING: Large asymmetry detected!"));
            Serial.println(F("  Possible causes:"));
            Serial.println(F("  - Misalignment between transducers"));
            Serial.println(F("  - Different acoustic coupling on each side"));
            Serial.println(F("  - Air bubbles in path"));
            Serial.println(F("  - One transducer partially blocked"));
        }
        
        // Update statistics
        if (reasonable) successCount++;
        else errorCount++;
        
    }
    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ CASE 2: Only one channel successful                                │
    // └─────────────────────────────────────────────────────────────────────┘
    else if (success_ch1 || success_ch2) {
        Serial.println(F("STATUS: ⚠ PARTIAL SIGNAL"));
        Serial.println();
        
        if (success_ch1) {
            Serial.print(F("Only Channel 1: "));
            Serial.print((double)tof_ch1 / 1000000.0, 3);
            Serial.println(F(" us"));
            Serial.println(F("Channel 2 failed - check transducer B"));
        }
        
        if (success_ch2) {
            Serial.print(F("Only Channel 2: "));
            Serial.print((double)tof_ch2 / 1000000.0, 3);
            Serial.println(F(" us"));
            Serial.println(F("Channel 1 failed - check transducer A"));
        }
        
        errorCount++;
        
    }
    // ┌─────────────────────────────────────────────────────────────────────┐
    // │ CASE 3: Both channels failed                                       │
    // └─────────────────────────────────────────────────────────────────────┘
    else {
        Serial.println(F("STATUS: ✗✗ NO SIGNAL"));
        Serial.println();
        Serial.println(F("Both channels failed"));
        Serial.println(F("Check:"));
        Serial.println(F("  - Water present?"));
        Serial.println(F("  - Transducers aligned and facing each other?"));
        Serial.println(F("  - Wiring connections secure?"));
        Serial.println(F("  - 300pF capacitors installed?"));
        
        errorCount++;
    }
    
    // ═════════════════════════════════════════════════════════════════════════
    // Display Running Statistics
    // ═════════════════════════════════════════════════════════════════════════
    
    Serial.println();
    Serial.print(F("Success rate: "));
    Serial.print(successCount);
    Serial.print(F("/"));
    Serial.print(successCount + errorCount);
    if (successCount + errorCount > 0) {
        Serial.print(F(" ("));
        Serial.print(100.0 * successCount / (successCount + errorCount), 1);
        Serial.println(F("%)"));
    }
    Serial.println(F("========================================\n"));
    
    digitalWrite(RedLED, LOW);  // Turn off measurement indicator
    
    // Delay before next measurement cycle
    // Adjust this value based on your application needs:
    //   - Faster updates: reduce to 100-200ms
    //   - More stable: increase to 1000ms or more
    delay(500);
}

