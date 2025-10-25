/*******************************************************************************
 * TDC1000/TDC7200 Bidirectional Flow Measurement - CHsel External Mode
 * 
 * This example measures flow in both directions (A→B and B→A) using hardware
 * pin control instead of software configuration. The key difference from the
 * software-based version is that channel selection is done by toggling the
 * CHsel pin (LOW=Channel1, HIGH=Channel2) rather than reconfiguring the TDC1000
 * via SPI commands. The TDC1000 is configured once in setup() with External
 * channel mode, then the CHsel pin is sampled on each trigger to determine
 * which channel to use. This approach is faster (no SPI overhead) and more
 * deterministic. Additionally, this code includes optimizations for out-of-water
 * operation: higher echo threshold (410mV), longer blank period (T0×256), and
 * stronger TX signal (16 cycles) to reject electrical crosstalk that occurs
 * when transducers are not acoustically coupled through water.
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

// ═════════════════════════════════════════════════════════════════════════════
#define NUM_STOPS (5)

// External clock is 8 MHz
#define TDC1000_CLKIN_FREQ_HZ  (8000000UL)
#define TDC1000_CLKIN_FREQ_DIV (TDC1000::TxFreqDivider::Div8)
#define TDC7200_CLOCK_FREQ_HZ  (8000000UL)


// ═════════════════════════════════════════════════════════════════════════════
// DEBUG CONFIGURATION
// ═════════════════════════════════════════════════════════════════════════════

#define DEBUG_OUTPUT true       // Enable detailed serial output
#define SHOW_RAW_STOPS false    // Show all 5 STOP times for debugging

// ═════════════════════════════════════════════════════════════════════════════
// HARDWARE INITIALIZATION
// ═════════════════════════════════════════════════════════════════════════════

static TDC7200 tof(PIN_TDC7200_ENABLE, PIN_TDC7200_SPI_CS, TDC7200_CLOCK_FREQ_HZ);
static TDC1000 usafe(PIN_TDC1000_SPI_CS, PIN_TDC1000_RESET);

// ═════════════════════════════════════════════════════════════════════════════
// STATISTICS TRACKING
// ═════════════════════════════════════════════════════════════════════════════

uint32_t successCount = 0;  // Successful bidirectional measurements
uint32_t errorCount = 0;    // Failed or partial measurements

// ═════════════════════════════════════════════════════════════════════════════
// MEASUREMENT FUNCTION - CHsel Hardware Control
// ═════════════════════════════════════════════════════════════════════════════

bool measureOneDirection(uint64_t &tof_value, const char* direction, bool chselHigh) {
    // Set CHsel pin to select channel (sampled by TDC1000 on trigger edge)
    // This MUST be done before triggering the measurement
    digitalWrite(CHsel, chselHigh ? HIGH : LOW);
    delay(50);  // Allow TDC1000 internal multiplexers to settle
    
    // Clear any previous error state in TDC1000
    usafe.clearErrorFlags();
    usafe.resetStatemachine();
    delay(10);
    
    // Arm TDC7200 for measurement (INT pin goes HIGH)
    tof.startMeasurement();
    delayMicroseconds(100);  // Give TDC7200 time to be ready
    
    // Trigger TDC1000 measurement
    // TDC1000 samples CHsel pin at this moment to determine which channel
    digitalWrite(PIN_TDC1000_TRIGGER, HIGH);
    delayMicroseconds(20);
    digitalWrite(PIN_TDC1000_TRIGGER, LOW);
    
    // Poll INT pin until measurement completes (INT goes LOW)
    unsigned long t0 = millis();
    while (digitalRead(PIN_TDC7200_INT) == HIGH) {
        if (millis() - t0 > 150) {  // 150ms timeout
            if (DEBUG_OUTPUT) {
                Serial.print(direction);
                Serial.println(" - TIMEOUT");
                
                // Read and display TDC1000 error flags
                bool sigWeak, noSig, sigHigh;
                usafe.getErrorFlags(sigWeak, noSig, sigHigh);
                if (sigWeak) Serial.println("  Error: Signal weak");
                if (noSig) Serial.println("  Error: No signal");
                if (sigHigh) Serial.println("  Error: Signal high");
            }
            return false;
        }
    }
    
    // Read all 5 STOP measurements from TDC7200
    uint64_t stops[5];
    bool stopValid[5];
    int validCount = 0;
    
    for (uint8_t i = 0; i < 5; i++) {
        stopValid[i] = tof.readMeasurement(i + 1, stops[i]);
        if (stopValid[i] && stops[i] > 0) {
            validCount++;
        }
    }
    
    // Optional: Display all raw STOP times for debugging
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
    
    // Use first valid stop (most reliable), fallback to second if needed
    if (stopValid[0] && stops[0] > 0) {
        tof_value = stops[0];
        return true;
    } else if (stopValid[1] && stops[1] > 0) {
        tof_value = stops[1];
        return true;
    } else {
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
// SETUP
// ═════════════════════════════════════════════════════════════════════════════

void setup()
{
    Serial.begin(115200);
    delay(100);
    
    // Configure CHsel pin as output (controls channel selection)
    pinMode(CHsel, OUTPUT);
    digitalWrite(CHsel, LOW);  // Start with Channel 1
    delay(100);
    
    // Configure other GPIO pins
    pinMode(PIN_TDC1000_TRIGGER, OUTPUT);
    digitalWrite(PIN_TDC1000_TRIGGER, LOW);
    pinMode(PIN_TDC7200_INT, INPUT_PULLUP);  // Read-only, TDC7200 drives this
    pinMode(RedLED, OUTPUT);
    pinMode(GreenLED, OUTPUT);
    
    digitalWrite(GreenLED, LOW);
    digitalWrite(RedLED, LOW);
    
    Serial.println(F("=== TDC System - CHsel External (Optimized) ===\n"));
    Serial.println(F("Initializing TDC1000..."));
    
    // Initialize TDC1000 via SPI
    if (!usafe.begin()) {
        Serial.println(F("Failed to init TDC1000"));
        while (1);
    }

    bool ok = true;
    
    // Configure trigger edge (rising = true)
    ok &= usafe.setTriggerEdge(true);
    
    // TX Configuration: 16 cycles for stronger signal (helps with out-of-water operation)
    // Stronger signal overcomes air coupling losses and electrical crosstalk
    ok &= usafe.setTx(TDC1000_CLKIN_FREQ_DIV, 16, 0, true);
    
    // RX Configuration: Single echo mode (false = better noise rejection)
    // In air/out-of-water, we want to reject multiple reflections and crosstalk
    ok &= usafe.setRx(false);
    
    // Sensitivity: Higher threshold (410mV) to reject electrical crosstalk
    // When out of water, electrical noise can trigger false detections
    // Higher threshold ensures only real acoustic signals are detected
    ok &= usafe.setRxSensitivity(
        TDC1000::RxDacEchoThreshold::m410mV,  // Higher than typical (was m220mV)
        TDC1000::RxPgaGain::g15dB,            // Moderate gain
        TDC1000::RxLnaFbMode::capacitive      
    );
    
    // Number of stops to capture per measurement
    ok &= usafe.setRepeat(TDC1000::TxRxCycles::x1, 5);
    
    // Timing configuration: Longer blank period (T0×256) to avoid ringing/crosstalk
    // TX burst creates ringing on PCB traces; longer blank gives more time for it to die down
    ok &= usafe.setTofMeasuementShort(
        TDC1000::T0::ClkInDiv1,                    // Base timing unit (125ns)
        TDC1000::TxAutoZeroPeriod::T0x128,         // 16µs settling time
        TDC1000::TxBlankPeriod::T0x256,            // 32µs blanking (longer than normal)
        TDC1000::TxEchoTimeoutPeriod::T0x1024      // 128µs timeout
    );
    
    // CRITICAL: Set channel selection to External mode
    // This tells TDC1000 to read the CHsel pin instead of using software configuration
    ok &= usafe.setMeasureTOF(
        TDC1000::TxRxChannel::External,  // Use CHsel pin for channel selection
        TDC1000::TofMode::Mode1          // Mode 1 (through-transmission)
    );
    
    // Optional: Dump TDC1000 configuration for debugging
    if (DEBUG_OUTPUT) {
        Serial.println(F("\n===== TDC1000 Configuration ====="));
        usafe.dumpSettings(8000000);
        Serial.println(F("=================================\n"));
    }

    // Check if all configuration succeeded
    if (!ok) {
        Serial.println(F("TDC1000 config failed"));
        while (1);
    }
    Serial.println(F("TDC1000 OK\n"));

    // Initialize TDC7200 (Time-to-Digital Converter)
    Serial.println(F("Initializing TDC7200..."));
    if (!tof.begin()) {
        Serial.println(F("Failed to init TDC7200"));
        while (1);
    }

    // Setup TDC7200 measurement parameters
    // (calibration, averaging, stops, measurement mode)
    if (!tof.setupMeasurement(10, 1, 5, 2)) {
        Serial.println(F("Failed to setup TDC7200"));
        while (1);
    }
    Serial.println(F("TDC7200 OK\n"));
    
    // Configure overflow handling
    tof.setupOverflow(0);

    // Display startup information
    Serial.println(F("==========================================="));
    Serial.println(F("System ready - CHsel External Mode"));
    Serial.println(F("Optimized for out-of-water operation"));
    Serial.println(F("- Higher threshold (410mV) rejects noise"));
    Serial.println(F("- Longer blank period avoids crosstalk"));
    Serial.println(F("- Stronger TX signal (16 cycles)"));
    Serial.println(F("===========================================\n"));
    
    // Indicate successful initialization
    digitalWrite(GreenLED, HIGH);
    delay(1000);
}

// ═════════════════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═════════════════════════════════════════════════════════════════════════════

void loop()
{
    uint64_t tof_ch1, tof_ch2;
    bool success_ch1, success_ch2;
    
    digitalWrite(RedLED, HIGH);  // Indicate measurement in progress
    
    if (DEBUG_OUTPUT) {
        Serial.println(F("\n========================================"));
        Serial.println(F("Measurement cycle"));
        Serial.println(F("========================================"));
    }
    
    // ═════════════════════════════════════════════════════════════════════════
    // MEASUREMENT 1: Channel 1 (A→B)
    // ═════════════════════════════════════════════════════════════════════════
    // Set CHsel=LOW to select Channel 1 (TX1→RX2)
    
    if (DEBUG_OUTPUT) Serial.println(F("\n--- Channel 1 (CHsel=LOW, A→B) ---"));
    success_ch1 = measureOneDirection(tof_ch1, "Channel 1", false);  // false = CHsel LOW
    
    if (success_ch1 && DEBUG_OUTPUT) {
        Serial.print(F("✓ Channel 1: "));
        Serial.print((double)tof_ch1 / 1000000.0, 3);
        Serial.println(F(" us"));
    }
    
    delay(100);  // Gap between measurements to avoid acoustic interference
    
    // ═════════════════════════════════════════════════════════════════════════
    // MEASUREMENT 2: Channel 2 (B→A)
    // ═════════════════════════════════════════════════════════════════════════
    // Set CHsel=HIGH to select Channel 2 (TX2→RX1)
    
    if (DEBUG_OUTPUT) Serial.println(F("\n--- Channel 2 (CHsel=HIGH, B→A) ---"));
    success_ch2 = measureOneDirection(tof_ch2, "Channel 2", true);  // true = CHsel HIGH
    
    if (success_ch2 && DEBUG_OUTPUT) {
        Serial.print(F("✓ Channel 2: "));
        Serial.print((double)tof_ch2 / 1000000.0, 3);
        Serial.println(F(" us"));
    }
    
    // ═════════════════════════════════════════════════════════════════════════
    // ANALYZE RESULTS
    // ═════════════════════════════════════════════════════════════════════════
    
    Serial.println(F("\n========================================"));
    Serial.println(F("           RESULTS          "));
    Serial.println(F("========================================"));
    
    if (success_ch1 && success_ch2) {
        // Both channels successful - calculate flow parameters
        double tof_ch1_us = (double)tof_ch1 / 1000000.0;
        double tof_ch2_us = (double)tof_ch2 / 1000000.0;
        double avg_tof = (tof_ch1_us + tof_ch2_us) / 2.0;
        double tof_diff = tof_ch1_us - tof_ch2_us;
        double diff_percent = abs(tof_diff) / avg_tof * 100.0;
        
        // Check if measurements are reasonably symmetric
        bool reasonable = (diff_percent < 30.0);
        
        Serial.print(F("STATUS: "));
        if (reasonable) {
            Serial.println(F("✓✓ WATER DETECTED"));
        } else {
            Serial.print(F("⚠ ASYMMETRIC ("));
            Serial.print(diff_percent, 1);
            Serial.println(F("%)"));
        }
        Serial.println();
        
        // Display individual channel results
        Serial.print(F("Channel 1 (A→B): "));
        Serial.print(tof_ch1_us, 3);
        Serial.println(F(" us"));
        
        Serial.print(F("Channel 2 (B→A): "));
        Serial.print(tof_ch2_us, 3);
        Serial.println(F(" us"));
        
        Serial.print(F("Average:         "));
        Serial.print(avg_tof, 3);
        Serial.println(F(" us"));
        
        Serial.print(F("Difference:      "));
        Serial.print(tof_diff, 3);
        Serial.print(F(" us ("));
        Serial.print(diff_percent, 1);
        Serial.println(F("%)"));
        
        // Calculate distance (speed of sound in water ~1482 m/s at 20°C)
        double distance_cm = avg_tof * 0.1482;
        Serial.print(F("Distance:        "));
        Serial.print(distance_cm, 2);
        Serial.println(F(" cm"));
        
        // Flow detection (if measurements are reasonable)
        if (reasonable) {
            if (abs(tof_diff) > 0.15) {  // More than 150ns difference
                Serial.println();
                Serial.print(F("Flow: "));
                Serial.println(tof_diff > 0 ? F("B→A") : F("A→B"));
            } else {
                Serial.println(F("Flow: Static"));
            }
        }
        
        // Update statistics
        if (reasonable) successCount++;
        else errorCount++;
        
    } else if (success_ch1 || success_ch2) {
        // Only one channel successful - partial measurement
        Serial.println(F("STATUS: ⚠ PARTIAL"));
        Serial.println();
        
        if (success_ch1) {
            Serial.print(F("Only CH1: "));
            Serial.print((double)tof_ch1 / 1000000.0, 3);
            Serial.println(F(" us"));
        }
        if (success_ch2) {
            Serial.print(F("Only CH2: "));
            Serial.print((double)tof_ch2 / 1000000.0, 3);
            Serial.println(F(" us"));
        }
        
        errorCount++;
        
    } else {
        // Both channels failed - no signal detected
        Serial.println(F("STATUS: ✗✗ NO SIGNAL"));
        Serial.println(F("Both channels failed"));
        
        // Special message for out-of-water operation
        Serial.println(F("\nIf transducers are OUT of water:"));
        Serial.println(F("  ✓ This is EXPECTED behavior"));
        Serial.println(F("  ✓ High threshold rejects crosstalk"));
        Serial.println(F("  ✓ Place in water for measurements"));
        
        errorCount++;
    }
    
    // Display success rate
    Serial.println();
    Serial.print(F("Success: "));
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
    delay(500);  // Wait before next measurement cycle
}
