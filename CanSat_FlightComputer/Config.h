#ifndef CONFIG_H
#define CONFIG_H

// =======================================================================
// PIN / PORT CONFIG
// Your schematic (RP2040, W25Q128 flash, L80-R GPS, BMP-class baro on I2C,
// BMI088 IMU, servo headers, battery divider) doesn't give me clean text
// for every GPIO number, so double check/adjust these against your board.
// =======================================================================

// --- Debug console (USB CDC on RP2040 - not a UART, always "Serial") ---
#define DEBUG_BAUD          115200

// --- GPS (L80-R), NMEA over SoftwareSerial on Core 1 ---
// SoftwareSerial(rxPin, txPin) - any two free GPIOs, no hardware UART needed.
#define GPS_BAUD             9600
#define GPS_SW_RX_PIN        6     // RP2040 pin reading GPS TX -> adjust
#define GPS_SW_TX_PIN        7     // RP2040 pin driving GPS RX -> adjust

// --- Telemetry radio: E32-433T-33D, transparent mode, SoftwareSerial on Core 1 ---
#define TELEM_BAUD           9600
#define TELEM_SW_RX_PIN      3     // adjust
#define TELEM_SW_TX_PIN      2     // adjust
#define PIN_E32_M0           -1    // -1 if M0 is hard-tied to GND on the PCB
#define PIN_E32_M1           -1    // -1 if M1 is hard-tied to GND on the PCB

// --- SD card (SPI) ---
#define PIN_SD_CS            17    // adjust to match schematic MISO/MOSI/SCK/CS block
#define SD_LOG_FILENAME      "flight_log.csv"

// --- Barometer (I2C): TE Connectivity MS5611 ---
#define BARO_I2C_ADDR        0x77    // MS5611 default; some breakouts use 0x76 (CSB pin state)
#define SEA_LEVEL_HPA        1013.25f  // only used as a fallback

// --- Battery voltage divider on an ADC pin ---
#define PIN_BATT_ADC         26        // RP2040 ADC0
#define ADC_REF_VOLTAGE      3.3f
#define ADC_MAX_COUNTS       4095.0f
#define VDIV_TOP_OHMS        27000.0f  // R13 - adjust to your actual values
#define VDIV_BOTTOM_OHMS     15000.0f  // R14 - adjust to your actual values

// --- Deployment mechanism outputs (placeholders, wire to servo/cutter driver) ---
#define PIN_AEROBRAKE_ACTUATOR   -1
#define PIN_PARACHUTE_ACTUATOR   -1
#define Servo1Pin 20
#define Servo2Pin 21
#define Servo3Pin 22

// --- Other items for use

// =======================================================================
// TIMING
// =======================================================================
#define SAMPLE_INTERVAL_MS       10     // ~100 Hz IMU sampling / integration
#define LOG_INTERVAL_MS          50     // ~20 Hz SD + Serial print
#define TELEM_INTERVAL_MS_Flight 334    // ~3 Hz MAX telemetry rate (per spec)
#define TELEM_INTERVAL_MS_Idle   2000   // ~0.5 Hz for sitting on the pad
#define STATE_SAVE_INTERVAL_MS   1000   // 1 Hz LittleFS persistent-state save
#define CALIBRATION_SAMPLES      300    // ~3 s at 100 Hz, cold-start bias capture

// =======================================================================
// FLIGHT LOGIC THRESHOLDS
// =======================================================================
#define LIFTOFF_VVEL_THRESHOLD_MS     5.0f   // m/s upward => confirm liftoff
#define APOGEE_DESCEND_CONFIRM_N      5      // consecutive falling baro samples => apogee
#define PARACHUTE_DEPLOY_ALT_M        150.0f // per mission guide C3 / M6
#define LANDED_ALT_THRESHOLD_M        5.0f
#define LANDED_VVEL_THRESHOLD_MS      1.0f
#define LANDED_CONFIRM_N              100    // consecutive calm samples => landed

// =======================================================================
// PHYSICAL CONSTANTS
// =======================================================================
#define G_MS2                9.80665f

#endif
