#pragma once
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cstdint>
#endif

// Constants
const int MAX_ADC_VALUE    = 1023;  // Maximum ADC value for 10-bit ADC (0-1023 range)

const int LOOP_DELAY_MS = 1;        // how often we run the main loop
const int JOYSTICK_CENTER = 512;    // Center position of joystick (0-1023 range)
const int JOYSTICK_DEADZONE = 20;   // deadzone around center based on joystick variance

// Voltage Divider Constants (shared between transmitter and receiver)
// Current configuration: R1 = 276K (3 x 92K), R2 = 100K
// Arduino: 3.3V/8MHz, ADC reference = 3.29V measured
const float VOLTAGE_CORRECTION_FACTOR = 12.16f / 11.96f;       // Corrector for Vref tolerance (measured 3.29V vs nominal 3.3V)
const float VOLTAGE_ADC_REFERENCE = 3.3f;        // Measured ADC reference voltage (3.3V Arduino)
const float VOLTAGE_DIVIDER_RATIO = VOLTAGE_CORRECTION_FACTOR * (276.0f + 97.5f) / 97.5f; // ≈ 3.831
const float VOLTAGE_CALIBRATION_BATTERY = 12.6f; // Actual battery voltage when ADC reads maximum
const float VBATT_MIN = 3.0f;               // Minimum battery voltage (per cell)
const float VBATT_MAX = 4.2f;               // Maximum battery voltage (per cell)
    
const int PACKET_HISTORY_SIZE = 32; // Number of packets to track for success rate
extern uint32_t packetHistory;
extern uint8_t packetIndex;

void updatePacketHistory(bool success);
float getSuccessRate();

#ifdef ARDUINO
String pad5(int val);
String pad5f(float val);
String pad3(int val);
String pad3f(float val);
String pad3s(int val);
String pad4s(int val);  // 4-char right-aligned signed integer (for motor speeds)
String pad2s(int val);  // 2-char right-aligned integer (for percentages/minutes/seconds)
String padString(String str, int width, char padChar = ' ', bool leftAlign = false);
#else
#include <string>
std::string pad5(int val);
std::string pad5f(float val);
std::string pad3(int val);
std::string pad3f(float val);
std::string pad3s(int val);
std::string pad4s(int val);  // 4-char right-aligned signed integer (for motor speeds)
std::string pad2s(int val);  // 2-char right-aligned integer (for percentages/minutes/seconds)
std::string padString(std::string str, int width, char padChar = ' ', bool leftAlign = false);
#endif

struct JoystickInput {
    int x;
    int y;
};

struct JoystickProcessingResult {
    float rawRatioLR;
    float steppedRatioLR;
    uint16_t rawX;         // Raw joystick X (0-1023, center = 512)
    uint16_t rawY;         // Raw joystick Y (0-1023, center = 512)
    bool buzzerOn;
};


struct MotorTargets {
    int targetLeft;         // Target left speed (raw, before slew/brake)
    int targetRight;        // Target right speed (raw, before slew/brake)
    int outputLeft;   // Final output left speed (after slew/brake)
    int outputRight;  // Final output right speed (after slew/brake)
    bool skipSlewRate;
    bool brakingApplied;
};

// Request types for receiver-to-transmitter communication
enum RxRequestType : uint8_t {
    RXREQ_NONE = 0,
    RXREQ_REQUEST_CALIBRATION = 1,
    // Add more request types as needed
};

// Packet structures - MUST MATCH TRANSMITTER EXACTLY
struct __attribute__((packed)) JoystickData {
    uint16_t xValue;
    uint16_t yValue;
    uint8_t  buttonPressed; // 0/1
    uint8_t  checksum;      // simple xor
};
static_assert(sizeof(JoystickData) == 6, "JoystickData size mismatch");

// Calibration/config packet structure
struct __attribute__((packed)) CalibrationPacket {
    uint16_t xCenter;
    uint16_t yCenter;
    uint8_t checksum;
};
static_assert(sizeof(CalibrationPacket) == 5, "CalibrationPacket size mismatch");

// Simplified response data for ACK payload - both transmitter and receiver must agree exactly
struct __attribute__((packed)) RxData {
    uint8_t voltage;     // Battery voltage (0-255)
    uint8_t successRate; // Rx success rate (0-255)
    uint8_t status;      // Combined mode + state info
    RxRequestType request; // Request type (e.g., calibration)
    uint8_t crc;         // Simple CRC
    RxData(uint8_t v = 0, uint8_t sr = 0, uint8_t st = 0, RxRequestType req = RXREQ_NONE, uint8_t c = 0)
        : voltage(v), successRate(sr), status(st), request(req), crc(c) {}
};
static_assert(sizeof(RxData) == 5, "RxData size mismatch");

inline uint8_t calcCalibrationChecksum(uint16_t xCenter, uint16_t yCenter) {
    uint8_t checksum = 0;
    checksum ^= (xCenter & 0xFF);
    checksum ^= ((xCenter >> 8) & 0xFF);
    checksum ^= (yCenter & 0xFF);
    checksum ^= ((yCenter >> 8) & 0xFF);
    return checksum;
}
