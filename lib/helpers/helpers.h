#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#endif

// Constants
const int MAX_SPEED        = 255;
const int MIN_MOTOR_SPEED  = 70; // minimum speed to avoid stalling, can be adjusted
const int MOTOR_DEADZONE   = 10; // motor values within this range of zero are considered stopped

const int LR_OFFSET         = -10;        // Left/Right balance offset, +ve = more right speed, -ve = more left speed

const int MIN_DISTANCE     = 30;   // Minimum distance in cm before turning
const int TURN_TIME        = 800;     // Time to turn in milliseconds
const int SCAN_INTERVAL    = 300; // Time between distance measurements

const int LOOP_DELAY_MS = 1; // how often we run the main loop
const int RAMP_STEP = 30;    // how many speed units we change per loop, less = smoother but slower response

// Voltage Divider Constants (shared between transmitter and receiver)
// Current configuration: R1 = 276K (3 x 92K), R2 = 100K
// Arduino: 3.3V/8MHz, ADC reference = 3.29V measured
const float VOLTAGE_ADC_REFERENCE = 3.29f;        // Measured ADC reference voltage (3.3V Arduino)
const float VOLTAGE_DIVIDER_RATIO = 3.76f;        // (R1 + R2) / R2 = (276K + 100K) / 100K = 3.76
const float VOLTAGE_CALIBRATION_BATTERY = 12.27f; // Actual battery voltage when ADC reads maximum
const float VOLTAGE_CALIBRATION_ADC_PIN = 3.29f;  // Voltage at ADC pin when battery = 12.27V

// 2) Compute target speeds in some basic scenarios => FWD/BWD, L/R (sharp turn)
//    constaint to MIN_MOTOR_SPEED, maxMotorSpeed and apply ramping
const int JOYSTICK_CENTER = 512;                        // Center position of joystick (0-1023 range)
const int JOYSTICK_DEADZONE = 30;                       // deadzone around center based on joystick variance
const int MAX_ADC_VALUE = 1023;                         // Maximum ADC value for 10-bit ADC (0-1023 range)
const int FORWARD_THRESHOLD = JOYSTICK_CENTER + JOYSTICK_DEADZONE;  // threshold to start forward movement
const int BACKWARD_THRESHOLD = JOYSTICK_CENTER - JOYSTICK_DEADZONE; // threshold to start backward movement
const float FULL_TURN_THRESHOLD = 0.6f; // threshold (as a percentage of max X deflection) for full in-place turn
// function declarations
int slewRateLimit(int current, int target);

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
    int rawX;         // Raw joystick X (0-1023, center = 512)
    int rawY;         // Raw joystick Y (0-1023, center = 512)
    bool buzzerOn;
};

struct MotorTargets {
    int left;
    int right;
    bool skipSlewRate;
};