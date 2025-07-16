#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#endif

// Constants
const int MAX_SPEED        = 255;
const int MIN_MOTOR_SPEED  = 70; // minimum speed to avoid stalling, can be adjusted

const int LEFT_OFFSET      = +5;  // Offset for left motor speed
const int RIGHT_OFFSET     = -5;  // Offset for right motor speed

const int MIN_DISTANCE     = 30;   // Minimum distance in cm before turning
const int TURN_TIME        = 800;     // Time to turn in milliseconds
const int SCAN_INTERVAL    = 300; // Time between distance measurements

const int LOOP_DELAY_MS = 1; // how often we run the main loop
const int RAMP_STEP = 30;    // how many speed units we change per loop, less = smoother but slower response

// 2) Compute target speeds in some basic scenarios => FWD/BWD, L/R (sharp turn)
//    constaint to MIN_MOTOR_SPEED, maxMotorSpeed and apply ramping
const int JOYSTICK_DEADZONE = 18;                      // deadzone around center based on joystick variance
const int FORWARD_THRESHOLD = 512 + JOYSTICK_DEADZONE;  // center = 512, forward = 1023  (center can change with usage)
const int BACKWARD_THRESHOLD = 512 - JOYSTICK_DEADZONE; // center = 512, backward = 0

// function declarations
int slewRateLimit(int current, int target);

#ifdef ARDUINO
String pad5(int val);
String pad5f(float val);
#else
#include <string>
std::string pad5(int val);
std::string pad5f(float val);

#endif





struct JoystickInput {
    int x;
    int y;
};

struct JoystickProcessingResult {
    float rawRatioLR;
    float steppedRatioLR;
    float quantizeStep;
    int correctedX;
    int correctedY;
    bool buzzerOn;
};

struct MotorTargets {
    int left;
    int right;
    bool skipSlewRate;
};

struct ManualModeInputs {
    JoystickProcessingResult joystick;
    int leftSpeed;
    int rightSpeed;
    int prevLeftSpeed;
    int prevRightSpeed;
    // ...add more as needed
};

struct ManualModeOutputs {
    int leftSpeed;
    int rightSpeed;
    int outputLeft;
    int outputRight;
    bool brakingApplied;
    bool skipSlewRate;
    bool buzzerOn;
    // ...add more as needed
};

JoystickProcessingResult processJoystick(int joystickX, int joystickY, bool joystickButton = false);
MotorTargets computeMotorTargets(const JoystickProcessingResult &js, int prevLeft, int prevRight);
long map(float steppedRatioLR);
bool shouldSkipSlewRate(int prevLeft, int prevRight, int targetLeft, int targetRight);
bool shouldApplyBraking(int prevLeft, int prevRight, int targetLeft, int targetRight);
ManualModeOutputs manualModeStep(const ManualModeInputs& in);