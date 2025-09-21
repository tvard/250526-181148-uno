#include "helpers.h"

// Motor/receiver-specific constants
const int LR_OFFSET         = 0;  // Left/Right balance offset, +ve = more right speed, -ve = more left speed
const int MAX_SPEED        = 255;
const int MIN_MOTOR_SPEED  = 60;    // minimum speed to avoid stalling, can be adjusted
const int MOTOR_DEADZONE   = 10;    // motor values within this range of zero are considered stopped

// Receiver-specific drive/motor logic constants
const unsigned int RAMP_STEP_SLOW = 01;         // slow slew rate step (for gentle joystick deflection)
const unsigned int RAMP_STEP_FAST_MEH = 10;     // fast slew rate step (for aggressive joystick deflection)
const unsigned int RAMP_STEP_FAST_AF = 60;      // almost no slew (some ramping to protect motors)
const unsigned int SLEW_UPDATE_INTERVAL = 50; // ms
extern bool FAST_SLEW_MODE; // togglable by beep code

const int FORWARD_THRESHOLD = JOYSTICK_CENTER + JOYSTICK_DEADZONE;  // threshold to start forward movement
const int BACKWARD_THRESHOLD = JOYSTICK_CENTER - JOYSTICK_DEADZONE; // threshold to start backward movement
const float FULL_TURN_THRESHOLD = 0.80f;                            // threshold (as a percentage of max X deflection) for full in-place turn
const float FULL_THROTTLE_THRESHOLD = 0.90f;                        // threshold (as a percentage of max Y deflection) for full forward/backward speed
const float BRAKE_APPLY_THRESHOLD = 1.10f * MIN_MOTOR_SPEED;        // Minimum speed to apply braking when stopping
const float TURN_INTENSITY_FACTOR = 0.6f;                           // Factor to scale turn intensity based on joystick deflection, lower = less responsive

#pragma once

// All drive/motor logic for the RECEIVER side
// (mirrors the pattern of 2WD_RC_TRANSMITTER_logic.h/cpp)

int slewRateLimit(int current, int target, float deflection, unsigned long now, unsigned long& lastSlewUpdate);
MotorTargets computeMotorTargets(const JoystickProcessingResult& js, const MotorTargets& prevMt);
bool shouldSkipSlewRate(int prevLeft, int prevRight, int targetLeft, int targetRight);
bool shouldApplyBraking(int prevLeft, int prevRight, int targetLeft, int targetRight);
JoystickProcessingResult processJoystick(int joystickX, int joystickY, bool joystickButton);

