
// Motor/receiver-specific constants
const int LR_OFFSET         = 0;  // Left/Right balance offset, +ve = more right speed, -ve = more left speed
const int MIN_DISTANCE     = 30;    // Minimum distance in cm before turning
const int TURN_TIME        = 800;   // Time to turn in milliseconds
const int SCAN_INTERVAL    = 300;   // Time between distance measurements
const int MAX_SPEED        = 255;
const int MIN_MOTOR_SPEED  = 70;    // minimum speed to avoid stalling, can be adjusted
const int MOTOR_DEADZONE   = 10;    // motor values within this range of zero are considered stopped

// Receiver-specific drive/motor logic constants
const int RAMP_STEP_SLOW = 20;      // slow slew rate step (for gentle joystick deflection)
const int RAMP_STEP_FAST = 60;      // fast slew rate step (for aggressive joystick deflection)
const int RAMP_STEP = RAMP_STEP_SLOW; // default for compatibility



#pragma once

#include "helpers.h"

// All drive/motor logic for the RECEIVER side
// (mirrors the pattern of 2WD_RC_TRANSMITTER_logic.h/cpp)

int slewRateLimit(int current, int target, float deflection);  // applies slew rate limiting to move current speed towards target speed
MotorTargets computeMotorTargets(const JoystickProcessingResult& js, int prevLeft, int prevRight);
bool shouldSkipSlewRate(int prevLeft, int prevRight, int targetLeft, int targetRight);
bool shouldApplyBraking(int prevLeft, int prevRight, int targetLeft, int targetRight);
JoystickProcessingResult processJoystick(int joystickX, int joystickY, bool joystickButton);


const int FORWARD_THRESHOLD = JOYSTICK_CENTER + JOYSTICK_DEADZONE;  // threshold to start forward movement
const int BACKWARD_THRESHOLD = JOYSTICK_CENTER - JOYSTICK_DEADZONE; // threshold to start backward movement
const float FULL_TURN_THRESHOLD = 0.80f;                 // threshold (as a percentage of max X deflection) for full in-place turn
const float FULL_THROTTLE_THRESHOLD = 0.80f;             // threshold (as a percentage of max Y deflection) for full forward/backward speed
const float SLOW_SLEW_THRESHOLD = 0.50f;                 // below this, slew rate is slow
const float BRAKE_APPLY_THRESHOLD = 0.80f * MAX_SPEED;     // Minimum speed to apply braking when stopping