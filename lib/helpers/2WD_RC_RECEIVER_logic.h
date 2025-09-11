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