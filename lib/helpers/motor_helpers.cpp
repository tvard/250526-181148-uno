#include "helpers.h"
#include <stdlib.h>
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <algorithm>    // for std::min, std::max
#include <cmath>        //for round()
#endif

// For native builds, define constrain if not available
#ifndef ARDUINO
template<typename T>
T constrain(T val, T min_val, T max_val) {
    return std::min(std::max(val, min_val), max_val);
}
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#endif

// Helper function for slew-rate control (formerly lambda in manualMode)
/// @brief Limits the rate of change between the current and target speed values to ensure smooth acceleration and deceleration.
/// @param current The current speed value.
/// @param target The desired target speed value.
/// @return The new speed value after applying the slew rate limit.
int slewRateLimit(int current, int target)
{
    int delta = target - current;
    int next;

    if (delta > RAMP_STEP) {
        next = current + RAMP_STEP;
    } else if (delta < -RAMP_STEP) {
        next = current - RAMP_STEP;
    } else {
        next = target;
    }

    // If both target and next are within the deadzone, output zero
    if (next > -JOYSTICK_DEADZONE && next < JOYSTICK_DEADZONE &&
        target > -JOYSTICK_DEADZONE && target < JOYSTICK_DEADZONE) {
        return 0;
    }

    // Clamp to zero only when ramping down and crossing below MIN_MOTOR_SPEED in the positive or negative direction
    if (current > 0 && next < MIN_MOTOR_SPEED && next > 0) {
        next = 0;
    } else if (current < 0 && next > -MIN_MOTOR_SPEED && next < 0) {
        next = 0;
    }

    // Clamp to min/max if outside allowed range
    if (next > MAX_SPEED) next = MAX_SPEED;
    if (next < -MAX_SPEED) next = -MAX_SPEED;

    // Clamp to MIN_MOTOR_SPEED when ramping up and next is between deadzone and MIN_MOTOR_SPEED
    if (next > 0 && next >= JOYSTICK_DEADZONE && next < MIN_MOTOR_SPEED) {
        next = MIN_MOTOR_SPEED;
    } else if (next < 0 && next <= -JOYSTICK_DEADZONE && next > -MIN_MOTOR_SPEED) {
        next = -MIN_MOTOR_SPEED;
    }

    return next;
}


/// @brief Computes the motor targets based on joystick input and previous speeds.
/// @param js The joystick processing result.
/// @param prevLeft The previous left motor speed.
/// @param prevRight The previous right motor speed.
/// @return The computed motor targets.
/// @remarks
/// Forward/backward is controlled by correctedY, Left/Right is controlled by correctedX
/// The mixing formula is standard for differential drive (tank drive)
MotorTargets computeMotorTargets(const JoystickProcessingResult& js, int prevLeft, int prevRight) {
    MotorTargets mt = {};
    float quantizeStep = js.quantizeStep;
    float steppedRatioLR = js.steppedRatioLR;
    int correctedY = js.correctedY + 512;

    // In-place turn (sharp turn) scenario
    if (abs(correctedY - 512) < (0.20 * 1023.0) && fabs(steppedRatioLR) >= quantizeStep * 1.0) {
        int sp = map((int)(fabs(steppedRatioLR) * 100), 0, 100, MIN_MOTOR_SPEED, MIN_MOTOR_SPEED + 5);
        if (steppedRatioLR < 0) {
            mt.right = sp;
            if (steppedRatioLR <= -0.9)
                mt.left = -sp;
        } else if (steppedRatioLR > 0) {
            mt.left = sp;
            if (steppedRatioLR >= +0.9)
                mt.right = -sp;
        }
        mt.skipSlewRate = true;
        return mt;
    }

    // Forward/backward scaling
    int sp = 0;
    if (correctedY > FORWARD_THRESHOLD) {
        sp = map(constrain(correctedY, FORWARD_THRESHOLD, 1023), FORWARD_THRESHOLD, 1023, MIN_MOTOR_SPEED, MAX_SPEED);
        mt.left = sp;
        mt.right = sp;
        mt.skipSlewRate = shouldSkipSlewRate(prevLeft, prevRight, mt.left, mt.right);
        return mt;
    } else if (correctedY < BACKWARD_THRESHOLD) {
        sp = map(constrain(correctedY, 0, BACKWARD_THRESHOLD), 0, BACKWARD_THRESHOLD, -MAX_SPEED, -MIN_MOTOR_SPEED);
        mt.left = sp;
        mt.right = sp;
        mt.skipSlewRate = shouldSkipSlewRate(prevLeft, prevRight, mt.left, mt.right);
        return mt;
    }

    // Default: simple tank mixing + gentle turn
    mt.left = js.correctedY + js.correctedX;
    mt.right = js.correctedY - js.correctedX;
    int turn = js.correctedX / 2;
    mt.left = constrain(mt.left + turn, -MAX_SPEED, MAX_SPEED);
    mt.right = constrain(mt.right - turn, -MAX_SPEED, MAX_SPEED);

    // If the joystick is within the deadzone, stop the motors
    if (abs(js.correctedX) < JOYSTICK_DEADZONE && abs(js.correctedY) < JOYSTICK_DEADZONE) {
        mt.left = 0;
        mt.right = 0;
        mt.skipSlewRate = false;
        return mt;
    }

    mt.skipSlewRate = shouldSkipSlewRate(prevLeft, prevRight, mt.left, mt.right);
    return mt;
}

bool shouldSkipSlewRate(int prevLeft, int prevRight, int targetLeft, int targetRight) {
    // Skip slew rate if direction reverses
    return ((prevLeft > 0 && targetLeft < 0) || (prevLeft < 0 && targetLeft > 0) ||
            (prevRight > 0 && targetRight < 0) || (prevRight < 0 && targetRight > 0));
}

bool shouldApplyBraking(int prevLeft, int prevRight, int targetLeft, int targetRight) {
    // Only brake when stopping from motion (not on direction reversal)
    return (targetLeft == 0 && targetRight == 0 &&
            (abs(prevLeft) > 0 || abs(prevRight) > 0));
}

JoystickProcessingResult processJoystick(int joystickX, int joystickY, bool joystickButton) {
    JoystickProcessingResult js;
    js.correctedX = joystickX - 512;
    js.correctedY = joystickY - 512;
    js.buzzerOn = joystickButton;
    js.rawRatioLR = ((float)js.correctedX) / 512.0f;
    js.rawRatioLR = constrain(js.rawRatioLR, -1.0f, 1.0f);

    js.quantizeStep = 0.10f;
    js.steppedRatioLR = round(js.rawRatioLR / js.quantizeStep) * js.quantizeStep;
    return js;
}

ManualModeOutputs manualModeStep(const ManualModeInputs& in) {
    ManualModeOutputs out = {};
    // 1. Process joystick input
    JoystickProcessingResult js = processJoystick(in.joystick.correctedX, in.joystick.correctedY, in.joystick.buzzerOn);

    // 2. Compute motor targets and slew skip
    MotorTargets mt = computeMotorTargets(js, in.leftSpeed, in.rightSpeed);

    // 3. Slew rate logic
    int leftSpeed = in.leftSpeed;
    int rightSpeed = in.rightSpeed;
    if (mt.skipSlewRate) {
        leftSpeed = mt.left;
        rightSpeed = mt.right;
    } else {
        if (leftSpeed != mt.left)
            leftSpeed = slewRateLimit(leftSpeed, mt.left);
        if (rightSpeed != mt.right)
            rightSpeed = slewRateLimit(rightSpeed, mt.right);
    }

    // 4. Braking logic
    bool braking = shouldApplyBraking(in.prevLeftSpeed, in.prevRightSpeed, mt.left, mt.right);
    out.brakingApplied = braking;
    if (braking) {
        out.outputLeft = -in.prevLeftSpeed / 4;
        out.outputRight = -in.prevRightSpeed / 4;
        leftSpeed = 0;
        rightSpeed = 0;
    } else {
        out.outputLeft = (abs(leftSpeed) >= MIN_MOTOR_SPEED) ? leftSpeed : 0;
        out.outputRight = (abs(rightSpeed) >= MIN_MOTOR_SPEED) ? rightSpeed : 0;
    }

    out.leftSpeed = leftSpeed;
    out.rightSpeed = rightSpeed;
    out.skipSlewRate = mt.skipSlewRate;
    out.buzzerOn = in.joystick.buzzerOn; // or your buzzer logic

    return out;
}


