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

    // If both target and next are within the motor deadzone, output zero
    if (next > -MOTOR_DEADZONE && next < MOTOR_DEADZONE &&
        target > -MOTOR_DEADZONE && target < MOTOR_DEADZONE) {
        return 0;
    }

    // Clamp to zero only when ramping down and crossing below MIN_MOTOR_SPEED in the positive or negative direction
    if (current > 0 && next < MIN_MOTOR_SPEED && next > 0) {
        next = 0;
    } else if (current < 0 && next > -MIN_MOTOR_SPEED && next < 0) {
        next = 0;
    }
    
    // For zero crossing, clamp to zero if we would end up in the weak motor range
    if (current > 0 && next < 0 && next > -MIN_MOTOR_SPEED) {
        next = 0;
    } else if (current < 0 && next > 0 && next < MIN_MOTOR_SPEED) {
        next = 0;
    }

    // Clamp to min/max if outside allowed range
    if (next > MAX_SPEED) next = MAX_SPEED;
    if (next < -MAX_SPEED) next = -MAX_SPEED;

    // Clamp to MIN_MOTOR_SPEED when ramping up and next is between deadzone and MIN_MOTOR_SPEED
    if (next > 0 && next >= MOTOR_DEADZONE && next < MIN_MOTOR_SPEED) {
        next = MIN_MOTOR_SPEED;
    } else if (next < 0 && next <= -MOTOR_DEADZONE && next > -MIN_MOTOR_SPEED) {
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
/// Forward/backward is controlled by rawY, Left/Right is controlled by rawX
/// The mixing formula is standard for differential drive (tank drive)
MotorTargets computeMotorTargets(const JoystickProcessingResult& js, int prevLeft, int prevRight) {
    MotorTargets mt = {};
    float quantizeStep = js.quantizeStep;
    float steppedRatioLR = js.steppedRatioLR;

    // In-place turn (sharp turn) scenario - when Y stick is near center (512) AND significant X deflection
    if (abs(js.rawY - JOYSTICK_CENTER) < (0.20 * 1023.0) && fabs(steppedRatioLR) >= quantizeStep * 2.0) {
        int sp = map((int)(fabs(steppedRatioLR) * 100), 0, 100, MIN_MOTOR_SPEED, MIN_MOTOR_SPEED + 5);
        if (steppedRatioLR < 0) {
            mt.right = sp + (sp > 0 ? LR_OFFSET : (sp < 0 ? -LR_OFFSET : 0));
            if (steppedRatioLR <= -0.9)
                mt.left = -sp + ((-sp) > 0 ? LR_OFFSET : ((-sp) < 0 ? -LR_OFFSET : 0));
        } else if (steppedRatioLR > 0) {
            mt.left = sp + (sp > 0 ? LR_OFFSET : (sp < 0 ? -LR_OFFSET : 0));
            if (steppedRatioLR >= +0.9)
                mt.right = -sp + ((-sp) > 0 ? LR_OFFSET : ((-sp) < 0 ? -LR_OFFSET : 0));
        }
        mt.skipSlewRate = (fabs(steppedRatioLR) >= quantizeStep * 6.0);  // Only skip slew for very sharp turns
        return mt;
    }

    // Forward/backward scaling - using raw Y values
    int sp = 0;
    if (js.rawY > FORWARD_THRESHOLD) {
        sp = map(constrain(js.rawY, FORWARD_THRESHOLD, 1023), FORWARD_THRESHOLD, 1023, MIN_MOTOR_SPEED, MAX_SPEED);
        mt.left = sp + (sp > 0 ? LR_OFFSET : (sp < 0 ? -LR_OFFSET : 0));
        mt.right = sp + (sp > 0 ? LR_OFFSET : (sp < 0 ? -LR_OFFSET : 0));
        mt.skipSlewRate = shouldSkipSlewRate(prevLeft, prevRight, mt.left, mt.right);
        return mt;
    } else if (js.rawY < BACKWARD_THRESHOLD) {
        sp = map(constrain(js.rawY, 0, BACKWARD_THRESHOLD), 0, BACKWARD_THRESHOLD, -MAX_SPEED, -MIN_MOTOR_SPEED);
        mt.left = sp + (sp > 0 ? LR_OFFSET : (sp < 0 ? -LR_OFFSET : 0));
        mt.right = sp + (sp > 0 ? LR_OFFSET : (sp < 0 ? -LR_OFFSET : 0));
        mt.skipSlewRate = shouldSkipSlewRate(prevLeft, prevRight, mt.left, mt.right);
        return mt;
    }

    // Default: simple tank mixing + gentle turn (convert to centered values for mixing)
    int centeredY = js.rawY - JOYSTICK_CENTER;  // Convert to -512 to +511 for mixing
    int centeredX = js.rawX - JOYSTICK_CENTER;  // Convert to -512 to +511 for mixing
    mt.left = centeredY + centeredX;
    mt.right = centeredY - centeredX;
    int turn = centeredX / 2;
    mt.left = constrain(mt.left + turn, -MAX_SPEED, MAX_SPEED);
    mt.right = constrain(mt.right - turn, -MAX_SPEED, MAX_SPEED);
    // Apply LR_OFFSET to both motors if moving in same direction
    if ((mt.left > 0 && mt.right > 0) || (mt.left < 0 && mt.right < 0)) {
        mt.left += mt.left > 0 ? LR_OFFSET : -LR_OFFSET;
        mt.right += mt.right > 0 ? LR_OFFSET : -LR_OFFSET;
    }

    // If the joystick is within the deadzone, stop the motors
    if (abs(js.rawX - JOYSTICK_CENTER) < JOYSTICK_DEADZONE && abs(js.rawY - JOYSTICK_CENTER) < JOYSTICK_DEADZONE) {
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

JoystickProcessingResult processJoystick(int joystickX, int joystickY, bool joystickButton, bool isRaw) {
    JoystickProcessingResult js;

    // Keep raw values throughout - no conversion needed
    js.rawX = joystickX;  // 0-1023, center = 512
    js.rawY = joystickY;  // 0-1023, center = 512

    js.buzzerOn = joystickButton;

    // Raw turning ratio based on deviation from center (512)
    js.rawRatioLR = ((float)(js.rawX - JOYSTICK_CENTER)) / 512.0f;
    js.rawRatioLR = constrain(js.rawRatioLR, -1.0f, 1.0f);

    // Quantized ratio
    js.quantizeStep = 0.05f;
    js.steppedRatioLR = round(js.rawRatioLR / js.quantizeStep) * js.quantizeStep;

    return js;
}