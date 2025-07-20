#include "helpers.h"
#include <stdlib.h>
#ifdef ARDUINO
#include <Arduino.h>
#else
// For native builds, define constrain if not available
#include <iostream>
#include <iomanip>  // for std::setprecision
#include <algorithm>    // for std::min, std::max
#include <cmath>        //for round()

template<typename T, typename U, typename V>
T constrain(T val, const U& min_val, const V& max_val) {
    return std::min(std::max(val, static_cast<T>(min_val)), static_cast<T>(max_val));
}
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class Serial {
public:
    static void print(int val) { printf("%d", val); }
    static void println(int val) { printf("%d\n", val); }
    static void print(const char* str) { printf("%s", str); }
    static void println(const char* str) { printf("%s\n", str); }
};

// You can also define a static instance of the Serial class
Serial Serial;

#endif

const float SHARP_TURN_THRESHOLD = 0.75f;

// Helper function for slew-rate control (formerly lambda in manualMode)
/// @brief Limits the rate of change between the current and target speed values to ensure smooth acceleration and deceleration.
/// @param current The current speed value.
/// @param target The desired target speed value.
/// @return The new speed value after applying the slew rate limit.
int slewRateLimit(int current, int target)
{
    int delta = target - current;
    int next;

    if (abs(target) <= JOYSTICK_DEADZONE) { return 0; }

    if (delta > RAMP_STEP) {
        next = current + RAMP_STEP;
    } else if (delta < -RAMP_STEP) {
        next = current - RAMP_STEP;
    } else {
        next = target;
    }

    // If both target and next are within the deadzone, output zero
    if (abs(next) <= JOYSTICK_DEADZONE) { return 0; }

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

bool isInDeadzone(int val) {
    return val >= BACKWARD_THRESHOLD && val <= FORWARD_THRESHOLD;
};


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

    // If the joystick is within the deadzone, stop the motors
    if (isInDeadzone(js.correctedX) && isInDeadzone(js.correctedY)) {

        mt.left = 0;
        mt.right = 0;
        mt.skipSlewRate = true;
        return mt;
    }
    

    // In-place turn (gentle / sharp turn) scenario
    if (isInDeadzone(js.correctedY) && !isInDeadzone(js.correctedX)) {
        int sp = map((int)(fabs(js.steppedRatioLR) * 100), 0, 100, MIN_MOTOR_SPEED, MIN_MOTOR_SPEED + 10); // capped speed

        // char buf[64];
        // snprintf(buf, sizeof(buf), "In-place turn, speed: %d | LR Ratio: %0.2f \n", sp, js.steppedRatioLR);
        // Serial.println(buf);

        mt.skipSlewRate = false;

        if (js.steppedRatioLR < 0) {
            mt.right = sp;
    
            if (js.steppedRatioLR <= -SHARP_TURN_THRESHOLD)  { // fast
                mt.left = -sp;
                mt.skipSlewRate = true;
            }
            else
                mt.left = 0;   // one-wheel only => slower turn
            
        } else if (js.steppedRatioLR > 0) {
            mt.left = sp;

            if (js.steppedRatioLR >= +SHARP_TURN_THRESHOLD)  { // fast
                mt.right = -sp;
                mt.skipSlewRate = true; 
            }
            else
                mt.right = 0;   // one-wheel only => slower turn
        }
        return mt;
    }

    // Forward/backward scaling
    int sp = 0;
    if (js.correctedY > FORWARD_THRESHOLD) {
        sp = map(js.correctedY, 512, 1023, MIN_MOTOR_SPEED, MAX_SPEED);
        mt.left = sp;
        mt.right = sp;
    } else if (js.correctedY < BACKWARD_THRESHOLD) {
        sp = map(js.correctedY, 0, 512, -MAX_SPEED, -MIN_MOTOR_SPEED);
        mt.left = sp;
        mt.right = sp;
    }

    // Trim left and right speeds according to LR ratio
    if (js.steppedRatioLR != 0) {

        int diff = abs(sp * js.steppedRatioLR);
        
        if (js.steppedRatioLR < 0) { 
            mt.left = sp - diff;
            mt.right = sp;
        } else if (js.steppedRatioLR > 0) {
            mt.left = sp;
            mt.right = sp - diff;
        }
    }

    mt.skipSlewRate = shouldSkipSlewRate(prevLeft, prevRight, mt.left, mt.right);

    return mt;
}

bool shouldSkipSlewRate(int prevLeft, int prevRight, int targetLeft, int targetRight) {
    const int DIRECTION_CHANGE_THRESHOLD = 20; // threshold to ignore small reversals
    bool leftReverses = (prevLeft > 0 && targetLeft < 0 && abs(prevLeft - targetLeft) > DIRECTION_CHANGE_THRESHOLD) ||
                        (prevLeft < 0 && targetLeft > 0 && abs(prevLeft - targetLeft) > DIRECTION_CHANGE_THRESHOLD);
    bool rightReverses = (prevRight > 0 && targetRight < 0 && abs(prevRight - targetRight) > DIRECTION_CHANGE_THRESHOLD) ||
                         (prevRight < 0 && targetRight > 0 && abs(prevRight - targetRight) > DIRECTION_CHANGE_THRESHOLD);
    return leftReverses || rightReverses;
}

bool shouldApplyBraking(int prevLeft, int prevRight, int targetLeft, int targetRight) {
    // Only brake when stopping from motion (not on direction reversal)
    return (targetLeft == 0 && targetRight == 0 &&
            (abs(prevLeft) > 0 || abs(prevRight) > 0));
}
JoystickProcessingResult processJoystick(int joystickX, int joystickY, bool joystickButton, bool isRaw = true) {
    JoystickProcessingResult js;

    if (isRaw) {
        // Center at zero and scale: from 0-1023 
        int tempX = joystickX;
        int tempY = joystickY; 

        // Axes are swapped (X <-> Y)
        js.correctedX = 1024 - tempY;
        js.correctedY = 1024 - tempX;
    }
    else {
        js.correctedX = joystickX;
        js.correctedY = joystickY;
    }

    js.buzzerOn = joystickButton;

    // Raw turning ratio
    js.rawRatioLR = ((float)(js.correctedX - 512)) / 512.0f;
    js.rawRatioLR = constrain(js.rawRatioLR, -1.0f, 1.0f);

    // Quantized ratio
    js.quantizeStep = 0.05f;
    js.steppedRatioLR = round(js.rawRatioLR / js.quantizeStep) * js.quantizeStep;
        
    return js;
}


ManualModeOutputs manualModeStep(const ManualModeInputs& in) {
    ManualModeOutputs out = {};
    // 1. Process joystick input
    JoystickProcessingResult js = in.joystick;

    // Serial.print("JS: "); Serial.print(js.correctedX); Serial.print(", "); Serial.println(js.correctedY); // #ignore
    // Serial.print("LR (Prev): "); Serial.print(in.leftSpeed); Serial.print(", "); Serial.println(in.rightSpeed);

    // 2. Compute motor targets and slew skip
    MotorTargets mt = computeMotorTargets(js, in.leftSpeed, in.rightSpeed);
    
    // 3. Slew rate 
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

    // 4. Braking 
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
    out.buzzerOn = in.joystick.buzzerOn; 

    return out;
}


