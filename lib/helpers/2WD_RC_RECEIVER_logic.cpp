#include "2WD_RC_RECEIVER_logic.h"
#include <stdlib.h>
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <algorithm>
#include <cmath>
#endif

// For native builds, define constrain if not available
#ifndef ARDUINO
template <typename T>
T constrain(T val, T min_val, T max_val)
{
    return std::min(std::max(val, min_val), max_val);
}
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#ifndef max
#define max(a, b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef abs
#define abs(x) ((x) > 0 ? (x) : -(x))
#endif
#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : (((amt) > (high)) ? (high) : (amt)))
#endif
#ifndef map
#define map(x, in_min, in_max, out_min, out_max) \
    (((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))
#endif

#define digitalWrite(pin, val) ((void)0)
#define analogWrite(pin, val) ((void)0)
#define analogRead(pin) (0)
#define delay(ms) ((void)0)
#define millis() (0UL)
#define tone(pin, freq) ((void)0)
#define noTone(pin) ((void)0)
#define pinMode(pin, mode) ((void)0)

#ifndef A0
#define A0 0
#define A1 1
#define A2 2
#define A3 3
#endif

struct
{
    void begin(int) {}
    void println(int) {}
    void print(int) {}

    void println(const char* s) { printf("%s\n", s); }
    void print(const char* s) { printf("%s", s); }
} Serial;
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

    if (delta > RAMP_STEP)
    {
        next = current + RAMP_STEP;
    }
    else if (delta < -RAMP_STEP)
    {
        next = current - RAMP_STEP;
    }
    else
    {
        next = target;
    }

    // If both target and next are within the motor deadzone, output zero
    if (next > -MOTOR_DEADZONE && next < MOTOR_DEADZONE &&
        target > -MOTOR_DEADZONE && target < MOTOR_DEADZONE)
    {
        return 0;
    }

    // Clamp to zero only when ramping down and crossing below MIN_MOTOR_SPEED in the positive or negative direction
    if (current > 0 && next < MIN_MOTOR_SPEED && next > 0)
    {
        next = 0;
    }
    else if (current < 0 && next > -MIN_MOTOR_SPEED && next < 0)
    {
        next = 0;
    }

    // For zero crossing, clamp to zero if we would end up in the weak motor range
    if (current > 0 && next < 0 && next > -MIN_MOTOR_SPEED)
    {
        next = 0;
    }
    else if (current < 0 && next > 0 && next < MIN_MOTOR_SPEED)
    {
        next = 0;
    }

    // Clamp to min/max if outside allowed range
    if (next > MAX_SPEED)
        next = MAX_SPEED;
    if (next < -MAX_SPEED)
        next = -MAX_SPEED;

    // Clamp to MIN_MOTOR_SPEED when ramping up and next is between deadzone and MIN_MOTOR_SPEED
    if (next > 0 && next >= MOTOR_DEADZONE && next < MIN_MOTOR_SPEED)
    {
        next = MIN_MOTOR_SPEED;
    }
    else if (next < 0 && next <= -MOTOR_DEADZONE && next > -MIN_MOTOR_SPEED)
    {
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
MotorTargets computeMotorTargets(const JoystickProcessingResult &js, int prevLeft, int prevRight)
{
    MotorTargets mt = {};
  
    // Debug print
    char buf[64];
    sprintf(buf, "JS X=%d Y=%d R=%.2f SR=%.2f", js.rawX, js.rawY, js.rawRatioLR, js.steppedRatioLR);
    Serial.println(buf);

    // If the joystick is within the deadzone, stop the motors
    if (abs(js.rawX - JOYSTICK_CENTER) < JOYSTICK_DEADZONE && abs(js.rawY - JOYSTICK_CENTER) < JOYSTICK_DEADZONE) {
        mt.left = 0;
        mt.right = 0;
        mt.skipSlewRate = false;
        return mt;
    }

    static const float JOYSTICK_DEADZONE_RATIO = (float)JOYSTICK_DEADZONE / (float)JOYSTICK_CENTER;

    // In-place turn - when Y stick is near center (512) AND significant X deflection
    if (abs(js.rawY - JOYSTICK_CENTER) < (0.20f * MAX_ADC_VALUE) && fabs(js.steppedRatioLR) >= JOYSTICK_DEADZONE_RATIO)
    {
        // Enforce strict symmetry: always use the same base speed for both left/right in-place turns
        int base_sp = MIN_MOTOR_SPEED + 1;
        float offset_half = LR_OFFSET / 2.0f;

        // Left in-place turn: right forward, left backward (if aggressive / high x-deflection)
        if (js.steppedRatioLR < 0)
        {
            if (js.steppedRatioLR <= -0.75f)
            {
                mt.left = min(-base_sp - offset_half, -MIN_MOTOR_SPEED);
            }
            else
            {
                mt.left = 0; // gentle wider turn
            }

            mt.right = max(base_sp + offset_half, MIN_MOTOR_SPEED);
        }
        // Right in-place turn: left forward, right backward
        else if (js.steppedRatioLR > 0)
        {
            if (js.steppedRatioLR >= 0.75f)
                mt.right = min(-base_sp - offset_half, -MIN_MOTOR_SPEED);
            else
                mt.right = 0;

            mt.left = max(base_sp + offset_half, MIN_MOTOR_SPEED);
        }

        mt.skipSlewRate = (fabs(js.steppedRatioLR) >= 0.90f); // near full x-deflection

        return mt;
    }

    // Forward/backward scaling - using raw Y values
    int sp = 0;
    float offset_half = LR_OFFSET / 2.0f;
    if (js.rawY > FORWARD_THRESHOLD)
    {
        sp = map(constrain(js.rawY, FORWARD_THRESHOLD, MAX_ADC_VALUE), FORWARD_THRESHOLD, MAX_ADC_VALUE, MIN_MOTOR_SPEED, MAX_SPEED);
        mt.left = sp - offset_half;
        mt.right = sp + offset_half;

        return mt;
    }
    else if (js.rawY < BACKWARD_THRESHOLD)
    {
        sp = map(constrain(js.rawY, 0, BACKWARD_THRESHOLD), 0, BACKWARD_THRESHOLD, -MAX_SPEED, -MIN_MOTOR_SPEED);
        mt.left = sp + offset_half; // For reverse, swap sign
        mt.right = sp - offset_half;

        if (fabs(js.steppedRatioLR) <= 0.15f)
            mt.skipSlewRate = shouldSkipSlewRate(prevLeft, prevRight, mt.left, mt.right);
        return mt; // ignore turn tank mixing at low x-deflection
    }

    // Default: simple tank mixing (convert to centered values for mixing)
    int centeredY = js.rawY - JOYSTICK_CENTER; // Convert to -512 to +511 for mixing
    int centeredX = js.rawX - JOYSTICK_CENTER; // Convert to -512 to +511 for mixing
    mt.left = centeredY + centeredX;
    mt.right = centeredY - centeredX;
    int turn = centeredX / 2;
    mt.left = constrain(mt.left + turn, -MAX_SPEED, MAX_SPEED);
    mt.right = constrain(mt.right - turn, -MAX_SPEED, MAX_SPEED);
    // Apply LR_OFFSET as L/R balance if moving in same direction
    if ((mt.left > 0 && mt.right > 0))
    {
        mt.left -= offset_half;
        mt.right += offset_half;
    }
    else if ((mt.left < 0 && mt.right < 0))
    {
        mt.left += offset_half;
        mt.right -= offset_half;
    }

    mt.skipSlewRate = shouldSkipSlewRate(prevLeft, prevRight, mt.left, mt.right);

    return mt;
}

bool shouldSkipSlewRate(int prevLeft, int prevRight, int targetLeft, int targetRight)
{
    // Skip slew rate if direction reverses
    return ((prevLeft > 0 && targetLeft < 0) || (prevLeft < 0 && targetLeft > 0) ||
            (prevRight > 0 && targetRight < 0) || (prevRight < 0 && targetRight > 0));
}

bool shouldApplyBraking(int prevLeft, int prevRight, int targetLeft, int targetRight)
{
    // Only brake when stopping from motion (not on direction reversal)
    return (targetLeft == 0 && targetRight == 0 &&
            (abs(prevLeft) > 0 || abs(prevRight) > 0));
}

JoystickProcessingResult processJoystick(int joystickX, int joystickY, bool joystickButton)
{
    JoystickProcessingResult js;

    // Keep raw values throughout - no conversion needed
    js.rawX = joystickX; // 0-MAX_ADC_VALUE, center = 512
    js.rawY = joystickY; // 0-MAX_ADC_VALUE, center = 512

    js.buzzerOn = joystickButton;

    // Raw turning ratio based on deviation from center (512)
    js.rawRatioLR = ((float)(js.rawX - JOYSTICK_CENTER)) / 512.0f;
    js.rawRatioLR = constrain(js.rawRatioLR, -1.0f, 1.0f);

    const float quantizeStep = 0.05f; // value if which steppedRatioLR is quantized / rounded to
    
    if (js.rawRatioLR > 0)
        js.steppedRatioLR = ceil(js.rawRatioLR / quantizeStep) * quantizeStep;  // round up for positive values (to favor turning)  
    else
        js.steppedRatioLR = floor(js.rawRatioLR / quantizeStep) * quantizeStep;

    // Debug output for diagnosis

    return js;
}
