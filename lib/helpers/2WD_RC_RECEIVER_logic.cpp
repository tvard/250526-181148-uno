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
// Variable slew rate: pass in joystick deflection (0.0-1.0) to select slew rate
int slewRateLimit(int current, int target, float deflection)
{
    int rampStep = RAMP_STEP_SLOW;
    if (deflection >= FULL_THROTTLE_THRESHOLD || deflection >= FULL_TURN_THRESHOLD) {
        rampStep = RAMP_STEP_FAST;
    }

    int delta = target - current;
    int next;

    if (delta > rampStep)
        next = current + rampStep;
    else if (delta < -rampStep)
        next = current - rampStep;
    else
        next = target;

    // 1. Clamp to 0 if both target and next are in the deadzone
    if (next > -MOTOR_DEADZONE && next < MOTOR_DEADZONE &&
        target > -MOTOR_DEADZONE && target < MOTOR_DEADZONE)
        return 0;

    // 2. Always clamp to MIN_MOTOR_SPEED when ramping up from 0, regardless of step size
    if (current == 0 && next > 0)
        next = MIN_MOTOR_SPEED;
    else if (current == 0 && next < 0)
        next = -MIN_MOTOR_SPEED;

    // 3. Special case: when target is 0 and current > MIN_MOTOR_SPEED, clamp directly to 0 
    if (target == 0 && current > MIN_MOTOR_SPEED)
        return 0;
    else if (target == 0 && current < -MIN_MOTOR_SPEED)
        return 0;

    // 4. Special case: when target is MIN_MOTOR_SPEED and current < MIN_MOTOR_SPEED, clamp directly to MIN_MOTOR_SPEED
    if (target == MIN_MOTOR_SPEED && current > 0 && current < MIN_MOTOR_SPEED)
        return MIN_MOTOR_SPEED;
    else if (target == -MIN_MOTOR_SPEED && current < 0 && current > -MIN_MOTOR_SPEED)
        return -MIN_MOTOR_SPEED;

    // 4a. Special case: when target is MAX_SPEED and current is near MAX_SPEED, clamp directly to MAX_SPEED
    if (target == MAX_SPEED && current > 0 && current < MAX_SPEED)
        return MAX_SPEED;
    else if (target == -MAX_SPEED && current < 0 && current > -MAX_SPEED)
        return -MAX_SPEED;

    // 5. Special case: when crossing MIN_MOTOR_SPEED downward and target < MIN_MOTOR_SPEED, clamp to 0
    if (current > MIN_MOTOR_SPEED && target > 0 && target < MIN_MOTOR_SPEED)
        return 0;
    else if (current < -MIN_MOTOR_SPEED && target < 0 && target > -MIN_MOTOR_SPEED)
        return 0;

    // 5a. Special case: zero crossing - when going from positive to negative or vice versa, clamp to 0 for small current values
    if (current > 0 && target < 0 && current < MIN_MOTOR_SPEED)
        return 0;
    else if (current < 0 && target > 0 && abs(current) < MIN_MOTOR_SPEED)
        return 0;

    // 6. Clamp to 0 when ramping down and next would be between 0 and MIN_MOTOR_SPEED (or -MIN_MOTOR_SPEED)
    if (current > 0 && target == 0 && next < MIN_MOTOR_SPEED && next > 0)
        return 0;
    else if (current < 0 && target == 0 && next > -MIN_MOTOR_SPEED && next < 0)
        return 0;

    // 7. Clamp to MAX_SPEED if outside allowed range
    if (next > MAX_SPEED)
        next = MAX_SPEED;
    if (next < -MAX_SPEED)
        next = -MAX_SPEED;

    // 8. Dynamic braking: only apply when target==0 AND we're going from high speed to exactly target AND high speed motion
    if (target == 0 && current != 0 && next == target && abs(current) >= BRAKE_APPLY_THRESHOLD)
        next = -MIN_MOTOR_SPEED * (current > 0 ? 1 : -1);

    return next;
}

/// @brief Computes the motor targets based on joystick input and previous motor targets.
/// @param js The joystick processing result.
/// @param prevMt The previous MotorTargets struct (for previous output values).
/// @return The computed motor targets.
/// @remarks
/// Forward/backward is controlled by rawY, Left/Right is controlled by rawX
/// The mixing formula is standard for differential drive (tank drive)
MotorTargets computeMotorTargets(const JoystickProcessingResult &js, const MotorTargets &prevMt)
{
    MotorTargets mt = {};
    int nextLeft = prevMt.outputLeft;
    int nextRight = prevMt.outputRight;

    unsigned long now = millis();

    // If the joystick is within the deadzone, stop the motors
    bool joystickCentered = (abs(js.rawX - JOYSTICK_CENTER) < JOYSTICK_DEADZONE && abs(js.rawY - JOYSTICK_CENTER) < JOYSTICK_DEADZONE);
    if (joystickCentered)
    {
        mt.targetLeft = 0;
        mt.targetRight = 0;
        mt.skipSlewRate = false;
        
        // Only apply dynamic braking once when transitioning to stop, not if already braking
        mt.brakingApplied = !prevMt.brakingApplied && shouldApplyBraking(prevMt.outputLeft, prevMt.outputRight, 0, 0);
        
        if (mt.brakingApplied)
        {
            // Set both targets and outputs for dynamic braking
            int leftBrakeDirection = (prevMt.outputLeft > 0) ? 1 : -1;
            int rightBrakeDirection = (prevMt.outputRight > 0) ? 1 : -1;
            mt.targetLeft = -MIN_MOTOR_SPEED * leftBrakeDirection;
            mt.targetRight = -MIN_MOTOR_SPEED * rightBrakeDirection;
            mt.outputLeft = mt.targetLeft;
            mt.outputRight = mt.targetRight;
        }
        else
        {
            // Slew rate limit toward zero
            if (nextLeft != 0)
                nextLeft = slewRateLimit(nextLeft, 0, 0.0f);
            if (nextRight != 0)
                nextRight = slewRateLimit(nextRight, 0, 0.0f);
            mt.outputLeft = (abs(nextLeft) >= MIN_MOTOR_SPEED) ? nextLeft : 0;
            mt.outputRight = (abs(nextRight) >= MIN_MOTOR_SPEED) ? nextRight : 0;
        }
        return mt;
    }

    static const float JOYSTICK_DEADZONE_RATIO = (float)JOYSTICK_DEADZONE / (float)JOYSTICK_CENTER;

    // In-place turn - when Y stick is near center (512) AND significant X deflection
    if (abs(js.rawY - JOYSTICK_CENTER) < (0.20f * MAX_ADC_VALUE) && fabs(js.steppedRatioLR) >= JOYSTICK_DEADZONE_RATIO)
    {
        int base_sp = MIN_MOTOR_SPEED + 1;
        float offset_half = LR_OFFSET / 2.0f;
        if (js.steppedRatioLR < 0)
        {
            if (js.steppedRatioLR <= -0.75f)
                mt.targetLeft = min(-base_sp - offset_half, -MIN_MOTOR_SPEED);
            else
                mt.targetLeft = 0;
            mt.targetRight = max(base_sp + offset_half, MIN_MOTOR_SPEED);
        }
        else if (js.steppedRatioLR > 0)
        {
            if (js.steppedRatioLR >= 0.75f)
                mt.targetRight = min(-base_sp - offset_half, -MIN_MOTOR_SPEED);
            else
                mt.targetRight = 0;
            mt.targetLeft = max(base_sp + offset_half, MIN_MOTOR_SPEED);
        }
        mt.skipSlewRate = (fabs(js.steppedRatioLR) >= 0.90f);
        // Slew rate logic
        if (mt.skipSlewRate) {
            nextLeft = mt.targetLeft;
            nextRight = mt.targetRight;
        } else {
            if (nextLeft != mt.targetLeft)
                nextLeft = slewRateLimit(nextLeft, mt.targetLeft, 0.0f);
            if (nextRight != mt.targetRight)
                nextRight = slewRateLimit(nextRight, mt.targetRight, 0.0f);
        }
        mt.brakingApplied = false;
        mt.outputLeft = (abs(nextLeft) >= MIN_MOTOR_SPEED) ? nextLeft : 0;
        mt.outputRight = (abs(nextRight) >= MIN_MOTOR_SPEED) ? nextRight : 0;
        return mt;
    }

    // Forward/backward scaling - using raw Y values
    int sp = 0;
    float offset_half = LR_OFFSET / 2.0f;
    if (js.rawY > FORWARD_THRESHOLD)
    {
        sp = map(constrain(js.rawY, FORWARD_THRESHOLD, MAX_ADC_VALUE), FORWARD_THRESHOLD, MAX_ADC_VALUE, MIN_MOTOR_SPEED, MAX_SPEED);
        mt.targetLeft = sp - offset_half;
        mt.targetRight = sp + offset_half;
        mt.skipSlewRate = shouldSkipSlewRate(prevMt.outputLeft, prevMt.outputRight, mt.targetLeft, mt.targetRight);
        
        // Calculate deflection for variable slew rate (how far from center toward max)
        float deflection = (float)(js.rawY - FORWARD_THRESHOLD) / (float)(MAX_ADC_VALUE - FORWARD_THRESHOLD);
        
        if (mt.skipSlewRate) {
            nextLeft = mt.targetLeft;
            nextRight = mt.targetRight;
        } else {
            if (nextLeft != mt.targetLeft)
                nextLeft = slewRateLimit(nextLeft, mt.targetLeft, deflection);
            if (nextRight != mt.targetRight)
                nextRight = slewRateLimit(nextRight, mt.targetRight, deflection);
        }
        mt.brakingApplied = false;
        mt.outputLeft = (abs(nextLeft) >= MIN_MOTOR_SPEED) ? nextLeft : 0;
        mt.outputRight = (abs(nextRight) >= MIN_MOTOR_SPEED) ? nextRight : 0;
        return mt;
    }
    else if (js.rawY < BACKWARD_THRESHOLD)
    {
        sp = map(constrain(js.rawY, 0, BACKWARD_THRESHOLD), 0, BACKWARD_THRESHOLD, -MAX_SPEED, -MIN_MOTOR_SPEED);
        mt.targetLeft = sp + offset_half; // For reverse, swap sign
        mt.targetRight = sp - offset_half;
        mt.skipSlewRate = shouldSkipSlewRate(prevMt.outputLeft, prevMt.outputRight, mt.targetLeft, mt.targetRight);
        
        // Calculate deflection for variable slew rate (how far from center toward min)  
        float deflection = (float)(BACKWARD_THRESHOLD - js.rawY) / (float)(BACKWARD_THRESHOLD - 0);
        
        if (mt.skipSlewRate) {
            nextLeft = mt.targetLeft;
            nextRight = mt.targetRight;
        } else {
            if (nextLeft != mt.targetLeft)
                nextLeft = slewRateLimit(nextLeft, mt.targetLeft, deflection);
            if (nextRight != mt.targetRight)
                nextRight = slewRateLimit(nextRight, mt.targetRight, deflection);
        }
        mt.brakingApplied = false;
        mt.outputLeft = (abs(nextLeft) >= MIN_MOTOR_SPEED) ? nextLeft : 0;
        mt.outputRight = (abs(nextRight) >= MIN_MOTOR_SPEED) ? nextRight : 0;
        return mt; // ignore turn tank mixing at low x-deflection
    }

    // Default: simple tank mixing (convert to centered values for mixing)
    int centeredY = js.rawY - JOYSTICK_CENTER; // Convert to -512 to +511 for mixing
    int centeredX = js.rawX - JOYSTICK_CENTER; // Convert to -512 to +511 for mixing
    mt.targetLeft = centeredY + centeredX;
    mt.targetRight = centeredY - centeredX;
    int turn = centeredX / 2;
    mt.targetLeft = constrain(mt.targetLeft + turn, -MAX_SPEED, MAX_SPEED);
    mt.targetRight = constrain(mt.targetRight - turn, -MAX_SPEED, MAX_SPEED);
    // Apply LR_OFFSET as L/R balance if moving in same direction
    if ((mt.targetLeft > 0 && mt.targetRight > 0))
    {
        mt.targetLeft -= offset_half;
        mt.targetRight += offset_half;
    }
    else if ((mt.targetLeft < 0 && mt.targetRight < 0))
    {
        mt.targetLeft += offset_half;
        mt.targetRight -= offset_half;
    }

    const unsigned long SLEW_UPDATE_INTERVAL = 20; // ms
    static unsigned long lastSlewUpdate = 0;

    mt.skipSlewRate = shouldSkipSlewRate(prevMt.outputLeft, prevMt.outputRight, mt.targetLeft, mt.targetRight);
    if (mt.skipSlewRate) {
        nextLeft = mt.targetLeft;
        nextRight = mt.targetRight;
    } 
    else if (now - lastSlewUpdate < SLEW_UPDATE_INTERVAL) {
        nextLeft = prevMt.outputLeft;
        nextRight = prevMt.outputRight;
    } 
    else {
        if (nextLeft != mt.targetLeft)
            nextLeft = slewRateLimit(nextLeft, mt.targetLeft, 0.0f);
        if (nextRight != mt.targetRight)
            nextRight = slewRateLimit(nextRight, mt.targetRight, 0.0f);
    }

    mt.brakingApplied = false;
    mt.outputLeft = (abs(nextLeft) >= MIN_MOTOR_SPEED) ? nextLeft : 0;
    mt.outputRight = (abs(nextRight) >= MIN_MOTOR_SPEED) ? nextRight : 0;
    return mt;
}


/// @brief Determine if the slew rate should be skipped based on joystick input.
/// @param prevLeft The previous left motor speed.
/// @param prevRight The previous right motor speed.
/// @param targetLeft The target left motor speed.
/// @param targetRight The target right motor speed.
/// @return True if the slew rate should be skipped, false otherwise.
bool shouldSkipSlewRate(int prevLeft, int prevRight, int targetLeft, int targetRight)
{
    // Skip slew rate if direction reverses
    if ((prevLeft > 0 && targetLeft < 0) || (prevLeft < 0 && targetLeft > 0) ||
            (prevRight > 0 && targetRight < 0) || (prevRight < 0 && targetRight > 0))
        return true;

    // skip if aggressive turn (sharp left/right)
    if ((targetLeft <= -MIN_MOTOR_SPEED && targetRight >= MIN_MOTOR_SPEED) ||
        (targetLeft >= MIN_MOTOR_SPEED && targetRight <= -MIN_MOTOR_SPEED))
        return true;

    return false;
}

bool shouldApplyBraking(int prevLeft, int prevRight, int targetLeft, int targetRight)
{
    // Only brake when stopping from motion (not on direction reversal)
    return (abs(targetLeft) <= MIN_MOTOR_SPEED && abs(targetRight) <= MIN_MOTOR_SPEED &&
        (abs(prevLeft) >= BRAKE_APPLY_THRESHOLD || abs(prevRight) >= BRAKE_APPLY_THRESHOLD));
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
