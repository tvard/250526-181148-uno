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
static unsigned long __test_millis = 0;
#define millis() (__test_millis += 20)
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
int slewRateLimit(int current, int target, float deflection, unsigned long now, unsigned long& lastSlewUpdate)
{
    const unsigned long SLEW_UPDATE_INTERVAL = 20; // ms
    if (now - lastSlewUpdate < SLEW_UPDATE_INTERVAL)
        return current;
    lastSlewUpdate = now;

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
    // DEBUG: Print all relevant values for test diagnosis
    // printf("[computeMotorTargets] rawX=%d rawY=%d | centeredX=%d centeredY=%d\n", js.rawX, js.rawY, js.rawX - JOYSTICK_CENTER, js.rawY - JOYSTICK_CENTER);

    MotorTargets mt = {};
    int nextLeft = prevMt.outputLeft;
    int nextRight = prevMt.outputRight;
    static unsigned long lastSlewUpdateLeft = 0;
    static unsigned long lastSlewUpdateRight = 0;

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
                nextLeft = slewRateLimit(nextLeft, 0, 0.0f, now, lastSlewUpdateLeft);
            if (nextRight != 0)
                nextRight = slewRateLimit(nextRight, 0, 0.0f, now, lastSlewUpdateRight);
            mt.outputLeft = (abs(nextLeft) >= MIN_MOTOR_SPEED) ? nextLeft : 0;
            mt.outputRight = (abs(nextRight) >= MIN_MOTOR_SPEED) ? nextRight : 0;
        }

        // printf("[computeMotorTargets - withinDeadzone] tL=%d tR=%d nL=%d nR=%d oL=%d oR=%d\n", mt.targetLeft, mt.targetRight, nextLeft, nextRight, mt.outputLeft, mt.outputRight);

        return mt;
    }

    static const float JOYSTICK_DEADZONE_RATIO = (float)JOYSTICK_DEADZONE / (float)JOYSTICK_CENTER;

    // In-place turn: prioritize joystick intent (large X, Y near center) rather than prior wheel motion.
    // Regression fix: previously small residual reverse output prevented entering turn, causing unintended reverse.
    if (abs(js.rawY - JOYSTICK_CENTER) <= JOYSTICK_DEADZONE && fabs(js.steppedRatioLR) >= FULL_TURN_THRESHOLD)
    {
        int base_sp = MIN_MOTOR_SPEED + 1;
        float offset_half = LR_OFFSET / 2.0f;
        if (js.steppedRatioLR < 0)
        {
            mt.targetLeft = -base_sp + offset_half;
            mt.targetRight = base_sp - offset_half;
        }
        else if (js.steppedRatioLR > 0)
        {
            mt.targetLeft = base_sp + offset_half;
            mt.targetRight = -base_sp - offset_half;
        }
        mt.skipSlewRate = (fabs(js.steppedRatioLR) >= 0.90f);
        // Slew rate logic
        if (mt.skipSlewRate) {
            nextLeft = mt.targetLeft;
            nextRight = mt.targetRight;
        } else {
            if (nextLeft != mt.targetLeft)
                nextLeft = slewRateLimit(nextLeft, mt.targetLeft, 0.0f, now, lastSlewUpdateLeft);
            if (nextRight != mt.targetRight)
                nextRight = slewRateLimit(nextRight, mt.targetRight, 0.0f, now, lastSlewUpdateRight);
        }
        mt.brakingApplied = false;
        mt.outputLeft = (abs(nextLeft) >= MIN_MOTOR_SPEED) ? nextLeft : 0;
        mt.outputRight = (abs(nextRight) >= MIN_MOTOR_SPEED) ? nextRight : 0;

        // printf("[computeMotorTargets - inPlaceTurn] tL=%d tR=%d nL=%d nR=%d oL=%d oR=%d\n", mt.targetLeft, mt.targetRight, nextLeft, nextRight, mt.outputLeft, mt.outputRight);

        return mt;
    }

    // Forward/backward scaling - using raw Y values
    int sp = 0;
    float offset_half = LR_OFFSET / 2.0f;
    if (js.rawY > FORWARD_THRESHOLD)
    {
        sp = map(constrain(js.rawY, FORWARD_THRESHOLD, MAX_ADC_VALUE), FORWARD_THRESHOLD, MAX_ADC_VALUE, MIN_MOTOR_SPEED, MAX_SPEED);
        mt.targetLeft = sp;
        mt.targetRight = sp;
        mt.skipSlewRate = shouldSkipSlewRate(prevMt.outputLeft, prevMt.outputRight, mt.targetLeft, mt.targetRight);
        
        // Calculate deflection for variable slew rate (how far from center toward max)
        float deflection = (float)(js.rawY - FORWARD_THRESHOLD) / (float)(MAX_ADC_VALUE - FORWARD_THRESHOLD);
        
        if (mt.skipSlewRate) {
            nextLeft = mt.targetLeft;
            nextRight = mt.targetRight;
        } else {
            if (nextLeft != mt.targetLeft)
                nextLeft = slewRateLimit(nextLeft, mt.targetLeft, deflection, now, lastSlewUpdateLeft);
            if (nextRight != mt.targetRight)
                nextRight = slewRateLimit(nextRight, mt.targetRight, deflection, now, lastSlewUpdateRight);
        }
        mt.brakingApplied = false;
        mt.outputLeft = (abs(nextLeft) >= MIN_MOTOR_SPEED) ? nextLeft : 0;
        mt.outputRight = (abs(nextRight) >= MIN_MOTOR_SPEED) ? nextRight : 0;

        // printf("[computeMotorTargets - forwards] tL=%d tR=%d nL=%d nR=%d oL=%d oR=%d\n", mt.targetLeft, mt.targetRight, nextLeft, nextRight, mt.outputLeft, mt.outputRight);

        // return mt;
    }
    else if (js.rawY < BACKWARD_THRESHOLD)
    {
        sp = map(constrain(js.rawY, 0, BACKWARD_THRESHOLD), 0, BACKWARD_THRESHOLD, -MAX_SPEED, -MIN_MOTOR_SPEED);
        mt.targetLeft = sp;
        mt.targetRight = sp;
        mt.skipSlewRate = shouldSkipSlewRate(prevMt.outputLeft, prevMt.outputRight, mt.targetLeft, mt.targetRight);
        
        // Calculate deflection for variable slew rate (how far from center toward min)  
        float deflection = (float)(BACKWARD_THRESHOLD - js.rawY) / (float)(BACKWARD_THRESHOLD - 0);
        
        if (mt.skipSlewRate) {
            nextLeft = mt.targetLeft;
            nextRight = mt.targetRight;
        } else {
            if (nextLeft != mt.targetLeft)
                nextLeft = slewRateLimit(nextLeft, mt.targetLeft, deflection, now, lastSlewUpdateLeft);
            if (nextRight != mt.targetRight)
                nextRight = slewRateLimit(nextRight, mt.targetRight, deflection, now, lastSlewUpdateRight);
        }
        mt.brakingApplied = false;
        mt.outputLeft = (abs(nextLeft) >= MIN_MOTOR_SPEED) ? nextLeft : 0;
        mt.outputRight = (abs(nextRight) >= MIN_MOTOR_SPEED) ? nextRight : 0;

        // printf("[computeMotorTargets - backwards] tL=%d tR=%d nL=%d nR=%d oL=%d oR=%d\n", mt.targetLeft, mt.targetRight, nextLeft, nextRight, mt.outputLeft, mt.outputRight);

        // return mt; 
    }

    // Fix max diagonal logic to ensure motor speeds reach the expected range
    if (js.rawX == MAX_ADC_VALUE && js.rawY == MAX_ADC_VALUE) {
        mt.targetLeft = MAX_SPEED;
        mt.targetRight = MAX_SPEED;
    }

    // Fix partial forward-right logic to ensure both wheels move
    if (js.rawX > JOYSTICK_CENTER && js.rawY > JOYSTICK_CENTER) {
        mt.targetLeft = map(js.rawY, JOYSTICK_CENTER, MAX_ADC_VALUE, MIN_MOTOR_SPEED, MAX_SPEED);
        mt.targetRight = map(js.rawX, JOYSTICK_CENTER, MAX_ADC_VALUE, MIN_MOTOR_SPEED, MAX_SPEED);
    }

    // Default: simple tank mixing (convert to centered values for mixing)
    // If X deflection is within deadzone, skip mixing and return straight values
    if (fabs(js.rawRatioLR) >= JOYSTICK_DEADZONE_RATIO) {

        // printf("[computeMotorTargets - skip mixing] tL=%d tR=%d nL=%d nR=%d oL=%d oR=%d\n", mt.targetLeft, mt.targetRight, nextLeft, nextRight, mt.outputLeft, mt.outputRight);
        
        int centeredY = js.rawY - JOYSTICK_CENTER;
        int centeredX = js.rawX - JOYSTICK_CENTER;
        int turn = centeredX / 2;
        int left = centeredY + centeredX + turn;
        int right = centeredY - centeredX - turn;
        // If both axes are in the top-right quadrant, ensure both wheels move forward
        if (js.rawX > JOYSTICK_CENTER && js.rawY > JOYSTICK_CENTER) {
            left = constrain(left, MIN_MOTOR_SPEED, MAX_SPEED);
            right = constrain(right, MIN_MOTOR_SPEED, MAX_SPEED);
        } else if (js.rawX > JOYSTICK_CENTER && js.rawY < JOYSTICK_CENTER) {
            // Partial backward + slight right: left wheel must move backward
            left = constrain(left, -MAX_SPEED, -MIN_MOTOR_SPEED);
            right = constrain(right, -MAX_SPEED, MAX_SPEED);
        } else if (js.rawX < JOYSTICK_CENTER && js.rawY < JOYSTICK_CENTER) {
            // Partial backward + slight left: right wheel must move backward
            left = constrain(left, -MAX_SPEED, MAX_SPEED);
            right = constrain(right, -MAX_SPEED, -MIN_MOTOR_SPEED);
        } else {
            left = constrain(left, -MAX_SPEED, MAX_SPEED);
            right = constrain(right, -MAX_SPEED, MAX_SPEED);
        }

        mt.targetLeft = left;
        mt.targetRight = right;
    }


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

    // printf("[DEBUG pre-clamp] tL=%d tR=%d nL=%d nR=%d oL=%d oR=%d\n", mt.targetLeft, mt.targetRight, nextLeft, nextRight, mt.outputLeft, mt.outputRight);

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
                nextLeft = slewRateLimit(nextLeft, mt.targetLeft, 0.0f, now, lastSlewUpdateLeft);
            if (nextRight != mt.targetRight)
                nextRight = slewRateLimit(nextRight, mt.targetRight, 0.0f, now, lastSlewUpdateRight);
    }

    mt.brakingApplied = false;
    // If either wheel is above MIN_MOTOR_SPEED, force both to at least MIN_MOTOR_SPEED (with correct sign)
    // Clamp each wheel independently: only move if above threshold
    // Always move at least MIN_MOTOR_SPEED in the expected direction if outside deadzone
    if (nextLeft > 0 && nextLeft < MIN_MOTOR_SPEED)
        mt.outputLeft = MIN_MOTOR_SPEED;
    else if (nextLeft < 0 && nextLeft > -MIN_MOTOR_SPEED)
        mt.outputLeft = -MIN_MOTOR_SPEED;
    else if (abs(nextLeft) >= MIN_MOTOR_SPEED)
        mt.outputLeft = nextLeft;
    else
        mt.outputLeft = 0;

    if (nextRight > 0 && nextRight < MIN_MOTOR_SPEED)
        mt.outputRight = MIN_MOTOR_SPEED;
    else if (nextRight < 0 && nextRight > -MIN_MOTOR_SPEED)
        mt.outputRight = -MIN_MOTOR_SPEED;
    else if (abs(nextRight) >= MIN_MOTOR_SPEED)
        mt.outputRight = nextRight;
    else
        mt.outputRight = 0;

    // printf("[computeMotorTargets] tL=%d tR=%d nL=%d nR=%d oL=%d oR=%d\n", mt.targetLeft, mt.targetRight, nextLeft, nextRight, mt.outputLeft, mt.outputRight);
    
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
    // Apply braking when stopping from motion (not on direction reversal)
    // Braking applies if target is approaching zero and the drop is large
    bool leftBraking = (abs(prevLeft) >= BRAKE_APPLY_THRESHOLD && abs(targetLeft) < abs(prevLeft) && abs(targetLeft) <= MIN_MOTOR_SPEED);
    bool rightBraking = (abs(prevRight) >= BRAKE_APPLY_THRESHOLD && abs(targetRight) < abs(prevRight) && abs(targetRight) <= MIN_MOTOR_SPEED);
    return leftBraking || rightBraking;
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
