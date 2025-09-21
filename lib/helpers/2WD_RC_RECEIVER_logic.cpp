#include "2WD_RC_RECEIVER_logic.h"
#include "2WD_RC_TRANSMITTER_logic.h" // For calculateThrottlePercent and calculateLeftRightPercent
#include <stdlib.h>
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <algorithm>
#include <cmath>
#endif

// Calibration variables required by transmitter logic
// Use default values for receiver-side calculations
int16_t xMin = 0;
int16_t xMax = MAX_ADC_VALUE;
int16_t yMin = 0;
int16_t yMax = MAX_ADC_VALUE;
uint16_t xCenter = JOYSTICK_CENTER;
uint16_t yCenter = JOYSTICK_CENTER;

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
    if (now - lastSlewUpdate < SLEW_UPDATE_INTERVAL)
        return current;
    lastSlewUpdate = now;

    int rampStep = RAMP_STEP_SLOW;
    if (deflection >= FULL_THROTTLE_THRESHOLD || deflection >= FULL_TURN_THRESHOLD || FAST_SLEW_MODE) {
        rampStep = FAST_SLEW_MODE ? RAMP_STEP_FAST_AF : RAMP_STEP_FAST_MEH;
    }

    // Ensure rampStep at least 1
    if (rampStep < 1) rampStep = 1;

    int delta = target - current;
    int next;

    if (delta > rampStep) {
        next = current + rampStep;
    } else if (delta < -rampStep) {
        next = current - rampStep;
    } else {
        next = target;
    }

    // 1. Clamp to 0 if both target and next are in the deadzone
    if (next > -MOTOR_DEADZONE && next < MOTOR_DEADZONE &&
        target > -MOTOR_DEADZONE && target < MOTOR_DEADZONE)
        return 0;

    // 2. Always clamp to MIN_MOTOR_SPEED when ramping up from 0, regardless of step size
    if (current == 0 && target > 0) {
        // start positive ramp at +MIN
        next = MIN_MOTOR_SPEED;
    } else if (current == 0 && target < 0) {
        // start negative ramp at -MIN
        next = -MIN_MOTOR_SPEED;
    }

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
    // Remove instantaneous snap to MAX/ -MAX; allow normal ramping
    if (target == MAX_SPEED && current > 0 && current >= MAX_SPEED - rampStep)
        return MAX_SPEED; // near top, snap
    else if (target == -MAX_SPEED && current < 0 && current <= -MAX_SPEED + rampStep)
        return -MAX_SPEED; // near bottom, snap

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

/// @brief Computes the motor targets using transmitter-calculated percentages for clean differential steering
/// @param js The joystick processing result.
/// @param prevMt The previous MotorTargets struct (for previous output values).
/// @return The computed motor targets.
/// @remarks
/// Uses transmitter's calculateThrottlePercent and calculateLeftRightPercent directly,
/// then applies simple differential steering math for predictable behavior
MotorTargets computeMotorTargets(const JoystickProcessingResult &js, const MotorTargets &prevMt)
{
    MotorTargets mt = {};
    int nextLeft = prevMt.outputLeft;
    int nextRight = prevMt.outputRight;
    static unsigned long lastSlewUpdateLeft = 0;
    static unsigned long lastSlewUpdateRight = 0;
    unsigned long now = millis();

    // Get transmitter's calculated percentages (-100 to +100)
    int throttlePercent = calculateThrottlePercent(js.rawY);
    int lrPercent = calculateLeftRightPercent(js.rawX);
    
    // Handle center/deadzone case first - use raw joystick values for consistency with existing tests
    bool joystickCentered = (abs(js.rawX - JOYSTICK_CENTER) < JOYSTICK_DEADZONE && abs(js.rawY - JOYSTICK_CENTER) < JOYSTICK_DEADZONE);
    if (joystickCentered || (throttlePercent == 0 && lrPercent == 0))
    {
        mt.targetLeft = 0;
        mt.targetRight = 0;
        mt.skipSlewRate = false;
        
        // Dynamic braking logic
        mt.brakingApplied = shouldApplyBraking(prevMt.outputLeft, prevMt.outputRight, 0, 0);
        
        if (mt.brakingApplied)
        {
            int leftBrakeDirection = (prevMt.outputLeft > 0) ? 1 : -1;
            int rightBrakeDirection = (prevMt.outputRight > 0) ? 1 : -1;
            mt.targetLeft = 0;
            mt.targetRight = 0;
            mt.outputLeft = -MIN_MOTOR_SPEED * leftBrakeDirection;
            mt.outputRight = -MIN_MOTOR_SPEED * rightBrakeDirection;
        }
        else
        {
            if (nextLeft != 0)
                nextLeft = slewRateLimit(nextLeft, 0, 0.0f, now, lastSlewUpdateLeft);
            if (nextRight != 0)
                nextRight = slewRateLimit(nextRight, 0, 0.0f, now, lastSlewUpdateRight);
            mt.outputLeft = (abs(nextLeft) >= MIN_MOTOR_SPEED) ? nextLeft : 0;
            mt.outputRight = (abs(nextRight) >= MIN_MOTOR_SPEED) ? nextRight : 0;
        }
        return mt;
    }

    
    // Serial.println("Throttle: " + String(throttlePercent) + "%, LR: " + String(lrPercent) + "%");

    // Pure in-place turn: throttle near zero, high L/R deflection (use raw values for compatibility)
    if ((abs(throttlePercent) < 30 && abs(lrPercent) >= 25))    
    {
        int base_sp = MIN_MOTOR_SPEED + 1;
        if (lrPercent < 0) // Left turn (negative LR)
        {
            mt.targetLeft = -base_sp;
            mt.targetRight = base_sp;
        }
        else // Right turn (positive LR)
        {
            mt.targetLeft = base_sp;
            mt.targetRight = -base_sp;
        }
        mt.skipSlewRate = (abs(lrPercent) >= 90);
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

        return mt;
    }

    // Standard differential steering: base speed ± turn adjustment
    // Handle edge case: if transmitter says 0% but we're outside raw deadzone, use minimal movement
    int baseSpeed = 0;
    if (throttlePercent > 0) {
        baseSpeed = map(throttlePercent, 1, 100, MIN_MOTOR_SPEED, MAX_SPEED);
    } else if (throttlePercent < 0) {
        // Fix: ensure full reverse uses full range
        baseSpeed = map(throttlePercent, -100, -1, -MAX_SPEED, -MIN_MOTOR_SPEED);
        if (throttlePercent == -100) baseSpeed = -MAX_SPEED;
        // Remove any capping at -25
        if (baseSpeed > -MIN_MOTOR_SPEED && baseSpeed < 0) baseSpeed = -MIN_MOTOR_SPEED;
    } else if (throttlePercent == 0 && abs(js.rawY - JOYSTICK_CENTER) > JOYSTICK_DEADZONE) {
        // Edge of deadzone: minimal movement in detected direction
        if (js.rawY > JOYSTICK_CENTER) {
            baseSpeed = MIN_MOTOR_SPEED;
        } else if (js.rawY < JOYSTICK_CENTER) {
            baseSpeed = -MIN_MOTOR_SPEED;
        }
    }

    /**
     * turnAdjustment determines how much one wheel's speed is reduced relative to the other to achieve turning.
     * - For normal steering, turnAdjustment is proportional to both the base speed and the left/right joystick deflection (turnIntensity).
     * - turnScale is adaptive: higher at low speeds for more responsive turning, lower at high speeds for stability.
     * - For edge cases (when lrPercent is zero but rawX is outside the deadzone), a minimal turnAdjustment is applied to ensure some movement.
     * - The calculation ensures smooth transitions between straight driving, gentle arcs, and sharp turns.
     */
    int turnAdjustment = 0;
    if (lrPercent != 0) {
        // Calculate turn intensity (0.0 to 1.0)
        float turnIntensity = abs(lrPercent) / 100.0f;
        // Adaptive scaling for turn adjustment: more responsive at low speed, less at high speed
        float turnScale = TURN_INTENSITY_FACTOR; // Default value; can be tuned or made adaptive
        if (abs(baseSpeed) < 100) {
            turnScale = TURN_INTENSITY_FACTOR + 0.2f; // More responsive at low speed
        } else if (abs(baseSpeed) > 200) {
            turnScale = TURN_INTENSITY_FACTOR - 0.2f; // Less responsive at high speed
        }
        turnAdjustment = (int)(abs(baseSpeed) * turnIntensity * turnScale);
    } else if (lrPercent == 0 && abs(js.rawX - JOYSTICK_CENTER) > JOYSTICK_DEADZONE) {
        // Edge of deadzone turning: minimal turn adjustment for gentle steering
        float rawTurnIntensity = abs(js.rawX - JOYSTICK_CENTER) / (float)JOYSTICK_CENTER;
        turnAdjustment = max((int)(abs(baseSpeed) * rawTurnIntensity * TURN_INTENSITY_FACTOR), MIN_MOTOR_SPEED / 2); // Ensure some movement
    }
    
    // Handle edge cases where we're outside deadzone but transmitter gives 0%
    bool outsideDeadzone = (abs(js.rawX - JOYSTICK_CENTER) > JOYSTICK_DEADZONE || abs(js.rawY - JOYSTICK_CENTER) > JOYSTICK_DEADZONE);
    
    if (outsideDeadzone && baseSpeed == 0 && turnAdjustment == 0) {
        // Force minimal movement for edge of deadzone
        if (abs(js.rawY - JOYSTICK_CENTER) > JOYSTICK_DEADZONE) {
            baseSpeed = (js.rawY > JOYSTICK_CENTER) ? MIN_MOTOR_SPEED : -MIN_MOTOR_SPEED;
        }
        if (abs(js.rawX - JOYSTICK_CENTER) > JOYSTICK_DEADZONE) {
            turnAdjustment = MIN_MOTOR_SPEED / 2;
        }
    }
    
    if (lrPercent < 0 || (lrPercent == 0 && js.rawX < JOYSTICK_CENTER - JOYSTICK_DEADZONE)) { // Left turn (negative LR)
        if (baseSpeed >= 0) {
            mt.targetLeft = baseSpeed - turnAdjustment;
            mt.targetRight = baseSpeed;
            if (baseSpeed > 0 && mt.targetRight <= 0) {
                mt.targetRight = MIN_MOTOR_SPEED;
            }
        } else {
            mt.targetLeft = baseSpeed + turnAdjustment;
            mt.targetRight = baseSpeed;
            if (baseSpeed < 0 && mt.targetRight >= 0) {
                mt.targetRight = -MIN_MOTOR_SPEED;
            }
        }
    } else if (lrPercent > 0 || (lrPercent == 0 && js.rawX > JOYSTICK_CENTER + JOYSTICK_DEADZONE)) { // Right turn (positive LR)
        if (baseSpeed >= 0) {
            mt.targetLeft = baseSpeed;
            mt.targetRight = baseSpeed - turnAdjustment;
            if (baseSpeed > 0 && mt.targetLeft <= 0) {
                mt.targetLeft = MIN_MOTOR_SPEED;
            }
        } else {
            mt.targetLeft = baseSpeed;
            mt.targetRight = baseSpeed + turnAdjustment;
            if (baseSpeed < 0 && mt.targetLeft >= 0) {
                mt.targetLeft = -MIN_MOTOR_SPEED;
            }
        }
    } else { // Straight
        mt.targetLeft = baseSpeed;
        mt.targetRight = baseSpeed;
    }
    
    // Apply LR_OFFSET balance
    float offset_half = LR_OFFSET / 2.0f;
    if ((mt.targetLeft > 0 && mt.targetRight > 0) || (mt.targetLeft < 0 && mt.targetRight < 0)) {
        mt.targetLeft += (mt.targetLeft > 0) ? -offset_half : offset_half;
        mt.targetRight += (mt.targetRight > 0) ? offset_half : -offset_half;
    }

    // Clamp to valid motor ranges
    mt.targetLeft = constrain(mt.targetLeft, -MAX_SPEED, MAX_SPEED);
    mt.targetRight = constrain(mt.targetRight, -MAX_SPEED, MAX_SPEED);

    // Slew rate control
    float deflection = max(abs(throttlePercent), abs(lrPercent)) / 100.0f;
    mt.skipSlewRate = shouldSkipSlewRate(prevMt.outputLeft, prevMt.outputRight, mt.targetLeft, mt.targetRight);
    
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
    
    // Apply minimum speed thresholds
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

bool shouldApplyBraking(int prevLeft, int prevRight, int targetLeft, int targetRight) {
    // Only apply braking if both motors were moving in same direction
    bool sameDirection = (prevLeft > 0 && prevRight > 0) || (prevLeft < 0 && prevRight < 0);
    bool leftBraking = sameDirection && (abs(prevLeft) >= BRAKE_APPLY_THRESHOLD && abs(targetLeft) < abs(prevLeft) && abs(targetLeft) <= MIN_MOTOR_SPEED);
    bool rightBraking = sameDirection && (abs(prevRight) >= BRAKE_APPLY_THRESHOLD && abs(targetRight) < abs(prevRight) && abs(targetRight) <= MIN_MOTOR_SPEED);
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
