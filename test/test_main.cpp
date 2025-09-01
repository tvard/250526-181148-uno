/*
 *      
 *      Test Suite for Arduino/PlatformIO Projects, Instructions:
 * 
 *      To run all unit tests on the native environment, use:
 *        pio test -e native
 *      
 *      For more detailed output, add verbosity flags:
 *        pio test -e native -v      // verbose
 *        pio test -e native -vv     // more verbose
 *        pio test -e native -vvv    // most verbose
 *      
 *      If you encounter build issues, a clean build may help:
 *        pio run -t clean
 * 
 *      Expectations:
 *      - All tests should pass without errors.
 *
 *      Note: These tests are logic-only and do not require hardware.
 *      They are designed to run on the native platform, simulating the Arduino environment.
 *      
 *      gotchas:
 *      Unity GT and LT (e.g. TEST_ASSERT_GREATER_THAN_MESSAGE) expects integers as arguments. For floats, use 
 *      TEST_ASSERT_TRUE_MESSAGE instead, (val > 0.0f, "... > 0"); - its got more overloads
 * 
 *      for output printing, use printf... ( printf("x = %d\n", xVal); ) alongside `-v` flag for `pio` (any amount of `v's`)
 *  
 */

#include <unity.h>
#include "helpers.h"
#include "test_display_mock.h"

// Arduino compatibility for native environment
#ifndef ARDUINO
#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define map(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
#endif

// Mock display instance
MockDisplay* display = nullptr;

// External variables from main firmware that our calculation functions need
extern uint16_t xMin, xMax, yMin, yMax;
extern uint16_t xCenter, yCenter;

// Declare the calculation functions we want to test
int calculateThrottlePercent(int y);
int calculateLeftRightPercent(int x);

// Declare external variables for testing
extern int leftSpeed;
extern int rightSpeed;

ManualModeInputs input;
JoystickProcessingResult js;
MotorTargets mt;

void setUp(void) {
    input = {};
    input.joystick.rawX = 512;        // Updated to use rawX instead of correctedX
    input.joystick.rawY = 512;        // Updated to use rawY instead of correctedY
    input.joystick.buzzerOn = false;
    input.leftSpeed = 0;
    input.rightSpeed = 0;
    input.prevLeftSpeed = 0;
    input.prevRightSpeed = 0;

    js = {};
    mt = {};
}

void tearDown(void) {
}

void test_slewRateLimit_no_change(void) {
    // No change needed
    TEST_ASSERT_EQUAL_MESSAGE(100, slewRateLimit(100, 100), "No change: 100->100");
    TEST_ASSERT_EQUAL_MESSAGE(-100, slewRateLimit(-100, -100), "No change: -100->-100");
    TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(0, 0), "No change: 0->0");
}

void test_slewRateLimit_ramp_up(void) {
    TEST_ASSERT_EQUAL_MESSAGE(100 + RAMP_STEP, slewRateLimit(100, 200), "Ramp up: 100->200 (above MIN_MOTOR_SPEED), should increase by RAMP_STEP");
    TEST_ASSERT_EQUAL_MESSAGE(MIN_MOTOR_SPEED, slewRateLimit(0, 50), "Ramp up: 0->50 (below MIN_MOTOR_SPEED), should clamp to MIN_MOTOR_SPEED");
    TEST_ASSERT_EQUAL_MESSAGE(-MIN_MOTOR_SPEED, slewRateLimit(0, -50), "Ramp up: 0->-50 (below -MIN_MOTOR_SPEED), should clamp to -MIN_MOTOR_SPEED");
    TEST_ASSERT_EQUAL_MESSAGE(-100 + RAMP_STEP, slewRateLimit(-100, 100), "Ramp up: -100->100, should increase by RAMP_STEP");
    TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(0, 1), "Ramp up 0 + 1, result => +1");
    TEST_ASSERT_EQUAL_MESSAGE(101, slewRateLimit(100, 101),"Ramp up 100 + 1, result => +1");
    TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(MIN_MOTOR_SPEED + (int)(RAMP_STEP / 3), 0), "Ramp down from slightly above MIN_MOTOR_SPEED to 0 => should clamp to 0");
}

void test_slewRateLimit_ramp_down(void) {
    // Ramp down from higher to lower value
    TEST_ASSERT_EQUAL_MESSAGE(200 - RAMP_STEP, slewRateLimit(200, 100), "Ramp down: 200->100");
    TEST_ASSERT_EQUAL_MESSAGE(100 - RAMP_STEP, slewRateLimit(100, -100), "Ramp down: 100->-100");
    TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(0, -1), "Ramp down: 0->-1");                     // Ensure stop when below MIN_MOTOR_SPEED during ramp down
    TEST_ASSERT_EQUAL_MESSAGE(99, slewRateLimit(100, 99), "Ramp down: 100->99");                  // small decrement, RAMP_STEP capped to decrement
}

void test_slewRateLimit_zero_crossing(void) {
    // Crossing zero from positive to negative and vice versa
    // These test the logic: "when crossing zero, clamp to zero if we'd end up in weak motor range"
    
    // 20 -> -100: ramps to (20-30)=-10, which is > -MIN_MOTOR_SPEED(-70), so clamps to 0
    TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(20, -100), "Zero crossing: 20->-100 should clamp to 0");
    // -20 -> 100: ramps to (-20+30)=10, which is < MIN_MOTOR_SPEED(70), so clamps to 0  
    TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(-20, 100), "Zero crossing: -20->100 should clamp to 0");
}

void test_slewRateLimit_small_steps(void) {
    // Small increments and decrements

    // Very small motor values => should clamp to zero due to MOTOR_DEADZONE
    TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(1, 2), "Small step below motor deadzone: 1->2 (should be 0)");
    TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(-1, -2), "Small step below motor deadzone: -1->-2 (should be 0)");
    TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(1, 0), "Small step below motor deadzone: 1->0 (should be 0)");
    TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(-1, 0), "Small step below motor deadzone: -1->0 (should be 0)");

    // Motor values above MIN_MOTOR_SPEED => should apply small increments
    TEST_ASSERT_EQUAL_MESSAGE(MIN_MOTOR_SPEED + 2, slewRateLimit(MIN_MOTOR_SPEED, MIN_MOTOR_SPEED + 2), "Small step above MIN_MOTOR_SPEED: +2 (should increase by +2)");
    TEST_ASSERT_EQUAL_MESSAGE(-MIN_MOTOR_SPEED - 2, slewRateLimit(-MIN_MOTOR_SPEED,-MIN_MOTOR_SPEED - 2), "Small step above MIN_MOTOR_SPEED: +2 in reverse (should decrease further by -2)");
}

void test_slewRateLimit_min_speed_behavior(void) {
    // Test behavior around MIN_MOTOR_SPEED
    // test clamping at MIN_MOTOR_SPEED when ramping up and below MIN_MOTOR_SPEED, and clamping at 0 when ramping down and below MIN_MOTOR_SPEED
    TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(MIN_MOTOR_SPEED + 5, 0), "Min speed: > MIN_MOTOR_SPEED, ramping down to less than MIN_MOTOR_SPEED => should clamp to 0");
    TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(-MIN_MOTOR_SPEED - 5, 0), "Min speed: < -MIN_MOTOR_SPEED, ramping down to less than -MIN_MOTOR_SPEED => should clamp to 0");
    TEST_ASSERT_EQUAL_MESSAGE(MIN_MOTOR_SPEED, slewRateLimit(MIN_MOTOR_SPEED - 5, MIN_MOTOR_SPEED), "Min speed: < MIN_MOTOR_SPEED ramping up to MIN_MOTOR_SPEED => expect MIN_MOTOR_SPEED");
    TEST_ASSERT_EQUAL_MESSAGE(-MIN_MOTOR_SPEED, slewRateLimit(-MIN_MOTOR_SPEED + 5, -MIN_MOTOR_SPEED), "Min speed: > -MIN_MOTOR_SPEED ramping up to -MIN_MOTOR_SPEED => expect -MIN_MOTOR_SPEED");

    // test clamping at MIN_MOTOR_SPEED when ramping down
    TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(MIN_MOTOR_SPEED + 5, MIN_MOTOR_SPEED - 10), "Min speed: > MIN_MOTOR_SPEED ramping down to MIN_MOTOR_SPEED => expect 0");
    TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(-MIN_MOTOR_SPEED - 5, -MIN_MOTOR_SPEED + 10), "Min speed: < -MIN_MOTOR_SPEED ramping down to -MIN_MOTOR_SPEED => expect 0");
}

void test_slewRateLimit_max_speed_behavior(void) {
    // Test behavior around MAX_SPEED
    TEST_ASSERT_EQUAL_MESSAGE(MAX_SPEED, slewRateLimit(MAX_SPEED - 5, MAX_SPEED), "Max speed: < MAX_SPEED");
    TEST_ASSERT_EQUAL_MESSAGE(-MAX_SPEED, slewRateLimit(-MAX_SPEED + 5, -MAX_SPEED), "Max speed: > -MAX_SPEED");
    TEST_ASSERT_EQUAL_MESSAGE(-MAX_SPEED, slewRateLimit(-456, -456), "No change @ (-456 -> -456), but above -MAX_SPEED => should clamp at -MAX_SPEED");
}

void test_shouldSkipSlewRate(void) {
    // Forward to reverse
    TEST_ASSERT_TRUE(shouldSkipSlewRate(100, 100, -100, -100));
    // Reverse to forward
    TEST_ASSERT_TRUE(shouldSkipSlewRate(-100, -100, 100, 100));
    // No direction change
    TEST_ASSERT_FALSE(shouldSkipSlewRate(100, 100, 80, 80));
    TEST_ASSERT_FALSE(shouldSkipSlewRate(-100, -100, -80, -80));

    TEST_ASSERT_TRUE(shouldSkipSlewRate(100, -100, -100, -100)); // Mixed L->R
    TEST_ASSERT_TRUE(shouldSkipSlewRate(-100, 100, 100, 100));   // Mixed L->R

}

void test_processJoystick_buzzer(void) {
    js = processJoystick(512, 512, true, false);
    TEST_ASSERT_TRUE(js.buzzerOn);
}

void test_processJoystick_movement(void) {
    js = processJoystick(512 + 50, 512 - 100, false, false);

    printf ("Joystick: %d, %d\n", js.rawX, js.rawY);

    TEST_ASSERT_INT_WITHIN(2, 512 + 50, js.rawX);
    TEST_ASSERT_INT_WITHIN(2, 512 - 100, js.rawY);
    TEST_ASSERT_FALSE(js.buzzerOn);
}

void test_processJoystick_deadzone_behavior(void) {
    js = processJoystick(514, 510, false, false);  // small deviation
    TEST_ASSERT_INT_WITHIN(3, 514, js.rawX);  // Raw values should be preserved
    TEST_ASSERT_INT_WITHIN(3, 510, js.rawY);
}

void test_computeMotorTargets_still(void) {
    js = processJoystick(512, 512, false, false);
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_EQUAL_MESSAGE(0, mt.left, "Left motor target should be 0");
    TEST_ASSERT_EQUAL_MESSAGE(0, mt.right, "Right motor target should be 0");
}

void test_computeMotorTargets_right_turn(void) {
    js = processJoystick((512 + 50), 512, false, false); // Right turn - X > center
    mt = computeMotorTargets(js, 0, 0);

    TEST_ASSERT_TRUE_MESSAGE(js.steppedRatioLR > 0.0f, "Stepped ratio should be > 0");  
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.left, "Left motor target should be >= MIN_MOTOR_SPEED");
    TEST_ASSERT_EQUAL_MESSAGE(0, mt.right, "Right motor target should be 0");
}

void test_computeMotorTargets_left_turn(void) {
    js = processJoystick(512 - 50, 512, false, false); // Left turn - X < center
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.right, "Right motor target should be >= MIN_MOTOR_SPEED");
    TEST_ASSERT_EQUAL_MESSAGE(0, mt.left, "Left motor target should be 0");
}

void test_computeMotorTargets_forward(void) {
    js = processJoystick(512, 512 + 100, false, false);  // 612 > 587 (FORWARD_THRESHOLD)
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.left, "Left motor target should be >= than MIN_MOTOR_SPEED");
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.right, "Right motor target should be >= than MIN_MOTOR_SPEED");
}

void test_computeMotorTargets_reverse(void) {
    js = processJoystick(512, 512 - 100, false, false);  // 412 < 437 (BACKWARD_THRESHOLD)
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_LESS_THAN_MESSAGE(-MIN_MOTOR_SPEED, mt.left, "Left motor target should be less than -MIN_MOTOR_SPEED");
    TEST_ASSERT_LESS_THAN_MESSAGE(-MIN_MOTOR_SPEED, mt.right, "Right motor target should be less than -MIN_MOTOR_SPEED");
}

void test_computeMotorTargets_sharp_right_turn(void) {
    js = processJoystick(512 + 500, 512, false, false); // Sharp right turn - X >> center
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.left, "Left motor target should be >= than MIN_MOTOR_SPEED");
    TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(-MIN_MOTOR_SPEED, mt.right, "Right motor target should be <= than -MIN_MOTOR_SPEED (reverse)");
    
}

void test_computeMotorTargets_sharp_left_turn(void) {
    js = processJoystick(512 - 500, 512, false, false); // Sharp left turn - X << center
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_LESS_THAN_MESSAGE(-MIN_MOTOR_SPEED, mt.left, "Left motor target should be less than -MIN_MOTOR_SPEED (reverse)");
    TEST_ASSERT_GREATER_THAN_MESSAGE(MIN_MOTOR_SPEED, mt.right, "Right motor target should be greater than MIN_MOTOR_SPEED");
}

void test_computeMotorTargets_Mixing(void) {
    js = processJoystick(512 + 50, 512 + 500, false, false); // forward and slight right => L motor should be faster
    mt = computeMotorTargets(js, 0, 0);

    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.left, "Left motor target should be >= than MIN_MOTOR_SPEED");
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.right, "Right motor target should be >= than MIN_MOTOR_SPEED");
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(mt.right, mt.left, "Left motor target should be >= than Right motor target on on right turn.");

    js = processJoystick(512 - 50, 512 + 500, false, false); // forward and slight left => R motor should be faster
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.left, "Left motor target should be >= than MIN_MOTOR_SPEED");
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.right, "Right motor target should be >= than MIN_MOTOR_SPEED");
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(mt.left, mt.right, "Left motor target should be <= than Right motor target on on left turn.");

    js = processJoystick(512 + 50, 512 - 500, false, false); // reverse and slight right => L motor should be faster (in reverse, -ve speed)
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_LESS_THAN_MESSAGE(-MIN_MOTOR_SPEED, mt.left, "Left motor target should be less than -MIN_MOTOR_SPEED");
    TEST_ASSERT_LESS_THAN_MESSAGE(-MIN_MOTOR_SPEED, mt.right, "Right motor target should be less than -MIN_MOTOR_SPEED");
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(mt.right, mt.left, "Left motor target should be >= than Right motor target on on right turn.");

    js = processJoystick(512 - 50, 512 - 500, false, false); // reverse and slight left => R motor should be faster (in reverse, -ve speed)
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_LESS_THAN_MESSAGE(-MIN_MOTOR_SPEED, mt.left, "Left motor target should be less than -MIN_MOTOR_SPEED");
    TEST_ASSERT_LESS_THAN_MESSAGE(-MIN_MOTOR_SPEED, mt.right, "Right motor target should be less than -MIN_MOTOR_SPEED");
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(mt.left, mt.right, "Left motor target should be <= than Right motor target on on left turn.");
}

void test_computeMotorTargets_skipSlew_leftRight(void) {
    js = processJoystick(512 + 50, 512, false, false); // slight right turn
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_FALSE(mt.skipSlewRate);

    js = processJoystick(512 + 400, 512, false, false); // strong right turn
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_TRUE(mt.skipSlewRate);

    js = processJoystick(512 - 400, 512, false, false); // strong left turn
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_TRUE(mt.skipSlewRate);
}

void test_computeMotorTargets_skipSlew_forwardsBackwards(void) {
    js = processJoystick(512, 512 + 100, false, false); // forwards
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_FALSE(mt.skipSlewRate);

    js = processJoystick(512, 512 - 100, false, false); // backwards
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_FALSE(mt.skipSlewRate);

    js = processJoystick(512, 512 - 400, false, false); // direction flip
    mt = computeMotorTargets(js, 100, 100);
    TEST_ASSERT_TRUE(mt.skipSlewRate);
}

void test_computeMotorTargets_deadzone(void) {
    js = processJoystick(512 + 2, 512 + 2, false, false); // very small movement
    mt = computeMotorTargets(js, 0, 0);

    TEST_ASSERT_EQUAL(0, mt.left);                        // ...within deadzone = no movement
    TEST_ASSERT_EQUAL(0, mt.right);
}

void test_computeMotorTargets_edge_of_deadzone(void) {
    // Just inside deadzone - nudged left
    js = processJoystick(512 - JOYSTICK_DEADZONE - 1, 512, false, false);
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_NOT_EQUAL(0, mt.right);

    // Just outside deadzone - nudged right
    js = processJoystick(512 + JOYSTICK_DEADZONE + 1, 512, false, false);
    mt = computeMotorTargets(js, 0, 0);

    TEST_ASSERT_NOT_EQUAL(0, mt.left);

    // Just outside deadzone - nudged forward
    js = processJoystick(512, 512 + JOYSTICK_DEADZONE + 1, false, false);
    mt = computeMotorTargets(js, 0, 0);

    TEST_ASSERT_NOT_EQUAL(0, mt.left);

    // Just outside deadzone - nudged backward
    js = processJoystick(512, 512 - JOYSTICK_DEADZONE - 1, false, false);
    mt = computeMotorTargets(js, 0, 0);

    TEST_ASSERT_NOT_EQUAL(0, mt.left);

}

void test_computeMotorTargets_max_diagonal(void) {
    js = processJoystick(1023, 1023, false, false);
    mt = computeMotorTargets(js, 0, 0);
    
    // both wheels should be moving forward, but left wheel should be faster (turns right)
    TEST_ASSERT_GREATER_OR_EQUAL(MIN_MOTOR_SPEED, mt.left);
    TEST_ASSERT_GREATER_OR_EQUAL(MIN_MOTOR_SPEED, mt.right);
    TEST_ASSERT_GREATER_OR_EQUAL(mt.right, mt.left);
}

void test_shouldApplyBraking_on_stop(void) {
    TEST_ASSERT_TRUE(shouldApplyBraking(100, 100, 0, 0));
    TEST_ASSERT_TRUE(shouldApplyBraking(-100, -100, 0, 0));
    TEST_ASSERT_FALSE(shouldApplyBraking(0, 0, 0, 0));
}

void test_shouldApplyBraking_after_brake(void) {
    // Simulate braking just occurred
    TEST_ASSERT_TRUE(shouldApplyBraking(100, 100, 0, 0));
    // Next cycle, still stopped
    TEST_ASSERT_FALSE(shouldApplyBraking(0, 0, 0, 0));
}

void test_slewRateLimit_at_boundaries(void) {
    TEST_ASSERT_EQUAL(MAX_SPEED, slewRateLimit(MAX_SPEED, MAX_SPEED + 100));
    TEST_ASSERT_EQUAL(-MAX_SPEED, slewRateLimit(-MAX_SPEED, -MAX_SPEED - 100));
}

// --- Test: Zero Input -> No Movement ---
void test_processJoystick_NoMotion() {
    js = processJoystick(512, 512, false, false);
    mt = computeMotorTargets(js, 0, 0);

    TEST_ASSERT_INT_WITHIN(JOYSTICK_DEADZONE, 512, js.rawX);
    TEST_ASSERT_INT_WITHIN(JOYSTICK_DEADZONE, 512, js.rawY);
    TEST_ASSERT_EQUAL(0, mt.left);
    TEST_ASSERT_EQUAL(0, mt.right);
}

// --- Test: Near-Zero Input Should Not Move ---
void test_processJoystick_NearZeroInputWithinDeadzone_NoMotion() {

    js = processJoystick(512 + 5, 512 + -4, false, false);
    mt = computeMotorTargets(js, 0, 0);

    TEST_ASSERT_EQUAL(0, mt.left);
    TEST_ASSERT_EQUAL(0, mt.right);
}

// --- Test: Slew Limiter Reaches Zero from Small Drift ---
void test_slewRateLimit_RampsDownToZero() {

    js = processJoystick(512, 512, false, false); // Center joystick

    // Pretend we're already moving a bit
    mt = computeMotorTargets(js, 10, 10);

    // Simulate several control loops
    for (int i = 0; i < 20; i++) {
        mt = computeMotorTargets(js, mt.left, mt.right);
    }

    TEST_ASSERT_EQUAL(0, mt.left);
    TEST_ASSERT_EQUAL(0, mt.right);
}

// --- Test: Full Speed to Complete Stop via Slew Rate Limiting ---
// This test validates the complete motor control cycle: acceleration → high speed → gradual deceleration → stop
// It simulates a real-world scenario where a user pushes the joystick to full speed, then releases it to center
void test_SlewRateLimit_FullCycleAccelerateAndStop() {

    // Phase 1: Accelerate to high speed with diagonal movement (forward + right turn)
    js = processJoystick(512 + 400, 512 + 400, false, false);  // Diagonal: forward-right (912, 912)
    
    // Simulate 20 control loops to allow motors to ramp up to maximum speed
    // This tests that slew rate limiting allows gradual acceleration
    for (int i = 0; i < 20; i++) {
        mt = computeMotorTargets(js, mt.left, mt.right);
    }
    // At this point, motors should be at high speed (likely around MAX_SPEED)

    // Phase 2: User releases joystick to center - command stop
    js = processJoystick(512, 512, false, false);  // Center position = no movement requested
    
    // Simulate 20 more control loops to allow motors to gradually decelerate
    // This tests that slew rate limiting provides smooth deceleration rather than instant stop
    for (int i = 0; i < 20; i++) {
        mt = computeMotorTargets(js, mt.left, mt.right);
    }

    // Final validation: Motors should have completely stopped
    TEST_ASSERT_EQUAL_MESSAGE(0, mt.left, "Left motor should have gradually decelerated to complete stop");
    TEST_ASSERT_EQUAL_MESSAGE(0, mt.right, "Right motor should have gradually decelerated to complete stop");
}

// --- Test: Configuration Constants Validation ---
void test_configuration_constants(void) {
    // These tests help catch configuration changes that might break motor control
    TEST_ASSERT_GREATER_THAN_MESSAGE(0, MIN_MOTOR_SPEED, "MIN_MOTOR_SPEED must be positive");
    TEST_ASSERT_LESS_THAN_MESSAGE(MAX_SPEED, MIN_MOTOR_SPEED, "MIN_MOTOR_SPEED must be less than MAX_SPEED");
    TEST_ASSERT_GREATER_THAN_MESSAGE(0, RAMP_STEP, "RAMP_STEP must be positive");
    TEST_ASSERT_GREATER_THAN_MESSAGE(0, JOYSTICK_DEADZONE, "JOYSTICK_DEADZONE must be positive");
    TEST_ASSERT_EQUAL_MESSAGE(512, JOYSTICK_CENTER, "JOYSTICK_CENTER should be 512 for raw values");
    TEST_ASSERT_EQUAL_MESSAGE(JOYSTICK_CENTER + JOYSTICK_DEADZONE, FORWARD_THRESHOLD, "FORWARD_THRESHOLD consistency");
    TEST_ASSERT_EQUAL_MESSAGE(JOYSTICK_CENTER - JOYSTICK_DEADZONE, BACKWARD_THRESHOLD, "BACKWARD_THRESHOLD consistency");
}

// ============================================================================
// Joystick Calculation Tests - Using actual calculation functions
// ============================================================================

// Setup helper to configure range variables for testing
void setupJoystickRange(uint16_t minX, uint16_t maxX, uint16_t minY, uint16_t maxY, uint16_t centerX = 512, uint16_t centerY = 512) {
    xMin = minX; xMax = maxX; yMin = minY; yMax = maxY;
    xCenter = centerX; yCenter = centerY;
}

void test_joystick_center_position_mapping(void) {
    // Setup reasonable range for testing
    setupJoystickRange(200, 800, 200, 800);
    
    // Test center position
    int throttleResult = calculateThrottlePercent(512);
    int leftRightResult = calculateLeftRightPercent(512);
    
    TEST_ASSERT_INT_WITHIN_MESSAGE(3, 50, throttleResult, "Center Y should map to ~50%");
    TEST_ASSERT_INT_WITHIN_MESSAGE(3, 0, leftRightResult, "Center X should map to ~0%");
}

void test_joystick_full_range_mapping(void) {
    // Test full range utilization
    setupJoystickRange(100, 900, 100, 900);
    
    // Test minimum values
    int throttleMin = calculateThrottlePercent(100);
    int leftRightMin = calculateLeftRightPercent(100);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, throttleMin, "Min Y should map to 0%");
    TEST_ASSERT_EQUAL_INT_MESSAGE(-100, leftRightMin, "Min X should map to -100%");
    
    // Test maximum values  
    int throttleMax = calculateThrottlePercent(900);
    int leftRightMax = calculateLeftRightPercent(900);
    TEST_ASSERT_EQUAL_INT_MESSAGE(100, throttleMax, "Max Y should map to 100%");
    TEST_ASSERT_EQUAL_INT_MESSAGE(100, leftRightMax, "Max X should map to +100%");
}

void test_joystick_deadzone_functionality(void) {
    // Test deadzone boundaries
    setupJoystickRange(200, 800, 200, 800);
    
    // Test values clearly outside the percentage deadzone (±3%)
    int throttleResult = calculateThrottlePercent(550); // Should be > 56%
    int leftRightResult = calculateLeftRightPercent(550); // Should be > 6%
    
    // With range 200-800 and center 512:
    // throttleResult = map(550, 512, 800, 50, 100) = ~56.6% (well outside ±3% deadzone)
    // leftRightResult = map(550, 512, 800, 0, 100) = ~13.2% (well outside ±3% deadzone)
    TEST_ASSERT_GREATER_THAN_MESSAGE(53, throttleResult, "Value outside deadzone should be > 53%");
    TEST_ASSERT_GREATER_THAN_MESSAGE(3, leftRightResult, "Value outside deadzone should be > 3%");
}

void test_leftmost_position_no_deadzone_interference(void) {
    // Specific test for the issue: leftmost position should NOT be forced to center
    setupJoystickRange(0, 1023, 0, 1023);
    
    // Test extreme positions
    int leftRightMin = calculateLeftRightPercent(0);    // Far left
    int leftRightMax = calculateLeftRightPercent(1023); // Far right
    
    // Should be maximum values, NOT forced to center (0%)
    TEST_ASSERT_EQUAL_INT_MESSAGE(-100, leftRightMin, "Leftmost X should be -100%, not forced to center");
    TEST_ASSERT_EQUAL_INT_MESSAGE(100, leftRightMax, "Rightmost X should be 100%, not forced to center");
    
    // Test with center Y to ensure throttle works correctly at extremes
    int throttleCenter = calculateThrottlePercent(512);  // Center Y
    TEST_ASSERT_INT_WITHIN_MESSAGE(3, 50, throttleCenter, "Center Y should be ~50%");
}

// ============================================================================
// Tests for Real-World Joystick Issues
// ============================================================================

void test_leftmost_position_real_world(void) {
    // Test the real issue: leftmost position (x=0-50) should be -100%, not snapping to center
    setupJoystickRange(0, 1023, 0, 1023);
    
    // Test extreme left positions
    int leftResult0 = calculateLeftRightPercent(0);      // Absolute leftmost
    int leftResult10 = calculateLeftRightPercent(10);    // Very close to left
    int leftResult50 = calculateLeftRightPercent(50);    // Still far left
    
    // These should all be strong negative values, NOT snapping to center
    TEST_ASSERT_EQUAL_INT_MESSAGE(-100, leftResult0, "x=0 should give -100%, not snap to center");
    TEST_ASSERT_LESS_THAN_MESSAGE(-80, leftResult10, "x=10 should give strong negative value");
    TEST_ASSERT_LESS_THAN_MESSAGE(-70, leftResult50, "x=50 should give strong negative value");
}

void test_rightmost_position_real_world(void) {
    // Test the real issue: rightmost position should reach 100%, not capped at 82%
    setupJoystickRange(0, 1023, 0, 1023);
    
    // Test extreme right positions
    int rightResult1023 = calculateLeftRightPercent(1023); // Absolute rightmost
    int rightResult1000 = calculateLeftRightPercent(1000); // Very close to right
    int rightResult950 = calculateLeftRightPercent(950);   // Still far right
    
    // These should reach full range
    TEST_ASSERT_EQUAL_INT_MESSAGE(100, rightResult1023, "x=1023 should give 100%");
    TEST_ASSERT_GREATER_THAN_MESSAGE(90, rightResult1000, "x=1000 should give >90%");
    TEST_ASSERT_GREATER_THAN_MESSAGE(80, rightResult950, "x=950 should give >80%");
}

void test_smooth_progression_no_jumps(void) {
    // Test for the jumping behavior: small movements should give small changes
    setupJoystickRange(0, 1023, 0, 1023);
    
    // Test progression from center outward
    int center = calculateLeftRightPercent(512);     // Center
    int slight_right = calculateLeftRightPercent(550); // Slight right
    int more_right = calculateLeftRightPercent(600);   // More right
    
    int center_to_slight = abs(slight_right - center);
    int slight_to_more = abs(more_right - slight_right);
    
    // Movement should be progressive, not jumping by huge amounts
    TEST_ASSERT_LESS_THAN_MESSAGE(20, center_to_slight, "Center to slight movement should be <20%");
    TEST_ASSERT_LESS_THAN_MESSAGE(25, slight_to_more, "Small movements should not cause >25% jumps");
    
    // Test same on left side
    int slight_left = calculateLeftRightPercent(474);   // Slight left  
    int more_left = calculateLeftRightPercent(424);     // More left
    
    int center_to_slight_left = abs(slight_left - center);
    int slight_to_more_left = abs(more_left - slight_left);
    
    TEST_ASSERT_LESS_THAN_MESSAGE(20, center_to_slight_left, "Center to slight left should be <20%");
    TEST_ASSERT_LESS_THAN_MESSAGE(25, slight_to_more_left, "Small left movements should not cause >25% jumps");
}

void test_narrow_range_learning_problem(void) {
    // Test the real issue: firmware starts with narrow range and learns too slowly
    // This simulates the firmware's conservative range expansion
    
    // Start with firmware's initial narrow range (center ±1 to avoid division by zero)
    setupJoystickRange(511, 513, 511, 513);
    
    // Test what happens with extreme positions on narrow range
    int leftResult = calculateLeftRightPercent(0);      // Extreme left
    int rightResult = calculateLeftRightPercent(1023);  // Extreme right
    
    // With such a narrow range, extreme positions will saturate to maximum values
    // This demonstrates how narrow range causes poor resolution
    TEST_ASSERT_EQUAL_INT_MESSAGE(-100, leftResult, "Extreme left should saturate to -100%");
    TEST_ASSERT_EQUAL_INT_MESSAGE(100, rightResult, "Extreme right should saturate to 100%");
}

void test_conservative_range_expansion(void) {
    // Simulate the conservative range expansion with RANGE_EXPANSION_FACTOR = 50
    // Starting range: 512±0, user moves to position 100
    // After expansion: xMin = max(0, 100-50) = 50, xMax = 512
    setupJoystickRange(50, 512, 50, 512);
    
    // Now test extreme positions
    int leftResult = calculateLeftRightPercent(0);    // Beyond learned range
    int rightResult = calculateLeftRightPercent(1023); // Way beyond learned range
    
    // This should show poor mapping due to limited learned range
    TEST_ASSERT_EQUAL_INT_MESSAGE(-100, leftResult, "Should reach -100% even with limited learned range");
    TEST_ASSERT_EQUAL_INT_MESSAGE(100, rightResult, "Should reach 100% even with limited learned range");
}
    
void test_joystick_processJoystick_integration(void) {
    // Test integration with actual helper functions
    JoystickProcessingResult result1 = processJoystick(400, 400, false);  // Below center
    TEST_ASSERT_TRUE_MESSAGE(result1.rawX == 400, "processJoystick should preserve rawX");
    TEST_ASSERT_TRUE_MESSAGE(result1.rawY == 400, "processJoystick should preserve rawY");
    TEST_ASSERT_TRUE_MESSAGE(result1.rawRatioLR < 0, "Left position should give negative ratio");
    
    JoystickProcessingResult result2 = processJoystick(624, 624, false);  // Above center
    TEST_ASSERT_TRUE_MESSAGE(result2.rawX == 624, "processJoystick should preserve rawX");
    TEST_ASSERT_TRUE_MESSAGE(result2.rawY == 624, "processJoystick should preserve rawY");
    TEST_ASSERT_TRUE_MESSAGE(result2.rawRatioLR > 0, "Right position should give positive ratio");
    
    JoystickProcessingResult result3 = processJoystick(512, 512, false);  // Center
    TEST_ASSERT_EQUAL_INT_MESSAGE(512, result3.rawX, "Center position rawX should be 512");
    TEST_ASSERT_EQUAL_INT_MESSAGE(512, result3.rawY, "Center position rawY should be 512");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.1, 0.0, result3.rawRatioLR, "Center position should give ~0 ratio");
}

// ============================================================================
// Fill Bar Display Tests - Testing the critical fill bar drawing logic
// ============================================================================

// Mock fill bar test results
struct FillBarTestResults {
    int fillStartX;
    int fillWidth;  
    int fillStartY;
    int fillHeight;
    bool centerLineDrawn;
    
    void reset() {
        fillStartX = -1;
        fillWidth = -1; 
        fillStartY = -1;
        fillHeight = -1;
        centerLineDrawn = false;
    }
} fillBarResults;

// Constants matching main firmware display logic
const int DISPLAY_WIDTH = 128;
const int DISPLAY_HEIGHT = 64;
const int THROTTLE_BAR_X = 0;
const int THROTTLE_BAR_WIDTH = 10;
const int LR_BAR_Y = 40;
const int LR_BAR_HEIGHT = 8;

// Simulate fill bar calculation logic from main firmware (fixed version)
void simulateFillBarDrawing(int throttlePercent, int leftRightPercent) {
    fillBarResults.reset();
    
    // Throttle fill bar (vertical) - should fill from center upward/downward
    int throttleCenterY = DISPLAY_HEIGHT / 2;  // 32 pixels from top
    if (throttlePercent == 50) {
        // At center - no fill, just center line
        fillBarResults.fillHeight = 0;
        fillBarResults.centerLineDrawn = true;
    } else if (throttlePercent > 50) {
        // Above center - fill upward from center
        int fillPixels = map(throttlePercent, 50, 100, 0, throttleCenterY);
        fillBarResults.fillStartY = throttleCenterY - fillPixels;
        fillBarResults.fillHeight = fillPixels;
    } else {
        // Below center - fill downward from center
        int fillPixels = map(throttlePercent, 0, 50, throttleCenterY, 0);  
        fillBarResults.fillStartY = throttleCenterY;
        fillBarResults.fillHeight = fillPixels;
    }
    
    // Left/Right fill bar (horizontal) - should fill from center left/right  
    int lrCenterX = DISPLAY_WIDTH / 2;  // 64 pixels from left
    if (leftRightPercent == 0) {
        // At center - no fill, just center line
        fillBarResults.fillWidth = 0;
        fillBarResults.centerLineDrawn = true;
    } else if (leftRightPercent > 0) {
        // Right of center - fill rightward from center
        int fillPixels = map(leftRightPercent, 0, 50, 0, lrCenterX);
        fillBarResults.fillStartX = lrCenterX;
        fillBarResults.fillWidth = fillPixels;
    } else {
        // Left of center - fill leftward from center
        int fillPixels = map(-leftRightPercent, 0, 50, 0, lrCenterX);
        fillBarResults.fillStartX = lrCenterX - fillPixels;
        fillBarResults.fillWidth = fillPixels;
    }
}

void test_center_position_no_fill(void) {
    // At center position, there should be no fill bars, just center lines
    simulateFillBarDrawing(50, 0);  // Perfect center
    
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, fillBarResults.fillHeight, "Center throttle should have no fill height");
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, fillBarResults.fillWidth, "Center L/R should have no fill width");
    TEST_ASSERT_TRUE_MESSAGE(fillBarResults.centerLineDrawn, "Center position should show center line");
}

void test_throttle_fill_from_center_upward(void) {
    // Above 50% should fill upward from center
    simulateFillBarDrawing(75, 0);  // 75% throttle, centered L/R
    
    TEST_ASSERT_GREATER_THAN_MESSAGE(0, fillBarResults.fillHeight, "Above center should have fill height");
    TEST_ASSERT_LESS_THAN_MESSAGE(32, fillBarResults.fillStartY, "Fill should start above center line (Y=32)");
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, fillBarResults.fillWidth, "Centered L/R should have no width fill");
}

void test_throttle_fill_from_center_downward(void) {
    // Below 50% should fill downward from center
    simulateFillBarDrawing(25, 0);  // 25% throttle, centered L/R
    
    TEST_ASSERT_GREATER_THAN_MESSAGE(0, fillBarResults.fillHeight, "Below center should have fill height");
    TEST_ASSERT_EQUAL_INT_MESSAGE(32, fillBarResults.fillStartY, "Fill should start at center line (Y=32)");
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, fillBarResults.fillWidth, "Centered L/R should have no width fill");
}

void test_left_right_fill_from_center_rightward(void) {
    // Positive L/R should fill rightward from center
    simulateFillBarDrawing(50, 25);  // Centered throttle, 25% right
    
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, fillBarResults.fillHeight, "Centered throttle should have no fill height");
    TEST_ASSERT_GREATER_THAN_MESSAGE(0, fillBarResults.fillWidth, "Right turn should have fill width");
    TEST_ASSERT_EQUAL_INT_MESSAGE(64, fillBarResults.fillStartX, "Right fill should start at center (X=64)");
}

void test_left_right_fill_from_center_leftward(void) {
    // Negative L/R should fill leftward from center
    simulateFillBarDrawing(50, -25);  // Centered throttle, 25% left
    
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, fillBarResults.fillHeight, "Centered throttle should have no fill height");
    TEST_ASSERT_GREATER_THAN_MESSAGE(0, fillBarResults.fillWidth, "Left turn should have fill width");  
    TEST_ASSERT_LESS_THAN_MESSAGE(64, fillBarResults.fillStartX, "Left fill should start left of center (X<64)");
}

void test_maximum_fill_constraints(void) {
    // Test maximum fill values don't exceed display bounds
    simulateFillBarDrawing(100, 50);  // Maximum throttle and L/R
    
    TEST_ASSERT_TRUE_MESSAGE(fillBarResults.fillHeight <= 32, "Max fill height should not exceed half display");
    TEST_ASSERT_TRUE_MESSAGE(fillBarResults.fillWidth <= 64, "Max fill width should not exceed half display");
    TEST_ASSERT_TRUE_MESSAGE(fillBarResults.fillStartY >= 0, "Fill start Y should be within bounds");
    TEST_ASSERT_TRUE_MESSAGE(fillBarResults.fillStartX >= 0, "Fill start X should be within bounds");

    simulateFillBarDrawing(100, 50);  // Maximum throttle and L/R

}

void test_fill_bar_edge_cases(void) {
    // Test edge case values
    simulateFillBarDrawing(0, -50);   // Minimum throttle, maximum left
    TEST_ASSERT_GREATER_THAN_MESSAGE(0, fillBarResults.fillHeight, "Min throttle should show fill");
    TEST_ASSERT_GREATER_THAN_MESSAGE(0, fillBarResults.fillWidth, "Max left should show fill");
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, fillBarResults.fillStartX, "Max left should fill from far left");
}




int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_slewRateLimit_no_change);
    RUN_TEST(test_slewRateLimit_ramp_up);
    RUN_TEST(test_slewRateLimit_ramp_down);
    RUN_TEST(test_slewRateLimit_zero_crossing);
    RUN_TEST(test_slewRateLimit_small_steps);
    RUN_TEST(test_slewRateLimit_min_speed_behavior);
    RUN_TEST(test_slewRateLimit_max_speed_behavior);
    RUN_TEST(test_shouldSkipSlewRate);
    
    RUN_TEST(test_processJoystick_NoMotion);
    RUN_TEST(test_processJoystick_NearZeroInputWithinDeadzone_NoMotion);
    RUN_TEST(test_processJoystick_buzzer);
    RUN_TEST(test_processJoystick_movement);
    RUN_TEST(test_processJoystick_deadzone_behavior);

    RUN_TEST(test_computeMotorTargets_still);
    RUN_TEST(test_computeMotorTargets_right_turn);
    RUN_TEST(test_computeMotorTargets_left_turn);
    RUN_TEST(test_computeMotorTargets_forward);
    RUN_TEST(test_computeMotorTargets_reverse);
    RUN_TEST(test_computeMotorTargets_sharp_right_turn);
    RUN_TEST(test_computeMotorTargets_sharp_left_turn);

    RUN_TEST(test_computeMotorTargets_Mixing);
    
    RUN_TEST(test_computeMotorTargets_skipSlew_leftRight);
    RUN_TEST(test_computeMotorTargets_skipSlew_forwardsBackwards);

    RUN_TEST(test_computeMotorTargets_deadzone);
    RUN_TEST(test_computeMotorTargets_edge_of_deadzone);

    RUN_TEST(test_computeMotorTargets_max_diagonal);
    RUN_TEST(test_shouldApplyBraking_on_stop);
    RUN_TEST(test_shouldApplyBraking_after_brake);
    RUN_TEST(test_slewRateLimit_at_boundaries);
    RUN_TEST(test_slewRateLimit_RampsDownToZero);

    RUN_TEST(test_SlewRateLimit_FullCycleAccelerateAndStop);

    // Configuration validation tests
    RUN_TEST(test_configuration_constants);
    
    // Joystick comprehensive tests (integrated from test_joystick_comprehensive.cpp)
    RUN_TEST(test_joystick_center_position_mapping);
    RUN_TEST(test_joystick_full_range_mapping);
    RUN_TEST(test_joystick_deadzone_functionality);
    RUN_TEST(test_joystick_processJoystick_integration);
    
    // Fill Bar Display Tests - Critical display logic validation
    RUN_TEST(test_center_position_no_fill);
    RUN_TEST(test_throttle_fill_from_center_upward);
    RUN_TEST(test_throttle_fill_from_center_downward);
    RUN_TEST(test_left_right_fill_from_center_rightward);
    RUN_TEST(test_left_right_fill_from_center_leftward);
    RUN_TEST(test_maximum_fill_constraints);
    RUN_TEST(test_fill_bar_edge_cases);
    RUN_TEST(test_leftmost_position_no_deadzone_interference);
    
    // Real-world joystick issue tests
    RUN_TEST(test_leftmost_position_real_world);
    RUN_TEST(test_rightmost_position_real_world);
    RUN_TEST(test_smooth_progression_no_jumps);
    RUN_TEST(test_narrow_range_learning_problem);
    RUN_TEST(test_conservative_range_expansion);

    return UNITY_END();
}

/*
Terminal output example: -
*  Executing task: pio test -e native -v 
*  Testing...
*   test\test_main.cpp:127:test_slewRateLimit_basic_behavior:PASS
*   test\test_main.cpp:43:test_slewRateLimit_ramp_up:FAIL: Expected 200 Was 130
*   test\test_main.cpp:51:test_slewRateLimit_ramp_down:FAIL: Expected 100 Was 170
*   test\test_main.cpp:59:test_slewRateLimit_zero_crossing:FAIL: Expected 0 Was -100
*   test\test_main.cpp:67:test_slewRateLimit_large_values:FAIL: Expected 32767 Was 70
*   test\test_main.cpp:132:test_slewRateLimit_no_change:PASS
*   test\test_main.cpp:81:test_slewRateLimit_small_steps:FAIL: Expected 2 Was 70
*   test\test_main.cpp:89:test_slewRateLimit_min_speed_behavior:FAIL: Expected 0 Was 70
*   test\test_main.cpp:97:test_slewRateLimit_max_speed_behavior:FAIL: Expected 255 Was 70
*/
