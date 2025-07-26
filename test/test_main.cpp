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

#include "../.pio/libdeps/native/Unity/src/unity.h"
#include "helpers.h"

// Declare external variables for testing
extern int leftSpeed;
extern int rightSpeed;

ManualModeInputs input;
JoystickProcessingResult js;
MotorTargets mt;

void setUp(void) {
    input = {};
    input.joystick.correctedX = 512;
    input.joystick.correctedY = 512;
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
    TEST_ASSERT_EQUAL_MESSAGE(20 - RAMP_STEP > -MIN_MOTOR_SPEED ? 0 : 20 - RAMP_STEP, slewRateLimit(20, -100), "Zero crossing: 20->-100");
    TEST_ASSERT_EQUAL_MESSAGE(-20 + RAMP_STEP < MIN_MOTOR_SPEED ? 0 : -20 + RAMP_STEP, slewRateLimit(-20, 100), "Zero crossing: -20->100");
}

void test_slewRateLimit_small_steps(void) {
    // Small increments and decrements

    // first step => below deadzone => expect 0
    TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(1, 2), "Small step below deadzone: 1->2 (should be 0)");
    TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(-1, -2), "Small step below deadzone: -1->-2 (should be 0)");
    TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(1, 0), "Small step below deadzone: 1->0 (should be 0)");
    TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(-1, 0), "Small step below deadzone: -1->0 (should be 0)");

    // second step => above deadzone => expect RAMP_STEP
    TEST_ASSERT_EQUAL_MESSAGE(MIN_MOTOR_SPEED + 2, slewRateLimit(MIN_MOTOR_SPEED, MIN_MOTOR_SPEED + 2), "Small step above deadzone: +2 (should increase by +2)");
    TEST_ASSERT_EQUAL_MESSAGE(-MIN_MOTOR_SPEED - 2, slewRateLimit(-MIN_MOTOR_SPEED,-MIN_MOTOR_SPEED - 2), "Small step above deadzone: +2 in reverse (should decrease further by -2)");
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

    printf ("Joystick: %d, %d\n", js.correctedX, js.correctedY);

    TEST_ASSERT_INT_WITHIN(2, 512 + 50, js.correctedX);
    TEST_ASSERT_INT_WITHIN(2, 512 - 100, js.correctedY);
    TEST_ASSERT_FALSE(js.buzzerOn);
}

void test_processJoystick_deadzone_behavior(void) {
    js = processJoystick(514, 510, false, false);  // small deviation
    TEST_ASSERT_INT_WITHIN(3, 512, js.correctedX);  // Assuming deadzone absorbs minor noise
    TEST_ASSERT_INT_WITHIN(3, 512, js.correctedY);
}

void test_computeMotorTargets_still(void) {
    js = processJoystick(512, 512, false, false);
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_EQUAL_MESSAGE(0, mt.left, "Left motor target should be 0");
    TEST_ASSERT_EQUAL_MESSAGE(0, mt.right, "Right motor target should be 0");
}

void test_computeMotorTargets_right_turn(void) {
    js = processJoystick((512 + 50), 512, false, false); // correctedY = -100
    mt = computeMotorTargets(js, 0, 0);

    TEST_ASSERT_TRUE_MESSAGE(js.steppedRatioLR > 0.0f, "Stepped ratio should be > 0");  
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.left, "Left motor target should be >= MIN_MOTOR_SPEED");
    TEST_ASSERT_EQUAL_MESSAGE(0, mt.right, "Right motor target should be 0");
}

void test_computeMotorTargets_left_turn(void) {
    js = processJoystick(512 - 50, 512, false, false); // correctedY = 0
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.right, "Right motor target should be >= MIN_MOTOR_SPEED");
    TEST_ASSERT_EQUAL_MESSAGE(0, mt.left, "Left motor target should be 0");
}

void test_computeMotorTargets_forward(void) {
    js = processJoystick(512, 512 + 50, false, false);
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.left, "Left motor target should be >= than MIN_MOTOR_SPEED");
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.right, "Right motor target should be >= than MIN_MOTOR_SPEED");
}

void test_computeMotorTargets_reverse(void) {
    js = processJoystick(512, 512 - 50, false, false);
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_LESS_THAN_MESSAGE(-MIN_MOTOR_SPEED, mt.left, "Left motor target should be less than -MIN_MOTOR_SPEED");
    TEST_ASSERT_LESS_THAN_MESSAGE(-MIN_MOTOR_SPEED, mt.right, "Right motor target should be less than -MIN_MOTOR_SPEED");
}

void test_computeMotorTargets_sharp_right_turn(void) {
    js = processJoystick(512 + 500, 512, false, false); // correctedY
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.left, "Left motor target should be >= than MIN_MOTOR_SPEED");
    TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(-MIN_MOTOR_SPEED, mt.right, "Right motor target should be <= than -MIN_MOTOR_SPEED (reverse)");
    
}

void test_computeMotorTargets_sharp_left_turn(void) {
    js = processJoystick(512 - 500, 512, false, false); // correctedY
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

    TEST_ASSERT_INT_WITHIN(JOYSTICK_DEADZONE, 512, js.correctedX);
    TEST_ASSERT_INT_WITHIN(JOYSTICK_DEADZONE, 512, js.correctedY);
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

// --- Test: Brake Enforced ---
void test_Brake_EnforcesZeroSpeed() {

    js = processJoystick(512 + 400, 512 + 400, false, false);  // Try to go forward

    for (int i = 0; i < 20; i++) {
        mt = computeMotorTargets(js, mt.left, mt.right);
    }

    js = processJoystick(512, 512, false, false);  // Try to go forward

    for (int i = 0; i < 20; i++) {
        mt = computeMotorTargets(js, mt.left, mt.right);
    }

    TEST_ASSERT_EQUAL(0, mt.left);
    TEST_ASSERT_EQUAL(0, mt.right);
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

    RUN_TEST(test_Brake_EnforcesZeroSpeed);

    
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
