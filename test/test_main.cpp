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
 */

#include "../.pio/libdeps/native/Unity/src/unity.h"
#include "helpers.h"

// Declare external variables for testing
extern int leftSpeed;
extern int rightSpeed;


void setUp(void) {
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
    TEST_ASSERT_EQUAL_MESSAGE(20 - RAMP_STEP, slewRateLimit(20, -100), "Zero crossing: 20->-100");
    TEST_ASSERT_EQUAL_MESSAGE(-20 + RAMP_STEP, slewRateLimit(-20, 100), "Zero crossing: -20->100");
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
}

void test_processJoystick_center(void) {
    JoystickProcessingResult js = processJoystick(512, 512, false);
    TEST_ASSERT_INT_WITHIN(2, 0, js.correctedX); // Allow small rounding error
    TEST_ASSERT_INT_WITHIN(2, 0, js.correctedY);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, js.rawRatioLR);
    TEST_ASSERT_FALSE(js.buzzerOn);
}

void test_processJoystick_buzzer(void) {
    JoystickProcessingResult js = processJoystick(512, 512, true);
    TEST_ASSERT_TRUE(js.buzzerOn);
}

void test_computeMotorTargets_basic(void) {

    // slight right turn
    JoystickProcessingResult js = processJoystick(512 + 50, 512 - 100, false); // correctedY = -100
    MotorTargets mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.left, "Left motor target should be MIN_MOTOR_SPEED");
    TEST_ASSERT_EQUAL_MESSAGE(0, mt.right, "Right motor target should be 0");

    // slight left turn
    js = processJoystick(512 - 50, 512, false); // correctedY = 0
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_EQUAL_MESSAGE(0, mt.left, "Left motor target should be 0");
    TEST_ASSERT_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.right, "Right motor target should be MIN_MOTOR_SPEED");

    // slight forward movement
    js = processJoystick(512, 512 + 50, false);
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.left, "Left motor target should be >= than MIN_MOTOR_SPEED");
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.right, "Right motor target should be >= than MIN_MOTOR_SPEED");

    // slight reverse movement
    js = processJoystick(512, 512 - 50, false);
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_LESS_THAN_MESSAGE(-MIN_MOTOR_SPEED, mt.left, "Left motor target should be less than -MIN_MOTOR_SPEED");
    TEST_ASSERT_LESS_THAN_MESSAGE(-MIN_MOTOR_SPEED, mt.right, "Right motor target should be less than -MIN_MOTOR_SPEED");

    // sharp right turn - pivot
    js = processJoystick(512 + 500, 512, false); // correctedY
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.left, "Left motor target should be >= than MIN_MOTOR_SPEED");
    TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(-MIN_MOTOR_SPEED, mt.right, "Right motor target should be <= than -MIN_MOTOR_SPEED (reverse)");
    
    // sharp left turn - pivot
    js = processJoystick(512 - 500, 512, false); // correctedY
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_LESS_THAN_MESSAGE(-MIN_MOTOR_SPEED, mt.left, "Left motor target should be less than -MIN_MOTOR_SPEED (reverse)");
    TEST_ASSERT_GREATER_THAN_MESSAGE(MIN_MOTOR_SPEED, mt.right, "Right motor target should be greater than MIN_MOTOR_SPEED");

    // full forward
    js = processJoystick(512 + 500, 512 + 500, false);
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.left, "Left motor target should be >= than MIN_MOTOR_SPEED");
    TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.right, "Right motor target should be >= than MIN_MOTOR_SPEED");

    // full reverse
    js = processJoystick(512 - 500, 512 - 500, false);
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_LESS_THAN_MESSAGE(-MIN_MOTOR_SPEED, mt.left, "Left motor target should be less than -MIN_MOTOR_SPEED");
    TEST_ASSERT_LESS_THAN_MESSAGE(-MIN_MOTOR_SPEED, mt.right, "Right motor target should be less than -MIN_MOTOR_SPEED");
}

void test_computeMotorTargets_skipSlew(void) {
    JoystickProcessingResult js = processJoystick(512, 512, false); // center position

    // slight right turn (should not skip slew rate)
    MotorTargets mt = computeMotorTargets(js, 100, -100);
    TEST_ASSERT_FALSE(mt.skipSlewRate);

    // strong right turn (should skip slew rate)
    js = processJoystick(512 + 400, 512, false); 
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_TRUE(mt.skipSlewRate);

    // strong left turn (should skip slew rate)
    js = processJoystick(512 - 400, 512, false);
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_TRUE(mt.skipSlewRate);

    // forwards (should NOT skip slew rate)
    js = processJoystick(512, 512, false);
    mt = computeMotorTargets(js, 100, 100); // prevLeft > 0, targetLeft < 0
    TEST_ASSERT_FALSE(mt.skipSlewRate);

    // backwards (should NOT skip slew rate)
    js = processJoystick(512, 512, false);
    mt = computeMotorTargets(js, -100, -100); // prevLeft < 0, targetLeft > 0
    TEST_ASSERT_FALSE(mt.skipSlewRate); 

    // direction flip (should skip slew rate)
    js = processJoystick(512, 512 - 400, false);    // going backwards
    mt = computeMotorTargets(js, 100, 100);         // targeting forwards
    TEST_ASSERT_TRUE(mt.skipSlewRate);  
}

void test_computeMotorTargets_deadzone(void) {
    JoystickProcessingResult js = processJoystick(512 + 2, 512 + 2, false); // very small movement
    MotorTargets mt = computeMotorTargets(js, 0, 0);

    TEST_ASSERT_EQUAL(0, mt.left);
    TEST_ASSERT_EQUAL(0, mt.right);
}

void test_computeMotorTargets_edge_of_deadzone(void) {
    // Just inside deadzone
    JoystickProcessingResult js = processJoystick(512 + JOYSTICK_DEADZONE - 1, 512, false);
    MotorTargets mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_EQUAL(0, mt.left);
    TEST_ASSERT_EQUAL(0, mt.right);

    // Just outside deadzone
    js = processJoystick(512 + JOYSTICK_DEADZONE + 1, 512, false);
    mt = computeMotorTargets(js, 0, 0);
    TEST_ASSERT_NOT_EQUAL(0, mt.left);
    TEST_ASSERT_NOT_EQUAL(0, mt.right);
}

// Test full diagonal stick deflection (should mix both axes).
void test_computeMotorTargets_max_diagonal(void) {
    JoystickProcessingResult js = processJoystick(1023, 1023, false);
    MotorTargets mt = computeMotorTargets(js, 0, 0);
    // Depending on mixing, left or right may saturate at MAX_SPEED
    TEST_ASSERT_LESS_OR_EQUAL(MAX_SPEED, mt.left);
    TEST_ASSERT_LESS_OR_EQUAL(MAX_SPEED, mt.right);
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
    RUN_TEST(test_processJoystick_center);
    RUN_TEST(test_processJoystick_buzzer);
    RUN_TEST(test_computeMotorTargets_basic);
    RUN_TEST(test_computeMotorTargets_skipSlew);
    RUN_TEST(test_computeMotorTargets_deadzone);
    RUN_TEST(test_computeMotorTargets_edge_of_deadzone);

    RUN_TEST(test_computeMotorTargets_max_diagonal);
    RUN_TEST(test_shouldApplyBraking_on_stop);
    RUN_TEST(test_shouldApplyBraking_after_brake);
    RUN_TEST(test_slewRateLimit_at_boundaries);

    
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
