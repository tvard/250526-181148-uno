#include "../.pio/libdeps/native/Unity/src/unity.h"
#include "helpers.h"
// #include "motor_helpers.cpp"
// #include "other_helpers.cpp"

// Declare external variables for testing
extern int leftSpeed;
extern int rightSpeed;


void setUp(void) {
}

void tearDown(void) {
}

void test_slewRateLimit_basic_behavior(void) {
    // No change needed
    TEST_ASSERT_EQUAL(100, slewRateLimit(100, 100));
    TEST_ASSERT_EQUAL(-100, slewRateLimit(-100, -100));
    TEST_ASSERT_EQUAL(0, slewRateLimit(0, 0));
}

void test_slewRateLimit_ramp_up(void) {
    // Ramp up from lower to higher value
    TEST_ASSERT_EQUAL(200, slewRateLimit(100, 200));
    TEST_ASSERT_EQUAL(50, slewRateLimit(0, 50));
    TEST_ASSERT_EQUAL(100, slewRateLimit(-100, 100));
    TEST_ASSERT_EQUAL(1, slewRateLimit(0, 1));
}

void test_slewRateLimit_ramp_down(void) {
    // Ramp down from higher to lower value
    TEST_ASSERT_EQUAL(100, slewRateLimit(200, 100));
    TEST_ASSERT_EQUAL(-50, slewRateLimit(0, -50));
    TEST_ASSERT_EQUAL(-100, slewRateLimit(100, -100));
    TEST_ASSERT_EQUAL(-1, slewRateLimit(0, -1));
}

void test_slewRateLimit_zero_crossing(void) {
    // Crossing zero from positive to negative and vice versa
    TEST_ASSERT_EQUAL(0, slewRateLimit(100, -100));
    TEST_ASSERT_EQUAL(0, slewRateLimit(-100, 100));
    TEST_ASSERT_EQUAL(0, slewRateLimit(50, -50));
    TEST_ASSERT_EQUAL(0, slewRateLimit(-50, 50));
}

void test_slewRateLimit_large_values(void) {
    // Test with large values
    TEST_ASSERT_EQUAL(32767, slewRateLimit(0, 32767));
    TEST_ASSERT_EQUAL(-32768, slewRateLimit(0, -32768));
    TEST_ASSERT_EQUAL(32767, slewRateLimit(-32768, 32767));
    TEST_ASSERT_EQUAL(-32768, slewRateLimit(32767, -32768));
}

void test_slewRateLimit_no_change(void) {
    // No change if current equals target
    TEST_ASSERT_EQUAL(123, slewRateLimit(123, 123));
    TEST_ASSERT_EQUAL(-456, slewRateLimit(-456, -456));
}

void test_slewRateLimit_small_steps(void) {
    // Small increments and decrements
    TEST_ASSERT_EQUAL(2, slewRateLimit(1, 2));
    TEST_ASSERT_EQUAL(-2, slewRateLimit(-1, -2));
    TEST_ASSERT_EQUAL(0, slewRateLimit(1, 0));
    TEST_ASSERT_EQUAL(0, slewRateLimit(-1, 0));
}

void test_slewRateLimit_min_speed_behavior(void) {
    // Test behavior around MIN_MOTOR_SPEED
    TEST_ASSERT_EQUAL(0, slewRateLimit(0, MIN_MOTOR_SPEED - 1));
    TEST_ASSERT_EQUAL(MIN_MOTOR_SPEED, slewRateLimit(0, MIN_MOTOR_SPEED));
    TEST_ASSERT_EQUAL(MIN_MOTOR_SPEED, slewRateLimit(MIN_MOTOR_SPEED - 1, MIN_MOTOR_SPEED));
    TEST_ASSERT_EQUAL(0, slewRateLimit(MIN_MOTOR_SPEED, 0));
}

void test_slewRateLimit_max_speed_behavior(void) {
    // Test behavior around MAX_SPEED
    TEST_ASSERT_EQUAL(MAX_SPEED, slewRateLimit(0, MAX_SPEED));
    TEST_ASSERT_EQUAL(MAX_SPEED, slewRateLimit(MAX_SPEED - 1, MAX_SPEED));
    TEST_ASSERT_EQUAL(0, slewRateLimit(MAX_SPEED, 0));
    TEST_ASSERT_EQUAL(0, slewRateLimit(-MAX_SPEED, 0));
}

// void test_setMotorSpeeds_basic(void) {
//     // Test setting motor speeds directly
//     setMotorSpeeds(100, 100);
//     TEST_ASSERT_EQUAL(100, leftSpeed);
//     TEST_ASSERT_EQUAL(100, rightSpeed);

//     setMotorSpeeds(-100, -100);
//     TEST_ASSERT_EQUAL(-100, leftSpeed);
//     TEST_ASSERT_EQUAL(-100, rightSpeed);
// }

// void test_setMotorSpeeds_with_offsets(void) {
//     // Test setting motor speeds with offsets
//     setMotorSpeeds(150, 150);
//     TEST_ASSERT_EQUAL(150 + LEFT_OFFSET, leftSpeed);
//     TEST_ASSERT_EQUAL(150 + RIGHT_OFFSET, rightSpeed);

//     setMotorSpeeds(-150, -150);
//     TEST_ASSERT_EQUAL(-150 - LEFT_OFFSET, leftSpeed);
//     TEST_ASSERT_EQUAL(-150 - RIGHT_OFFSET, rightSpeed);
// }

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_slewRateLimit_basic_behavior);
    RUN_TEST(test_slewRateLimit_ramp_up);
    RUN_TEST(test_slewRateLimit_ramp_down);
    RUN_TEST(test_slewRateLimit_zero_crossing);
    RUN_TEST(test_slewRateLimit_large_values);
    RUN_TEST(test_slewRateLimit_no_change);
    RUN_TEST(test_slewRateLimit_small_steps);
    RUN_TEST(test_slewRateLimit_min_speed_behavior);
    RUN_TEST(test_slewRateLimit_max_speed_behavior);
    // RUN_TEST(test_setMotorSpeeds_basic);
    // RUN_TEST(test_setMotorSpeeds_with_offsets);
    return UNITY_END();
}

/* Test Suite Overview */
// - Designed for use with Arduino/PlatformIO and Unity test framework (pio lib). Includes necessary headers for Unity testing and the main source file.
// - Contains multiple test functions for:
//     - slewRateLimit: tests basic, ramp up/down, zero crossing, large values, no change, small steps, min/max speed behaviors.
//     - setMotorSpeeds: tests basic setting and with offsets.
// - Define test setup and teardown functions (or leave empty).
// - Each test uses TEST_ASSERT_EQUAL to check expected vs actual results.
// - main() runs all tests using Unity macros.
// - Comments note that tests are logic-only, not hardware, and should be run in a controlled/unit test environment.
