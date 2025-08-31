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
#include <test_data.h>

void test_slewRateLimit_no_change(void) {
    TestData testData = loadTestData("test_slewRateLimit_no_change.json");
    for (const auto& testCase : testData.testCases) {
        int actualOutput = slewRateLimit(testCase.input1, testCase.input2);
        TEST_ASSERT_EQUAL_MESSAGE(testCase.expectedOutput, actualOutput, testCase.description);
    }
}

void test_slewRateLimit_ramp_up(void) {
    TestData testData = loadTestData("test_slewRateLimit_ramp_up.json");
    for (const auto& testCase : testData.testCases) {
        int actualOutput = slewRateLimit(testCase.input1, testCase.input2);
        TEST_ASSERT_EQUAL_MESSAGE(testCase.expectedOutput, actualOutput, testCase.description);
    }
}

void test_slewRateLimit_ramp_down(void) {
    TestData testData = loadTestData("test_slewRateLimit_ramp_down.json");
    for (const auto& testCase : testData.testCases) {
        int actualOutput = slewRateLimit(testCase.input1, testCase.input2);
        TEST_ASSERT_EQUAL_MESSAGE(testCase.expectedOutput, actualOutput, testCase.description);
    }
}

void test_slewRateLimit_zero_crossing(void) {
    TestData testData = loadTestData("test_slewRateLimit_zero_crossing.json");
    for (const auto& testCase : testData.testCases) {
        int actualOutput = slewRateLimit(testCase.input1, testCase.input2);
        TEST_ASSERT_EQUAL_MESSAGE(testCase.expectedOutput, actualOutput, testCase.description);
    }
}

void test_slewRateLimit_small_steps(void) {
    TestData testData = loadTestData("test_slewRateLimit_small_steps.json");
    for (const auto& testCase : testData.testCases) {
        int actualOutput = slewRateLimit(testCase.input1, testCase.input2);
        TEST_ASSERT_EQUAL_MESSAGE(testCase.expectedOutput, actualOutput, testCase.description);
    }
}

void test_slewRateLimit_min_speed_behavior(void) {
    TestData testData = loadTestData("test_slewRateLimit_min_speed_behavior.json");
    for (const auto& testCase : testData.testCases) {
        int actualOutput = slewRateLimit(testCase.input1, testCase.input2);
        TEST_ASSERT_EQUAL_MESSAGE(testCase.expectedOutput, actualOutput, testCase.description);
    }
}

void test_slewRateLimit_max_speed_behavior(void) {
    TestData testData = loadTestData("test_slewRateLimit_max_speed_behavior.json");
    for (const auto& testCase : testData.testCases) {
        int actualOutput = slewRateLimit(testCase.input1, testCase.input2);
        TEST_ASSERT_EQUAL_MESSAGE(testCase.expectedOutput, actualOutput, testCase.description);
    }
}

void test_shouldSkipSlewRate(void) {
    TestData testData = loadTestData("test_shouldSkipSlewRate.json");
    for (const auto& testCase : testData.testCases) {
        bool actualOutput = shouldSkipSlewRate(testCase.input1, testCase.input2);
        TEST_ASSERT_EQUAL_MESSAGE(testCase.expectedOutput, actualOutput, testCase.description);
    }
}

void test_processJoystick_NoMotion(void) {
    TestData testData = loadTestData("test_processJoystick_NoMotion.json");
    for (const auto& testCase : testData.testCases) {
        int actualOutput = processJoystick(testCase.input1, testCase.input2);
        TEST_ASSERT_EQUAL_MESSAGE(testCase.expectedOutput, actualOutput, testCase.description);
    }
}

void test_processJoystick_NearZeroInputWithinDeadzone_NoMotion(void) {
    TestData testData = loadTestData("test_processJoystick_NearZeroInputWithinDeadzone_NoMotion.json");
    for (const auto& testCase : testData.testCases) {
        int actualOutput = processJoystick(testCase.input1, testCase.input2);
        TEST_ASSERT_EQUAL_MESSAGE(testCase.expectedOutput, actualOutput, testCase.description);
    }
}

void test_processJoystick_buzzer(void) {
    TestData testData = loadTestData("test_processJoystick_buzzer.json");
    for (const auto& testCase : testData.testCases) {
        int actualOutput = processJoystick(testCase.input1, testCase.input2);
        TEST_ASSERT_EQUAL_MESSAGE(testCase.expectedOutput, actualOutput, testCase.description);
    }
}

void test_processJoystick_movement(void) {
    TestData testData = loadTestData("test_processJoystick_movement.json");
    for (const auto& testCase : testData.testCases) {
        int actualOutput = processJoystick(testCase.input1, testCase.input2);
        TEST_ASSERT_EQUAL_MESSAGE(testCase.expectedOutput, actualOutput, testCase.description);
    }
}

void test_processJoystick_deadzone_behavior(void) {
    TestData testData = loadTestData("test_processJoystick_deadzone_behavior.json");
    for (const auto& testCase : testData.testCases) {
        int actualOutput = processJoystick(testCase.input1, testCase.input2);
        TEST_ASSERT_EQUAL_MESSAGE(testCase.expectedOutput, actualOutput, testCase.description);
    }
}

void test_processJoystick_deadzone_edge_cases(void) {
    TestData testData = loadTestData("test_processJoystick_deadzone_edge_cases.json");
    for (const auto& testCase : testData.testCases) {
        int actualOutput = processJoystick(testCase.input1, testCase.input2);
        TEST_ASSERT_EQUAL_MESSAGE(testCase.expectedOutput, actualOutput, testCase.description);
    }
}

void test_computeMotorTargets_basic(void) {
    TestData testData = loadTestData("test_computeMotorTargets_basic.json");
    for (const auto& testCase : testData.testCases) {
        MotorTarget actualOutput = computeMotorTargets(testCase.input1, testCase.input2);
        TEST_ASSERT_EQUAL_MESSAGE(testCase.expectedOutput, actualOutput, testCase.description);
    }
}

void test_computeMotorTargets_mixing(void) {
    TestData testData = loadTestData("test_computeMotorTargets_mixing.json");
    for (const auto& testCase : testData.testCases) {
        MotorTarget actualOutput = computeMotorTargets(testCase.input1, testCase.input2);
        TEST_ASSERT_EQUAL_MESSAGE(testCase.expectedOutput, actualOutput, testCase.description);
    }
}

void test_computeMotorTargets_skipping_slew_rate(void) {
    TestData testData = loadTestData("test_computeMotorTargets_skipping_slew_rate.json");
    for (const auto& testCase : testData.testCases) {
        MotorTarget actualOutput = computeMotorTargets(testCase.input1, testCase.input2);
        TEST_ASSERT_EQUAL_MESSAGE(testCase.expectedOutput, actualOutput, testCase.description);
    }
}

void test_shouldApplyBraking_on_stop(void) {
    TestData testData = loadTestData("test_shouldApplyBraking_on_stop.json");
    for (const auto& testCase : testData.testCases) {
        bool actualOutput = shouldApplyBraking(testCase.input1, testCase.input2, testCase.input3, testCase.input4);
        TEST_ASSERT_EQUAL_MESSAGE(testCase.expectedOutput, actualOutput, testCase.description);
    }
}