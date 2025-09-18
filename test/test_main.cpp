#include <cmath>
#include <cstdio>
#include "unity.h"
#include "../lib/helpers/helpers.h"
#include "../lib/helpers/2WD_RC_RECEIVER_logic.h"
#include "../lib/helpers/display_helpers.h"          // Fillbars
#include "../lib/helpers/2WD_RC_TRANSMITTER_logic.h" // production declarations

// --- Unity compatibility helpers (some Unity builds lack these compare macros)
#ifndef TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE
#define TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(expected, actual, msg) \
  TEST_ASSERT_TRUE_MESSAGE(((actual) >= (expected)), msg)
#endif
#ifndef TEST_ASSERT_LESS_OR_EQUAL_MESSAGE
#define TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(expected, actual, msg) \
  TEST_ASSERT_TRUE_MESSAGE(((actual) <= (expected)), msg)
#endif
#ifndef TEST_ASSERT_LESS_THAN_MESSAGE
#define TEST_ASSERT_LESS_THAN_MESSAGE(expected, actual, msg) \
  TEST_ASSERT_TRUE_MESSAGE(((actual) < (expected)), msg)
#endif
#ifndef TEST_ASSERT_GREATER_THAN_MESSAGE
#define TEST_ASSERT_GREATER_THAN_MESSAGE(expected, actual, msg) \
  TEST_ASSERT_TRUE_MESSAGE(((actual) > (expected)), msg)
#endif

// --- Arduino stubs for native ------------------------------------------------
#ifndef ARDUINO
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
  void println(const char *) {}
  void println(int) {}
  void print(const char *) {}
  void print(int) {}
} Serial;
#endif

// --- Optional ownership of calibration globals -------------------------------
// If your firmware already defines these, compile tests with -DDEFINE_CALIB_GLOBALS=0
#ifndef DEFINE_CALIB_GLOBALS
#define DEFINE_CALIB_GLOBALS 1
#endif
#if DEFINE_CALIB_GLOBALS
int16_t xMin = 0;
int16_t xMax = MAX_ADC_VALUE;
int16_t yMin = 0;
int16_t yMax = MAX_ADC_VALUE;
uint16_t xCenter = JOYSTICK_CENTER;
uint16_t yCenter = JOYSTICK_CENTER;
#endif

// // --- SUT functions used in tests (declared in headers) -----------------------
// extern int calculateThrottlePercent(int yAdc);
// extern int calculateThrottlePercent(int xAdc);

// (Optionally) extern motor speed state, if available in SUT
extern int leftSpeed;
extern int rightSpeed;

// --- Test fixtures -----------------------------------------------------------
static JoystickProcessingResult js{};
static MotorTargets mt{};

void setUp(void)
{
  js = {};
  js.rawX = JOYSTICK_CENTER;
  js.rawY = JOYSTICK_CENTER;
  js.buzzerOn = false;
  mt = {};
}

void tearDown(void) {}

// Test for strict symmetry and offset in in-place turns
void test_inplace_turn_symmetry_with_offset(void)
{
  // For max in-place left and right, both wheels should have equal magnitude (symmetry), offset is lost due to clamping.
  JoystickProcessingResult js_left = processJoystick(JOYSTICK_CENTER - 500, JOYSTICK_CENTER, false);
  MotorTargets prevMt_left = {};
  MotorTargets mt_left = computeMotorTargets(js_left, prevMt_left);
  JoystickProcessingResult js_right = processJoystick(JOYSTICK_CENTER + 500, JOYSTICK_CENTER, false);
  MotorTargets prevMt_right = {};
  MotorTargets mt_right = computeMotorTargets(js_right, prevMt_right);
  int abs_left = abs(mt_left.targetLeft);
  int abs_right = abs(mt_right.targetRight);
  TEST_ASSERT_EQUAL_MESSAGE(abs_left, abs_right, "In-place turn: left and right magnitudes should match");
}

// ----------------------------- Tests: Slew limiter ---------------------------

// New tests for variable slew rate
void test_slewRateLimit_variable_slew(void)
{
  // Slow slew rate (deflection < FULL_THROTTLE_THRESHOLD)
  float slow_deflection = 0.3f;
  int current = 0;
  int target = 100;
  int result = slewRateLimit(current, target, slow_deflection);
  TEST_ASSERT_EQUAL_MESSAGE(MIN_MOTOR_SPEED, result, "Slow slew rate step should clamp to MIN_MOTOR_SPEED");

  // Fast slew rate (deflection >= FULL_THROTTLE_THRESHOLD)
  float fast_deflection = 0.85f;
  current = 0;
  target = 100;
  result = slewRateLimit(current, target, fast_deflection);
  TEST_ASSERT_EQUAL_MESSAGE(MIN_MOTOR_SPEED, result, "Fast slew rate step should clamp to MIN_MOTOR_SPEED");

  // Edge case: exactly at FULL_THROTTLE_THRESHOLD
  float edge_deflection = FULL_THROTTLE_THRESHOLD;
  current = 0;
  target = 100;
  result = slewRateLimit(current, target, edge_deflection);
  // Should still be slow
  TEST_ASSERT_EQUAL_MESSAGE(MIN_MOTOR_SPEED, result, "Edge slow slew rate step should clamp to MIN_MOTOR_SPEED");

  // Edge case: exactly at FULL_THROTTLE_THRESHOLD
  edge_deflection = FULL_THROTTLE_THRESHOLD;
  current = 0;
  target = 100;
  result = slewRateLimit(current, target, edge_deflection);
  // Should be fast
  TEST_ASSERT_EQUAL_MESSAGE(MIN_MOTOR_SPEED, result, "Edge fast slew rate step should clamp to MIN_MOTOR_SPEED");
}

void test_slewRateLimit_ramp_up(void)
{

  // ramp up from positive to positive
  TEST_ASSERT_EQUAL_MESSAGE(100 + RAMP_STEP_SLOW, slewRateLimit(100, 200, FULL_THROTTLE_THRESHOLD - 0.1f), "Ramp up: 100->200 should increase by RAMP_STEP"); // slow
  TEST_ASSERT_EQUAL_MESSAGE(100 + RAMP_STEP_FAST, slewRateLimit(100, 200, FULL_THROTTLE_THRESHOLD + 0.1f), "Ramp up: 100->200 should increase by RAMP_STEP"); // fast
  TEST_ASSERT_EQUAL_MESSAGE(MIN_MOTOR_SPEED, slewRateLimit(0, MIN_MOTOR_SPEED - 1, FULL_THROTTLE_THRESHOLD - 0.1f), "Below MIN_MOTOR_SPEED clamps to MIN_MOTOR_SPEED");
  TEST_ASSERT_EQUAL_MESSAGE(-MIN_MOTOR_SPEED, slewRateLimit(0, -MIN_MOTOR_SPEED + 1, FULL_THROTTLE_THRESHOLD - 0.1f), "Below -MIN_MOTOR_SPEED clamps to -MIN_MOTOR_SPEED");
  TEST_ASSERT_EQUAL_MESSAGE(MIN_MOTOR_SPEED, slewRateLimit(0, MIN_MOTOR_SPEED/2, FULL_THROTTLE_THRESHOLD - 0.1f), "Ramp up: 0->1 yields +1");
  TEST_ASSERT_EQUAL_MESSAGE(101, slewRateLimit(100, 101, FULL_THROTTLE_THRESHOLD - 0.1f), "Ramp up: 100 -> 101 yields +1 (NOT INCREASE BY RAMP STEP VALUE)");
  TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(MIN_MOTOR_SPEED + 1, 0, FULL_THROTTLE_THRESHOLD - 0.1f), "Ramp down: MIN_MOTOR_SPEED+1 -> 0 clamps to 0");

  // slow down from negative to positive
  TEST_ASSERT_EQUAL_MESSAGE(-100 + RAMP_STEP_SLOW, slewRateLimit(-100, 100, FULL_THROTTLE_THRESHOLD - 0.1f), "Ramp up: -100->100 increases by RAMP_STEP");
  TEST_ASSERT_EQUAL_MESSAGE(-100 + RAMP_STEP_FAST, slewRateLimit(-100, 100, FULL_THROTTLE_THRESHOLD + 0.1f), "Ramp up: -100->100 increases by RAMP_STEP");
}

void test_slewRateLimit_ramp_down(void)
{
  TEST_ASSERT_EQUAL_MESSAGE(200 - RAMP_STEP_SLOW, slewRateLimit(200, 100, FULL_THROTTLE_THRESHOLD - 0.1f), "200->100");
  TEST_ASSERT_EQUAL_MESSAGE(100 - RAMP_STEP_FAST, slewRateLimit(100, -100, FULL_THROTTLE_THRESHOLD + 0.1f), "100->-100");
  TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(0, -1, FULL_THROTTLE_THRESHOLD + 0.1f), "0->-1");
  TEST_ASSERT_EQUAL_MESSAGE(99, slewRateLimit(100, 99, FULL_THROTTLE_THRESHOLD + 0.1f), "100->99");
}

void test_slewRateLimit_zero_crossing(void)
{
  TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(20, -100, FULL_THROTTLE_THRESHOLD - 0.1f), "20->-100 clamps to 0");
  TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(-20, 100, FULL_THROTTLE_THRESHOLD - 0.1f), "-20->100 clamps to 0");
}

void test_slewRateLimit_small_steps(void)
{
  TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(1, 2, FULL_THROTTLE_THRESHOLD - 0.1f), "deadzone 1->2");
  TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(-1, -2, FULL_THROTTLE_THRESHOLD - 0.1f), "deadzone -1->-2");
  TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(1, 0, FULL_THROTTLE_THRESHOLD - 0.1f), "deadzone 1->0");
  TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(-1, 0, FULL_THROTTLE_THRESHOLD - 0.1f), "deadzone -1->0");
  TEST_ASSERT_EQUAL_MESSAGE(MIN_MOTOR_SPEED + 2, slewRateLimit(MIN_MOTOR_SPEED, MIN_MOTOR_SPEED + 2, FULL_THROTTLE_THRESHOLD - 0.1f), "Above MIN: +2");
  TEST_ASSERT_EQUAL_MESSAGE(-MIN_MOTOR_SPEED - 2, slewRateLimit(-MIN_MOTOR_SPEED, -MIN_MOTOR_SPEED - 2, FULL_THROTTLE_THRESHOLD - 0.1f), "Above MIN reverse: -2");
}

void test_slewRateLimit_min_speed_behavior(void)
{
  TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(MIN_MOTOR_SPEED + 5, 0, FULL_THROTTLE_THRESHOLD - 0.1f), "Down past MIN clamps to 0");
  TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(-MIN_MOTOR_SPEED - 5, 0, FULL_THROTTLE_THRESHOLD - 0.1f), "Down past -MIN clamps to 0");
  TEST_ASSERT_EQUAL_MESSAGE(MIN_MOTOR_SPEED, slewRateLimit(MIN_MOTOR_SPEED - 5, MIN_MOTOR_SPEED, FULL_THROTTLE_THRESHOLD - 0.1f), "Up to MIN yields MIN");
  TEST_ASSERT_EQUAL_MESSAGE(-MIN_MOTOR_SPEED, slewRateLimit(-MIN_MOTOR_SPEED + 5, -MIN_MOTOR_SPEED, FULL_THROTTLE_THRESHOLD - 0.1f), "Up to -MIN yields -MIN");
  TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(MIN_MOTOR_SPEED + 5, MIN_MOTOR_SPEED - 10, FULL_THROTTLE_THRESHOLD - 0.1f), "Crossing MIN while down: 0");
  TEST_ASSERT_EQUAL_MESSAGE(0, slewRateLimit(-MIN_MOTOR_SPEED - 5, -MIN_MOTOR_SPEED + 10, FULL_THROTTLE_THRESHOLD - 0.1f), "Crossing -MIN while up: 0");
}

void test_slewRateLimit_max_speed_behavior(void)
{
  TEST_ASSERT_EQUAL_MESSAGE(MAX_SPEED, slewRateLimit(MAX_SPEED - 5, MAX_SPEED, FULL_THROTTLE_THRESHOLD - 0.1f), "near MAX -> MAX");
  TEST_ASSERT_EQUAL_MESSAGE(-MAX_SPEED, slewRateLimit(-MAX_SPEED + 5, -MAX_SPEED, FULL_THROTTLE_THRESHOLD - 0.1f), "near -MAX -> -MAX");
  TEST_ASSERT_EQUAL_MESSAGE(-MAX_SPEED, slewRateLimit(-456, -456, FULL_THROTTLE_THRESHOLD - 0.1f), "clamp at -MAX if exceeded");
}

void test_shouldSkipSlewRate(void)
{
  // Direction reversal (should skip)
  TEST_ASSERT_TRUE(shouldSkipSlewRate(100, 100, -100, -100));
  TEST_ASSERT_TRUE(shouldSkipSlewRate(-100, -100, 100, 100));
  TEST_ASSERT_TRUE(shouldSkipSlewRate(100, -100, -100, -100)); // mixed L->R
  TEST_ASSERT_TRUE(shouldSkipSlewRate(-100, 100, 100, 100));   // mixed L->R

  // Aggressive forward (should NOT skip - use fast slew rate instead)
  int aggressive_fwd = (int)(0.91f * MAX_SPEED);
  TEST_ASSERT_FALSE(shouldSkipSlewRate(0, 0, aggressive_fwd, aggressive_fwd));
  TEST_ASSERT_FALSE(shouldSkipSlewRate(10, 10, aggressive_fwd, aggressive_fwd));

  // Aggressive reverse (should NOT skip - use fast slew rate instead)
  int aggressive_rev = (int)(-0.91f * MAX_SPEED);
  TEST_ASSERT_FALSE(shouldSkipSlewRate(0, 0, aggressive_rev, aggressive_rev));
  TEST_ASSERT_FALSE(shouldSkipSlewRate(-10, -10, aggressive_rev, aggressive_rev));

  // Not aggressive (should NOT skip)
  int mild_fwd = (int)(0.5f * MAX_SPEED);
  int mild_rev = (int)(-0.5f * MAX_SPEED);
  TEST_ASSERT_FALSE(shouldSkipSlewRate(0, 0, mild_fwd, mild_fwd));
  TEST_ASSERT_FALSE(shouldSkipSlewRate(0, 0, mild_rev, mild_rev));

  // Aggressive sharp turn (should skip)
  TEST_ASSERT_TRUE(shouldSkipSlewRate(0, 0, -MIN_MOTOR_SPEED, MIN_MOTOR_SPEED));
  TEST_ASSERT_TRUE(shouldSkipSlewRate(0, 0, MIN_MOTOR_SPEED, -MIN_MOTOR_SPEED));

  // Not sharp turn (should NOT skip)
  TEST_ASSERT_FALSE(shouldSkipSlewRate(0, 0, -MIN_MOTOR_SPEED + 1, MIN_MOTOR_SPEED - 1));
  TEST_ASSERT_FALSE(shouldSkipSlewRate(0, 0, MIN_MOTOR_SPEED - 1, -MIN_MOTOR_SPEED + 1));
}

// ----------------------------- Tests: Braking logic ---------------------------
void test_shouldApplyBraking(void)
{ 
  TEST_ASSERT_TRUE(shouldApplyBraking(BRAKE_APPLY_THRESHOLD, BRAKE_APPLY_THRESHOLD, 0, 0));
  TEST_ASSERT_TRUE(shouldApplyBraking(-BRAKE_APPLY_THRESHOLD, -BRAKE_APPLY_THRESHOLD, 0, 0));
  TEST_ASSERT_FALSE(shouldApplyBraking(BRAKE_APPLY_THRESHOLD - 5, BRAKE_APPLY_THRESHOLD - 5, 0, 0));
  TEST_ASSERT_FALSE(shouldApplyBraking(-BRAKE_APPLY_THRESHOLD + 5, -BRAKE_APPLY_THRESHOLD + 5, 0, 0));
}

void test_dynamicBrakingApplied(void)
{
  js = processJoystick(JOYSTICK_CENTER, JOYSTICK_CENTER, false); // CENTER = stop
  {
    MotorTargets prevMt = { .targetLeft = MAX_SPEED, .targetRight = MAX_SPEED, .outputLeft = MAX_SPEED, .outputRight = MAX_SPEED };
    mt = computeMotorTargets(js, prevMt);
  }

  TEST_ASSERT_EQUAL_MESSAGE(-MIN_MOTOR_SPEED, mt.targetLeft, "Small Reverse expected (dynamic braking)");
  TEST_ASSERT_EQUAL_MESSAGE(-MIN_MOTOR_SPEED, mt.targetRight, "Small Reverse expected (dynamic braking)");

  // vice versa

  {
    MotorTargets prevMt = { .targetLeft = -MAX_SPEED, .targetRight = -MAX_SPEED, .outputLeft = -MAX_SPEED, .outputRight = -MAX_SPEED };
    mt = computeMotorTargets(js, prevMt);
  }
  TEST_ASSERT_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.targetLeft, "Small Forward expected (dynamic braking)");
  TEST_ASSERT_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.targetRight, "Small Forward expected (dynamic braking)");
}

void test_dynamicBraking_noOscillation(void)
{
  // Simulate the exact scenario: forward motion -> center -> should settle, not oscillate
  js = processJoystick(JOYSTICK_CENTER, JOYSTICK_CENTER, false); // CENTER = stop
  
  // Start with high-speed forward motion (like from the log)
  MotorTargets prevMt = { .targetLeft = 243, .targetRight = 243, .outputLeft = 243, .outputRight = 243, .brakingApplied = false };
  
  // First center cycle: should apply dynamic braking
  mt = computeMotorTargets(js, prevMt);
  TEST_ASSERT_TRUE_MESSAGE(mt.brakingApplied, "First center cycle should apply braking");
  TEST_ASSERT_EQUAL_MESSAGE(-MIN_MOTOR_SPEED, mt.targetLeft, "First cycle: reverse braking");
  TEST_ASSERT_EQUAL_MESSAGE(-MIN_MOTOR_SPEED, mt.targetRight, "First cycle: reverse braking");
  
  // Second center cycle: should NOT re-apply dynamic braking (should settle to 0)
  MotorTargets secondPrevMt = mt; // Use previous result
  mt = computeMotorTargets(js, secondPrevMt);
  TEST_ASSERT_FALSE_MESSAGE(mt.brakingApplied, "Second center cycle should NOT apply braking again");
  TEST_ASSERT_EQUAL_MESSAGE(0, mt.targetLeft, "Second cycle: should target 0");
  TEST_ASSERT_EQUAL_MESSAGE(0, mt.targetRight, "Second cycle: should target 0");
  
  // Third center cycle: should remain settled at 0
  MotorTargets thirdPrevMt = mt;
  mt = computeMotorTargets(js, thirdPrevMt);
  TEST_ASSERT_FALSE_MESSAGE(mt.brakingApplied, "Third center cycle should NOT apply braking");
  TEST_ASSERT_EQUAL_MESSAGE(0, mt.targetLeft, "Third cycle: should remain at 0");
  TEST_ASSERT_EQUAL_MESSAGE(0, mt.targetRight, "Third cycle: should remain at 0");
  TEST_ASSERT_EQUAL_MESSAGE(0, mt.outputLeft, "Third cycle: output should be 0");
  TEST_ASSERT_EQUAL_MESSAGE(0, mt.outputRight, "Third cycle: output should be 0");
}

void test_fullThrottle_reachesMaxSpeed(void)
{
  // Test full forward throttle reaches maximum speed with fast slew rate
  js = processJoystick(JOYSTICK_CENTER, 994, false); // Full forward (like from log)
  MotorTargets prevMt = { .targetLeft = 0, .targetRight = 0, .outputLeft = 0, .outputRight = 0 };
  
  // Simulate multiple cycles to allow slew rate to ramp up
  for (int i = 0; i < 1000; ++i) {
    delay(LOOP_DELAY_MS); // Simulate time between cycles
    mt = computeMotorTargets(js, prevMt);
    prevMt = mt;
  }
  
  // After multiple cycles with full deflection, should reach high speeds (not stuck at MIN_MOTOR_SPEED)
  TEST_ASSERT_GREATER_THAN_MESSAGE(150, mt.outputLeft, "Full forward should reach high speeds, not be stuck at MIN_MOTOR_SPEED");
  TEST_ASSERT_GREATER_THAN_MESSAGE(150, mt.outputRight, "Full forward should reach high speeds, not be stuck at MIN_MOTOR_SPEED");
  
  // Test full reverse throttle reaches maximum speed  
  js = processJoystick(JOYSTICK_CENTER, -18, false); // Full reverse (like from log)
  prevMt = { .targetLeft = 0, .targetRight = 0, .outputLeft = 0, .outputRight = 0 };
  
  // Simulate multiple cycles to allow slew rate to ramp up
  for (int i = 0; i < 10; ++i) {
    mt = computeMotorTargets(js, prevMt);
    prevMt = mt;
  }
  
  // After multiple cycles with full deflection, should reach high reverse speeds 
  TEST_ASSERT_LESS_THAN_MESSAGE(-150, mt.outputLeft, "Full reverse should reach high speeds, not be stuck at -MIN_MOTOR_SPEED");
  TEST_ASSERT_LESS_THAN_MESSAGE(-150, mt.outputRight, "Full reverse should reach high speeds, not be stuck at -MIN_MOTOR_SPEED");
}


void test_slewRateLimit_at_boundaries(void)
{
  TEST_ASSERT_EQUAL(MAX_SPEED, slewRateLimit(MAX_SPEED, MAX_SPEED + 100, 0.0f));
  TEST_ASSERT_EQUAL(-MAX_SPEED, slewRateLimit(-MAX_SPEED, -MAX_SPEED - 100, 0.0f));
}

// ----------------------------- Tests: Joystick & targets ---------------------
void test_processJoystick_buzzer(void)
{
  js = processJoystick(JOYSTICK_CENTER, JOYSTICK_CENTER, true);
  TEST_ASSERT_TRUE(js.buzzerOn);
}

void test_processJoystick_movement(void)
{
  js = processJoystick(JOYSTICK_CENTER + 50, JOYSTICK_CENTER - 100, false);
  TEST_ASSERT_INT_WITHIN(2, JOYSTICK_CENTER + 50, js.rawX);
  TEST_ASSERT_INT_WITHIN(2, JOYSTICK_CENTER - 100, js.rawY);
  TEST_ASSERT_FALSE(js.buzzerOn);
}

void test_processJoystick_deadzone_behavior(void)
{
  js = processJoystick(514, 510, false);
  TEST_ASSERT_INT_WITHIN(3, 514, js.rawX);
  TEST_ASSERT_INT_WITHIN(3, 510, js.rawY);
}

void test_computeMotorTargets_still(void)
{
  js = processJoystick(JOYSTICK_CENTER, JOYSTICK_CENTER, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_EQUAL_MESSAGE(0, mt.targetLeft, "Left motor target should be 0");
  TEST_ASSERT_EQUAL_MESSAGE(0, mt.targetRight, "Right motor target should be 0");
}

void test_computeMotorTargets_forward(void)
{

  js = processJoystick(JOYSTICK_CENTER, JOYSTICK_CENTER + 100, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.targetLeft, "Left >= MIN");
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.targetRight, "Right >= MIN");
}

void test_computeMotorTargets_reverse(void)
{
  js = processJoystick(JOYSTICK_CENTER, JOYSTICK_CENTER - 100, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(-MIN_MOTOR_SPEED, mt.targetLeft, "Left <= -MIN");
  TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(-MIN_MOTOR_SPEED, mt.targetRight, "Right <= -MIN");
}

void test_computeMotorTargets_right_turn(void)
{
  js = processJoystick(JOYSTICK_CENTER + 500, JOYSTICK_CENTER, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.targetLeft, "Left >= MIN");
  TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(-MIN_MOTOR_SPEED, mt.targetRight, "Right <= -MIN");
}

void test_computeMotorTargets_left_turn(void)
{
  js = processJoystick(JOYSTICK_CENTER - 500, JOYSTICK_CENTER, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(-MIN_MOTOR_SPEED, mt.targetLeft, "Left < -MIN");
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.targetRight, "Right > MIN");
}

void test_computeMotorTargets_Mixing(void)
{
  // forward + slight right
  js = processJoystick(JOYSTICK_CENTER + 50, JOYSTICK_CENTER + 500, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.targetLeft, "Left >= MIN");
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.targetRight, "Right >= MIN");
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(mt.targetRight, mt.targetLeft, "Left >= Right (right turn)");

  // forward + slight left
  js = processJoystick(JOYSTICK_CENTER - 50, JOYSTICK_CENTER + 500, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.targetLeft, "Left >= MIN");
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.targetRight, "Right >= MIN");
  TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(mt.targetLeft, mt.targetRight, "Left <= Right (left turn, allows equality if LR_OFFSET=0)");
}

void test_computeMotorTargets_skipSlew_targetLeft(void)
{
  // Test: Small X deflection (gentle turn) should NOT skip slew rate
  int gentle_deflection = JOYSTICK_DEADZONE + 10; // just above deadzone
  js = processJoystick(JOYSTICK_CENTER + gentle_deflection, JOYSTICK_CENTER, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_FALSE_MESSAGE(mt.skipSlewRate, "Gentle right turn should not skip slew rate");

  // Test: High X deflection (aggressive turn) SHOULD skip slew rate
  int high_deflection = (int)(0.90f * (float)JOYSTICK_CENTER + 0.5f); // To get steppedRatioLR >= 0.90, need rawRatioLR >= 0.90
  js = processJoystick(JOYSTICK_CENTER + high_deflection, JOYSTICK_CENTER, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_TRUE_MESSAGE(mt.skipSlewRate, "Aggressive right turn should skip slew rate");

  js = processJoystick(JOYSTICK_CENTER - high_deflection, JOYSTICK_CENTER, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_TRUE_MESSAGE(mt.skipSlewRate, "Aggressive left turn should skip slew rate");
}

void test_computeMotorTargets_skipSlew_forwardsBackwards(void)
{
  // Reset motor state to ensure clean test
  mt = {};
  
  // Gentle forward (just above center)
  int gentle_forward = JOYSTICK_CENTER + JOYSTICK_DEADZONE + 10;
  js = processJoystick(JOYSTICK_CENTER, gentle_forward, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_FALSE_MESSAGE(mt.skipSlewRate, "Gentle forward should not skip slew rate");

  // Reset state again for backward test
  mt = {};
  
  // Gentle backward (just below center)
  int gentle_backward = JOYSTICK_CENTER - JOYSTICK_DEADZONE - 10;
  js = processJoystick(JOYSTICK_CENTER, gentle_backward, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_FALSE_MESSAGE(mt.skipSlewRate, "Gentle backward should not skip slew rate");

  // Aggressive backward (large deflection)
  int aggressive_backward = JOYSTICK_CENTER - (int)(0.90f * JOYSTICK_CENTER + 0.5f); // matches skipSlew threshold
  js = processJoystick(JOYSTICK_CENTER, aggressive_backward, false);
  {
    MotorTargets prevMt = { .targetLeft = 100, .targetRight = 100 };
    mt = computeMotorTargets(js, prevMt);
  }
  TEST_ASSERT_FALSE_MESSAGE(mt.skipSlewRate, "Aggressive backward should use fast slew rate, not skip slew rate");
}

void test_computeMotorTargets_deadzone(void)
{
  // Both axes just inside deadzone
  js = processJoystick(JOYSTICK_CENTER + (JOYSTICK_DEADZONE / 2), JOYSTICK_CENTER + (JOYSTICK_DEADZONE / 2), false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_EQUAL(0, mt.targetLeft);
  TEST_ASSERT_EQUAL(0, mt.targetRight);
}

void test_computeMotorTargets_edge_of_deadzone(void)
{
  // Just outside deadzone on X-
  js = processJoystick(JOYSTICK_CENTER - JOYSTICK_DEADZONE - 1, JOYSTICK_CENTER, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_NOT_EQUAL(0, mt.targetRight);

  // Just outside deadzone on X+
  js = processJoystick(JOYSTICK_CENTER + JOYSTICK_DEADZONE + 1, JOYSTICK_CENTER, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_NOT_EQUAL(0, mt.targetLeft);

  // Just outside deadzone on Y+
  js = processJoystick(JOYSTICK_CENTER, JOYSTICK_CENTER + JOYSTICK_DEADZONE + 1, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_NOT_EQUAL(0, mt.targetLeft);

  // Just outside deadzone on Y-
  js = processJoystick(JOYSTICK_CENTER, JOYSTICK_CENTER - JOYSTICK_DEADZONE - 1, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_NOT_EQUAL(0, mt.targetLeft);
}

void test_computeMotorTargets_max_diagonal(void)
{
  // Both axes at maximum: expect both wheels to move forward at max mixing
  js = processJoystick(MAX_ADC_VALUE, MAX_ADC_VALUE, false); // Top-right corner
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.targetLeft, "Left wheel should move forward");
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.targetRight, "Right wheel should move forward");
  TEST_ASSERT_TRUE_MESSAGE(mt.targetLeft != mt.targetRight, "Diagonal mixing: wheels should not be equal at max diagonal");
}

void test_processJoystick_NoMotion(void)
{
  js = processJoystick(JOYSTICK_CENTER, JOYSTICK_CENTER, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_INT_WITHIN(JOYSTICK_DEADZONE, JOYSTICK_CENTER, js.rawX);
  TEST_ASSERT_INT_WITHIN(JOYSTICK_DEADZONE, JOYSTICK_CENTER, js.rawY);
  TEST_ASSERT_EQUAL(0, mt.targetLeft);
  TEST_ASSERT_EQUAL(0, mt.targetRight);
}

void test_processJoystick_Deadzone_NoMotion(void)
{
  // Test several points within the deadzone around the center
  int center = JOYSTICK_CENTER;
  int dz = JOYSTICK_DEADZONE;
  for (int dx = -dz + 1; dx < dz; ++dx) {
    for (int dy = -dz + 1; dy < dz; ++dy) {
      js = processJoystick(center + dx, center + dy, false);
      mt = computeMotorTargets(js, mt);
      TEST_ASSERT_EQUAL_MESSAGE(0, mt.targetLeft, "Left should be 0 in deadzone");
      TEST_ASSERT_EQUAL_MESSAGE(0, mt.targetRight, "Right should be 0 in deadzone");
    }
  }
}

void test_slewRateLimit_RampsDownToZero(void)
{
  js = processJoystick(JOYSTICK_CENTER, JOYSTICK_CENTER, false);
  mt = computeMotorTargets(js, mt);
  for (int i = 0; i < 20; ++i)
    mt = computeMotorTargets(js, mt);
  TEST_ASSERT_EQUAL(0, mt.targetLeft);
  TEST_ASSERT_EQUAL(0, mt.targetRight);
}

void test_SlewRateLimit_FullCycleAccelerateAndStop(void)
{
  js = processJoystick(JOYSTICK_CENTER, MAX_ADC_VALUE, false);
  for (int i = 0; i < 100; ++i)
    mt = computeMotorTargets(js, mt);
  js = processJoystick(JOYSTICK_CENTER, JOYSTICK_CENTER, false);
  for (int i = 0; i < 200; ++i)  // More iterations to allow dynamic braking to complete
    mt = computeMotorTargets(js, mt);
  // After dynamic braking, motors should be stopped (targets may be at MIN_MOTOR_SPEED for braking, but outputs should be 0 or low)
  TEST_ASSERT_TRUE_MESSAGE(abs(mt.outputLeft) <= MIN_MOTOR_SPEED, "Left motor output should be stopped or applying braking");
  TEST_ASSERT_TRUE_MESSAGE(abs(mt.outputRight) <= MIN_MOTOR_SPEED, "Right motor output should be stopped or applying braking");
}

void test_partial_forward_backwards_both_wheels_move(void)
{
  // Use values outside the joystick deadzone for both axes
  int x_offset = JOYSTICK_DEADZONE + 10;
  int y_offset = JOYSTICK_DEADZONE + 20;

  // Simulate partial forward (Y above center) and slight right (X above center)
  js = processJoystick(JOYSTICK_CENTER + x_offset, JOYSTICK_CENTER + y_offset, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.outputLeft, "Left wheel should move (>= MIN_MOTOR_SPEED)");

  // Simulate partial forward (Y above center) and slight left (X below center)
  js = processJoystick(JOYSTICK_CENTER - x_offset, JOYSTICK_CENTER + y_offset, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.outputRight, "Right wheel should move (>= MIN_MOTOR_SPEED)");

  // Simulate partial backward (Y below center) and slight right (X above center)
  js = processJoystick(JOYSTICK_CENTER + x_offset, JOYSTICK_CENTER - y_offset, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(-MIN_MOTOR_SPEED, mt.outputLeft, "Left wheel should move (>= MIN_MOTOR_SPEED)");

  // Simulate partial backward (Y below center) and slight left (X below center)
  js = processJoystick(JOYSTICK_CENTER - x_offset, JOYSTICK_CENTER - y_offset, false);
  mt = computeMotorTargets(js, mt);
  TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(-MIN_MOTOR_SPEED, mt.outputRight, "Right wheel should move (>= MIN_MOTOR_SPEED)");
}


// ----------------------------- Tests: Config constants -----------------------
void test_configuration_constants(void)
{
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, MIN_MOTOR_SPEED, "MIN_MOTOR_SPEED > 0");
  TEST_ASSERT_LESS_THAN_MESSAGE(MAX_SPEED, MIN_MOTOR_SPEED, "MIN < MAX");
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, RAMP_STEP_SLOW, "RAMP_STEP_SLOW > 0");
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, RAMP_STEP_FAST, "RAMP_STEP_FAST > 0");
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, JOYSTICK_DEADZONE, "DEADZONE > 0");
  TEST_ASSERT_EQUAL_MESSAGE(JOYSTICK_CENTER, JOYSTICK_CENTER, "CENTER == JOYSTICK_CENTER");
  TEST_ASSERT_EQUAL_MESSAGE(JOYSTICK_CENTER + JOYSTICK_DEADZONE, FORWARD_THRESHOLD, "FORWARD threshold");
  TEST_ASSERT_EQUAL_MESSAGE(JOYSTICK_CENTER - JOYSTICK_DEADZONE, BACKWARD_THRESHOLD, "BACKWARD threshold");
}

static int adc_to_percent(int adc_value)
{
  // Map ADC value (0-MAX_ADC_VALUE) to percentage (0-100)
  return (adc_value * 100) / MAX_ADC_VALUE;
}

// ----------------------------- Tests: Joystick mapping -----------------------
void test_joystick_center_position_mapping(void)
{
  int throttleResult = calculateThrottlePercent(JOYSTICK_CENTER);
  int targetLeftResult = calculateThrottlePercent(JOYSTICK_CENTER);
  TEST_ASSERT_INT_WITHIN_MESSAGE(3, 0, throttleResult, "Center Y -> ~0%");
  TEST_ASSERT_INT_WITHIN_MESSAGE(3, 0, targetLeftResult, "Center X -> ~0%");
}

void test_joystick_full_range_mapping(void)
{
  // Test throttle percent mapping at low and high ADC values
  int throttleLow = calculateThrottlePercent(100);  // Should be negative (reverse)
  int throttleHigh = calculateThrottlePercent(900); // Should be positive (forward)
  TEST_ASSERT_INT_WITHIN_MESSAGE(5, -80, throttleLow, "Y=100 ADC -> ~-80% throttle (reverse)");
  TEST_ASSERT_INT_WITHIN_MESSAGE(5, 76, throttleHigh, "Y=900 ADC -> ~76% throttle (forward)");

  // Test left/right percent mapping at low and high ADC values
  int targetLeftLow = calculateThrottlePercent(100);
  int targetLeftHigh = calculateThrottlePercent(900);
  TEST_ASSERT_EQUAL_INT_MESSAGE(-80, targetLeftLow, "X=100 ADC -> ~-80% left/right");
  TEST_ASSERT_EQUAL_INT_MESSAGE(76, targetLeftHigh, "X=900 ADC -> ~76% left/right");
}

void test_joystick_deadzone_functionality(void)
{
  int throttleResult = calculateThrottlePercent(550);
  int targetLeftResult = calculateThrottlePercent(550);
  TEST_ASSERT_GREATER_THAN_MESSAGE(3, throttleResult, "> 3% (forward)");
  TEST_ASSERT_GREATER_THAN_MESSAGE(3, targetLeftResult, "> 3%");
}

void test_leftmost_position_no_deadzone_interference(void)
{
  TEST_ASSERT_EQUAL_INT_MESSAGE(-100, calculateThrottlePercent(0), "x=0 -> -100%");
  TEST_ASSERT_EQUAL_INT_MESSAGE(100, calculateThrottlePercent(MAX_ADC_VALUE), "x=MAX_ADC_VALUE -> 100%");
  int throttleCenter = calculateThrottlePercent(JOYSTICK_CENTER);
  TEST_ASSERT_INT_WITHIN_MESSAGE(3, 0, throttleCenter, "Center Y ~ 0%");
}

void test_xMin_underflow_protection(void)
{
  TEST_ASSERT_EQUAL_INT_MESSAGE(-100, calculateThrottlePercent(-10), "clamp <-100%");
}

void test_xMax_overflow_protection(void)
{
  TEST_ASSERT_EQUAL_INT_MESSAGE(+100, calculateThrottlePercent(1050), "clamp >+100%");
}

void test_leftmost_position_real_world(void)
{
  TEST_ASSERT_EQUAL_INT_MESSAGE(-100, calculateThrottlePercent(0), "x=0 -> -100%");
  TEST_ASSERT_LESS_THAN_MESSAGE(-80, calculateThrottlePercent(10), "x=10 strong -ve");
  TEST_ASSERT_LESS_THAN_MESSAGE(-70, calculateThrottlePercent(50), "x=50 strong -ve");
}

void test_rightmost_position_real_world(void)
{
  TEST_ASSERT_EQUAL_INT_MESSAGE(100, calculateThrottlePercent(MAX_ADC_VALUE), "x=MAX_ADC_VALUE -> 100%");
  TEST_ASSERT_GREATER_THAN_MESSAGE(90, calculateThrottlePercent(1000), ">90%");
  TEST_ASSERT_GREATER_THAN_MESSAGE(80, calculateThrottlePercent(950), ">80%");
}

void test_smooth_progression_no_jumps(void)
{
  int center = calculateThrottlePercent(JOYSTICK_CENTER);
  int slightRight = calculateThrottlePercent(550);
  int moreRight = calculateThrottlePercent(600);
  TEST_ASSERT_LESS_THAN_MESSAGE(20, abs(slightRight - center), "<20%");
  TEST_ASSERT_LESS_THAN_MESSAGE(25, abs(moreRight - slightRight), "<25%");

  int slightLeft = calculateThrottlePercent(474);
  int moreLeft = calculateThrottlePercent(424);
  TEST_ASSERT_LESS_THAN_MESSAGE(20, abs(slightLeft - center), "<20%");
  TEST_ASSERT_LESS_THAN_MESSAGE(25, abs(moreLeft - slightLeft), "<25%");
}

void test_narrow_range_learning_problem(void)
{
  TEST_ASSERT_EQUAL_INT_MESSAGE(-100, calculateThrottlePercent(0), "saturate -100%");
  TEST_ASSERT_EQUAL_INT_MESSAGE(100, calculateThrottlePercent(MAX_ADC_VALUE), "saturate 100%");
}

void test_conservative_range_expansion(void)
{
  TEST_ASSERT_EQUAL_INT_MESSAGE(-100, calculateThrottlePercent(0), "-100%");
  TEST_ASSERT_EQUAL_INT_MESSAGE(100, calculateThrottlePercent(MAX_ADC_VALUE), "100%");
}

void test_joystick_processJoystick_integration(void)
{
  JoystickProcessingResult r1 = processJoystick(400, 400, false);
  TEST_ASSERT_TRUE_MESSAGE(r1.rawX == 400, "preserve rawX");
  TEST_ASSERT_TRUE_MESSAGE(r1.rawY == 400, "preserve rawY");
  TEST_ASSERT_TRUE_MESSAGE(r1.rawRatioLR < 0, "Left -> negative ratio");

  JoystickProcessingResult r2 = processJoystick(624, 624, false);
  TEST_ASSERT_TRUE_MESSAGE(r2.rawX == 624, "preserve rawX");
  TEST_ASSERT_TRUE_MESSAGE(r2.rawY == 624, "preserve rawY");
  TEST_ASSERT_TRUE_MESSAGE(r2.rawRatioLR > 0, "Right -> positive ratio");

  JoystickProcessingResult r3 = processJoystick(JOYSTICK_CENTER, JOYSTICK_CENTER, false);
  TEST_ASSERT_EQUAL_INT_MESSAGE(JOYSTICK_CENTER, r3.rawX, "Center rawX");
  TEST_ASSERT_EQUAL_INT_MESSAGE(JOYSTICK_CENTER, r3.rawY, "Center rawY");
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.1f, 0.0f, r3.rawRatioLR, "~0 ratio");
}

// ----------------------------- Tests: Fill bar logic -------------------------
static const int DISPLAY_WIDTH = 128;
static const int DISPLAY_HEIGHT = 64;

void test_center_position_no_fill(void)
{
  FillBarResult r = calculateThrottleFillBar(0, DISPLAY_WIDTH);
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, r.fillWidth, "center throttle no width");
  TEST_ASSERT_TRUE_MESSAGE(r.centerLineDrawn, "center line drawn");
}

void test_throttle_fillbar_from_center_upward(void)
{
  FillBarResult r = calculateThrottleFillBar(50, DISPLAY_WIDTH);
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, r.fillWidth, "throttle rightward width");
  TEST_ASSERT_EQUAL_INT_MESSAGE(DISPLAY_WIDTH / 2, r.fillStartX, "starts at center X");
}

void test_throttle_fillbar_from_center_downward(void)
{
  FillBarResult r = calculateThrottleFillBar(-50, DISPLAY_WIDTH);
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, r.fillWidth, "throttle leftward width");
  TEST_ASSERT_LESS_THAN_MESSAGE(DISPLAY_WIDTH / 2, r.fillStartX, "starts left of center X");
}

void test_left_right_fillbar_from_center_rightward(void)
{
  FillBarResult r = calculateThrottleFillBar(25, DISPLAY_WIDTH);
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, r.fillWidth, "L/R rightward width");
  TEST_ASSERT_EQUAL_INT_MESSAGE(DISPLAY_WIDTH / 2, r.fillStartX, "starts at center X");
}

void test_left_right_fillbar_from_center_leftward(void)
{
  FillBarResult r = calculateThrottleFillBar(-25, DISPLAY_WIDTH);
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, r.fillWidth, "L/R leftward width");
  TEST_ASSERT_LESS_THAN_MESSAGE(DISPLAY_WIDTH / 2, r.fillStartX, "starts left of center X");
}

void test_maximum_fillbar_constraints(void)
{
  FillBarResult r = calculateThrottleFillBar(100, DISPLAY_WIDTH);
  TEST_ASSERT_TRUE_MESSAGE(r.fillWidth <= DISPLAY_WIDTH / 2, "width <= half display");
  TEST_ASSERT_TRUE_MESSAGE(r.fillStartX >= 0, "X within bounds");
}

void test_fillbar_bar_edge_cases(void)
{
  FillBarResult r = calculateThrottleFillBar(-50, DISPLAY_WIDTH);
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, r.fillWidth, "max left has width");
  TEST_ASSERT_EQUAL_INT_MESSAGE(DISPLAY_WIDTH/2 - r.fillWidth, r.fillStartX, "fills from center leftward by fillWidth pixels");
}

void test_fillBarLogic(void)
{
  // Throttle bar: barW=6 => max fill is 3 (half), now -100% to +100%
  FillBarResult thr0 = calculateThrottleFillBar(0, 6);
  FillBarResult thrNeg = calculateThrottleFillBar(-100, 6);
  FillBarResult thrPos = calculateThrottleFillBar(100, 6);

  TEST_ASSERT_EQUAL(0, thr0.fillWidth);
  TEST_ASSERT_EQUAL(3, thrNeg.fillWidth);
  TEST_ASSERT_EQUAL(3, thrPos.fillWidth);

  // Left/Right bar: -100..100 across width
  FillBarResult lr0 = calculateThrottleFillBar(0, 52);
  FillBarResult lrLeft = calculateThrottleFillBar(-100, 52);
  FillBarResult lrRight = calculateThrottleFillBar(100, 52);
  TEST_ASSERT_EQUAL(0, lr0.fillWidth);
  TEST_ASSERT_EQUAL(26, lrLeft.fillWidth);
  TEST_ASSERT_EQUAL(26, lrRight.fillWidth);
}

// ---------------------------------------------------------------------------------
// main()
// ---------------------------------------------------------------------------------
int main(void)
{
  UNITY_BEGIN();

  // configuration constants expectations
  RUN_TEST(test_configuration_constants);

  // slew rate function tests
  RUN_TEST(test_slewRateLimit_ramp_up);
  RUN_TEST(test_slewRateLimit_ramp_down);
  RUN_TEST(test_slewRateLimit_zero_crossing);
  RUN_TEST(test_slewRateLimit_small_steps);
  RUN_TEST(test_slewRateLimit_min_speed_behavior);
  RUN_TEST(test_slewRateLimit_max_speed_behavior);
  RUN_TEST(test_shouldSkipSlewRate);
  RUN_TEST(test_slewRateLimit_at_boundaries);
  RUN_TEST(test_slewRateLimit_RampsDownToZero);
  RUN_TEST(test_SlewRateLimit_FullCycleAccelerateAndStop);
  RUN_TEST(test_slewRateLimit_variable_slew);
  

  // motor targets function tests
  RUN_TEST(test_computeMotorTargets_still);
  RUN_TEST(test_computeMotorTargets_right_turn);
  RUN_TEST(test_computeMotorTargets_left_turn);
  RUN_TEST(test_computeMotorTargets_forward);
  RUN_TEST(test_computeMotorTargets_reverse);
  RUN_TEST(test_computeMotorTargets_right_turn);
  RUN_TEST(test_computeMotorTargets_left_turn);
  RUN_TEST(test_computeMotorTargets_Mixing);
  RUN_TEST(test_computeMotorTargets_skipSlew_targetLeft);
  RUN_TEST(test_computeMotorTargets_skipSlew_forwardsBackwards);
  RUN_TEST(test_computeMotorTargets_deadzone);
  RUN_TEST(test_computeMotorTargets_edge_of_deadzone);
  RUN_TEST(test_computeMotorTargets_max_diagonal);

  // joystick => motor targets + action tests
  RUN_TEST(test_processJoystick_NoMotion);
  RUN_TEST(test_processJoystick_deadzone_behavior);
  RUN_TEST(test_processJoystick_Deadzone_NoMotion);
  RUN_TEST(test_processJoystick_movement);
  RUN_TEST(test_processJoystick_buzzer);

  // braking integration tests
  RUN_TEST(test_shouldApplyBraking);
  RUN_TEST(test_dynamicBrakingApplied);
  RUN_TEST(test_dynamicBraking_noOscillation);
  RUN_TEST(test_fullThrottle_reachesMaxSpeed);

  // joystick mapping tests (to throttle %, to L/R %)
  RUN_TEST(test_joystick_center_position_mapping);
  RUN_TEST(test_joystick_full_range_mapping);
  RUN_TEST(test_joystick_deadzone_functionality);

  RUN_TEST(test_center_position_no_fill);
  RUN_TEST(test_throttle_fillbar_from_center_upward);
  RUN_TEST(test_throttle_fillbar_from_center_downward);
  RUN_TEST(test_left_right_fillbar_from_center_rightward);
  RUN_TEST(test_left_right_fillbar_from_center_leftward);
  RUN_TEST(test_maximum_fillbar_constraints);
  RUN_TEST(test_fillbar_bar_edge_cases);

  RUN_TEST(test_leftmost_position_no_deadzone_interference);
  RUN_TEST(test_xMin_underflow_protection);
  RUN_TEST(test_xMax_overflow_protection);

  RUN_TEST(test_leftmost_position_real_world);
  RUN_TEST(test_rightmost_position_real_world);
  RUN_TEST(test_smooth_progression_no_jumps);
  RUN_TEST(test_narrow_range_learning_problem);
  RUN_TEST(test_conservative_range_expansion);

  RUN_TEST(test_fillBarLogic);

  // Regression: partial forward + backwards should move both wheels
  RUN_TEST(test_partial_forward_backwards_both_wheels_move);

  return UNITY_END();
}
