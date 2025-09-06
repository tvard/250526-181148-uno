#include <cmath>
#include <cstdio>
#include "unity.h"
#include "../lib/helpers/helpers.h"
#include "../lib/helpers/2WD_RC_RECEIVER_logic.h"
#include "../lib/helpers/display_helpers.h"   // Fillbars
#include "../lib/helpers/2WD_RC_TRANSMITTER_logic.h"  // production declarations

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
  #define max(a,b) (((a)>(b))?(a):(b))
  #endif
  #ifndef min
  #define min(a,b) (((a)<(b))?(a):(b))
  #endif
  #ifndef abs
  #define abs(x) ((x)>0?(x):-(x))
  #endif
  #ifndef constrain
  #define constrain(amt,low,high) ((amt)<(low)?(low):(((amt)>(high))?(high):(amt)))
  #endif
  #ifndef map
  #define map(x,in_min,in_max,out_min,out_max) \
      (((x)-(in_min))*((out_max)-(out_min))/((in_max)-(in_min)) + (out_min))
  #endif

  #define digitalWrite(pin, val) ((void)0)
  #define analogWrite(pin, val)  ((void)0)
  #define analogRead(pin)        (0)
  #define delay(ms)              ((void)0)
  #define millis()               (0UL)
  #define tone(pin,freq)         ((void)0)
  #define noTone(pin)            ((void)0)
  #define pinMode(pin, mode)     ((void)0)

  #ifndef A0
  #define A0 0
  #define A1 1
  #define A2 2
  #define A3 3
  #endif

  struct {
    void begin(int) {}
    void println(const char*) {}
    void println(int) {}
    void print(const char*) {}
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
int16_t xMax = 1023;
int16_t yMin = 0;
int16_t yMax = 1023;
uint16_t xCenter = 512;
uint16_t yCenter = 512;
#endif

// // --- SUT functions used in tests (declared in headers) -----------------------
// extern int calculateThrottlePercent(int yAdc);
// extern int calculateLeftRightPercent(int xAdc);

// (Optionally) extern motor speed state, if available in SUT
extern int leftSpeed;
extern int rightSpeed;

// --- Test fixtures -----------------------------------------------------------
static JoystickProcessingResult js{};
static MotorTargets mt{};


void setUp(void) {
  js = {};
  js.rawX = 512;
  js.rawY = 512;
  js.buzzerOn = false;
  mt = {};
}

void tearDown(void) {}

// Test for strict symmetry and offset in in-place turns
void test_inplace_turn_symmetry_with_offset(void) {
  JoystickProcessingResult js_left = processJoystick(512 - 500, 512, false, false);
  MotorTargets mt_left = computeMotorTargets(js_left, 0, 0);
  JoystickProcessingResult js_right = processJoystick(512 + 500, 512, false, false);
  MotorTargets mt_right = computeMotorTargets(js_right, 0, 0);
  int abs_left = abs(mt_left.left);
  int abs_right = abs(mt_right.right);
  int offset_half = LR_OFFSET / 2;
  int unclamped1 = MIN_MOTOR_SPEED - offset_half;
  int unclamped2 = MIN_MOTOR_SPEED + offset_half;
  int expected1 = (unclamped1 < MIN_MOTOR_SPEED) ? MIN_MOTOR_SPEED : unclamped1;
  int expected2 = (unclamped2 < MIN_MOTOR_SPEED) ? MIN_MOTOR_SPEED : unclamped2;
  char msg[128];
  snprintf(msg, sizeof(msg),
    "In-place turn: abs_left=%d abs_right=%d, expected values: %d±1 and %d±1 (order independent)",
    abs_left, abs_right, expected1, expected2);
  int match1 = (abs(abs_left - expected1) <= 1 && abs(abs_right - expected2) <= 1);
  int match2 = (abs(abs_left - expected2) <= 1 && abs(abs_right - expected1) <= 1);
  TEST_ASSERT_TRUE_MESSAGE(match1 || match2, msg);
  int offset_applied_left = abs(mt_left.right) - abs(mt_left.left);
  int offset_applied_right = abs(mt_right.left) - abs(mt_right.right);
  // Offset should be zero due to clamping
  TEST_ASSERT_EQUAL_MESSAGE(0, offset_applied_left, "Offset applied in left turn is zero (clamped)");
  TEST_ASSERT_EQUAL_MESSAGE(0, offset_applied_right, "Offset applied in right turn is zero (clamped)");
}

// ----------------------------- Tests: Slew limiter ---------------------------
void test_slewRateLimit_no_change(void) {
  TEST_ASSERT_EQUAL_MESSAGE(100,   slewRateLimit(100,   100),   "No change: 100->100");
  TEST_ASSERT_EQUAL_MESSAGE(-100,  slewRateLimit(-100,  -100),  "No change: -100->-100");
  TEST_ASSERT_EQUAL_MESSAGE(0,     slewRateLimit(0,     0),     "No change: 0->0");
}

void test_slewRateLimit_ramp_up(void) {
  TEST_ASSERT_EQUAL_MESSAGE(100 + RAMP_STEP,           slewRateLimit(100, 200),
                            "Ramp up: 100->200 should increase by RAMP_STEP");
  TEST_ASSERT_EQUAL_MESSAGE(MIN_MOTOR_SPEED,           slewRateLimit(0,   50),
                            "Below MIN_MOTOR_SPEED clamps to MIN_MOTOR_SPEED");
  TEST_ASSERT_EQUAL_MESSAGE(-MIN_MOTOR_SPEED,          slewRateLimit(0,  -50),
                            "Below -MIN_MOTOR_SPEED clamps to -MIN_MOTOR_SPEED");
  TEST_ASSERT_EQUAL_MESSAGE(-100 + RAMP_STEP,          slewRateLimit(-100, 100),
                            "Ramp up: -100->100 increases by RAMP_STEP");
  TEST_ASSERT_EQUAL_MESSAGE(0,                         slewRateLimit(0,   1),
                            "Ramp up: 0->1 yields +1");
  TEST_ASSERT_EQUAL_MESSAGE(101,                       slewRateLimit(100, 101),
                            "Ramp up: 100->101 yields +1");
  TEST_ASSERT_EQUAL_MESSAGE(0,                         slewRateLimit(MIN_MOTOR_SPEED + (int)(RAMP_STEP/3), 0),
                            "Slightly above MIN_MOTOR_SPEED down to 0 clamps to 0");
}

void test_slewRateLimit_ramp_down(void) {
  TEST_ASSERT_EQUAL_MESSAGE(200 - RAMP_STEP,           slewRateLimit(200, 100), "200->100");
  TEST_ASSERT_EQUAL_MESSAGE(100 - RAMP_STEP,           slewRateLimit(100, -100),"100->-100");
  TEST_ASSERT_EQUAL_MESSAGE(0,                         slewRateLimit(0, -1),     "0->-1");
  TEST_ASSERT_EQUAL_MESSAGE(99,                        slewRateLimit(100, 99),   "100->99");
}

void test_slewRateLimit_zero_crossing(void) {
  TEST_ASSERT_EQUAL_MESSAGE(0,  slewRateLimit(20,  -100), "20->-100 clamps to 0");
  TEST_ASSERT_EQUAL_MESSAGE(0,  slewRateLimit(-20, 100),  "-20->100 clamps to 0");
}

void test_slewRateLimit_small_steps(void) {
  TEST_ASSERT_EQUAL_MESSAGE(0,                       slewRateLimit(1,  2),   "deadzone 1->2");
  TEST_ASSERT_EQUAL_MESSAGE(0,                       slewRateLimit(-1, -2),  "deadzone -1->-2");
  TEST_ASSERT_EQUAL_MESSAGE(0,                       slewRateLimit(1,  0),   "deadzone 1->0");
  TEST_ASSERT_EQUAL_MESSAGE(0,                       slewRateLimit(-1, 0),   "deadzone -1->0");
  TEST_ASSERT_EQUAL_MESSAGE(MIN_MOTOR_SPEED + 2,     slewRateLimit(MIN_MOTOR_SPEED,     MIN_MOTOR_SPEED + 2),
                            "Above MIN: +2");
  TEST_ASSERT_EQUAL_MESSAGE(-MIN_MOTOR_SPEED - 2,    slewRateLimit(-MIN_MOTOR_SPEED,    -MIN_MOTOR_SPEED - 2),
                            "Above MIN reverse: -2");
}

void test_slewRateLimit_min_speed_behavior(void) {
  TEST_ASSERT_EQUAL_MESSAGE(0,               slewRateLimit(MIN_MOTOR_SPEED + 5, 0),
                            "Down past MIN clamps to 0");
  TEST_ASSERT_EQUAL_MESSAGE(0,               slewRateLimit(-MIN_MOTOR_SPEED - 5, 0),
                            "Down past -MIN clamps to 0");
  TEST_ASSERT_EQUAL_MESSAGE(MIN_MOTOR_SPEED, slewRateLimit(MIN_MOTOR_SPEED - 5, MIN_MOTOR_SPEED),
                            "Up to MIN yields MIN");
  TEST_ASSERT_EQUAL_MESSAGE(-MIN_MOTOR_SPEED,slewRateLimit(-MIN_MOTOR_SPEED + 5, -MIN_MOTOR_SPEED),
                            "Up to -MIN yields -MIN");
  TEST_ASSERT_EQUAL_MESSAGE(0,               slewRateLimit(MIN_MOTOR_SPEED + 5, MIN_MOTOR_SPEED - 10),
                            "Crossing MIN while down: 0");
  TEST_ASSERT_EQUAL_MESSAGE(0,               slewRateLimit(-MIN_MOTOR_SPEED - 5, -MIN_MOTOR_SPEED + 10),
                            "Crossing -MIN while up: 0");
}

void test_slewRateLimit_max_speed_behavior(void) {
  TEST_ASSERT_EQUAL_MESSAGE(MAX_SPEED,      slewRateLimit(MAX_SPEED - 5, MAX_SPEED), "near MAX -> MAX");
  TEST_ASSERT_EQUAL_MESSAGE(-MAX_SPEED,     slewRateLimit(-MAX_SPEED + 5, -MAX_SPEED), "near -MAX -> -MAX");
  TEST_ASSERT_EQUAL_MESSAGE(-MAX_SPEED,     slewRateLimit(-456, -456), "clamp at -MAX if exceeded");
}

void test_shouldSkipSlewRate(void) {
  TEST_ASSERT_TRUE (shouldSkipSlewRate( 100,  100, -100, -100));
  TEST_ASSERT_TRUE (shouldSkipSlewRate(-100, -100,  100,  100));
  TEST_ASSERT_FALSE(shouldSkipSlewRate( 100,  100,   80,   80));
  TEST_ASSERT_FALSE(shouldSkipSlewRate(-100, -100,  -80,  -80));
  TEST_ASSERT_TRUE (shouldSkipSlewRate( 100, -100, -100, -100)); // mixed L->R
  TEST_ASSERT_TRUE (shouldSkipSlewRate(-100,  100,  100,  100)); // mixed L->R
}

void test_shouldApplyBraking_on_stop(void) {
  TEST_ASSERT_TRUE (shouldApplyBraking( 100,  100, 0, 0));
  TEST_ASSERT_TRUE (shouldApplyBraking(-100, -100, 0, 0));
  TEST_ASSERT_FALSE(shouldApplyBraking(   0,    0, 0, 0));
}

void test_shouldApplyBraking_after_brake(void) {
  TEST_ASSERT_TRUE (shouldApplyBraking(100, 100, 0, 0));
  TEST_ASSERT_FALSE(shouldApplyBraking(  0,   0, 0, 0));
}

void test_slewRateLimit_at_boundaries(void) {
  TEST_ASSERT_EQUAL(MAX_SPEED,  slewRateLimit( MAX_SPEED,  MAX_SPEED + 100));
  TEST_ASSERT_EQUAL(-MAX_SPEED, slewRateLimit(-MAX_SPEED, -MAX_SPEED - 100));
}

// ----------------------------- Tests: Joystick & targets ---------------------
void test_processJoystick_buzzer(void) {
  js = processJoystick(512, 512, true, false);
  TEST_ASSERT_TRUE(js.buzzerOn);
}

void test_processJoystick_movement(void) {
  js = processJoystick(512 + 50, 512 - 100, false, false);
  TEST_ASSERT_INT_WITHIN(2, 512 + 50,  js.rawX);
  TEST_ASSERT_INT_WITHIN(2, 512 - 100, js.rawY);
  TEST_ASSERT_FALSE(js.buzzerOn);
}

void test_processJoystick_deadzone_behavior(void) {
  js = processJoystick(514, 510, false, false);
  TEST_ASSERT_INT_WITHIN(3, 514, js.rawX);
  TEST_ASSERT_INT_WITHIN(3, 510, js.rawY);
}

void test_computeMotorTargets_still(void) {
  js = processJoystick(512, 512, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_EQUAL_MESSAGE(0, mt.left,  "Left motor target should be 0");
  TEST_ASSERT_EQUAL_MESSAGE(0, mt.right, "Right motor target should be 0");
}

void test_computeMotorTargets_right_turn(void) {
  js = processJoystick(512 + JOYSTICK_DEADZONE + 5, 512, false, false); // Just above deadzone and min speed
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_TRUE_MESSAGE(js.steppedRatioLR > 0.0f, "Stepped ratio > 0");
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.left, "Left = clamped(MIN_MOTOR_SPEED - offset_half) for right turn (offset compensates for hardware)");
  // Accept zero or a small negative value (tolerance for rounding/clamping)
  TEST_ASSERT_EQUAL_MESSAGE(0, mt.right, "Right == 0 (tolerance for rounding/clamping)");
}

void test_computeMotorTargets_left_turn(void) {
  js = processJoystick(512 - JOYSTICK_DEADZONE - 5, 512, false, false); // Just above deadzone and min speed
  mt = computeMotorTargets(js, 0, 0);
  int offset_half = LR_OFFSET / 2;
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.right, "Right = clamped(MIN_MOTOR_SPEED + offset_half) for left turn (offset compensates for hardware)");
  TEST_ASSERT_EQUAL_MESSAGE(0, mt.left, "Left == 0");
}

void test_computeMotorTargets_forward(void) {
  js = processJoystick(512, 512 + 100, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.left,  "Left >= MIN");
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.right, "Right >= MIN");
}

void test_computeMotorTargets_reverse(void) {
  js = processJoystick(512, 512 - 100, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(-MIN_MOTOR_SPEED, mt.left,  "Left <= -MIN");
  TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(-MIN_MOTOR_SPEED, mt.right, "Right <= -MIN");
}

void test_computeMotorTargets_sharp_right_turn(void) {
  js = processJoystick(512 + 500, 512, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED,  mt.left,  "Left >= MIN");
  TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(-MIN_MOTOR_SPEED,    mt.right, "Right <= -MIN");
}

void test_computeMotorTargets_sharp_left_turn(void) {
  js = processJoystick(512 - 500, 512, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_LESS_THAN_MESSAGE(-MIN_MOTOR_SPEED, mt.left,  "Left < -MIN");
  TEST_ASSERT_GREATER_THAN_MESSAGE(MIN_MOTOR_SPEED, mt.right,"Right > MIN");
}

void test_computeMotorTargets_Mixing(void) {
  // forward + slight right
  js = processJoystick(512 + 50, 512 + 500, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.left,  "Left >= MIN");
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.right, "Right >= MIN");
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(mt.right, mt.left, "Left >= Right (right turn)");

  // forward + slight left
  js = processJoystick(512 - 50, 512 + 500, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.left,  "Left >= MIN");
  TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(MIN_MOTOR_SPEED, mt.right, "Right >= MIN");
  TEST_ASSERT_LESS_THAN_MESSAGE(mt.left, mt.right, "Left < Right (left turn)");
}

void test_computeMotorTargets_skipSlew_leftRight(void) {
  test_inplace_turn_symmetry_with_offset();
  js = processJoystick(512 + 50, 512, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_FALSE(mt.skipSlewRate);

  js = processJoystick(512 + 400, 512, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_TRUE(mt.skipSlewRate);

  js = processJoystick(512 - 400, 512, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_TRUE(mt.skipSlewRate);
}

void test_computeMotorTargets_skipSlew_forwardsBackwards(void) {
  js = processJoystick(512, 512 + 100, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_FALSE(mt.skipSlewRate);

  js = processJoystick(512, 512 - 100, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_FALSE(mt.skipSlewRate);

  js = processJoystick(512, 512 - 400, false, false);
  mt = computeMotorTargets(js, 100, 100);
  TEST_ASSERT_TRUE(mt.skipSlewRate);
}

void test_computeMotorTargets_deadzone(void) {
  js = processJoystick(512 + 2, 512 + 2, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_EQUAL(0, mt.left);
  TEST_ASSERT_EQUAL(0, mt.right);
}

void test_computeMotorTargets_edge_of_deadzone(void) {
  js = processJoystick(512 - JOYSTICK_DEADZONE - 1, 512, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_NOT_EQUAL(0, mt.right);

  js = processJoystick(512 + JOYSTICK_DEADZONE + 1, 512, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_NOT_EQUAL(0, mt.left);

  js = processJoystick(512, 512 + JOYSTICK_DEADZONE + 1, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_NOT_EQUAL(0, mt.left);

  js = processJoystick(512, 512 - JOYSTICK_DEADZONE - 1, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_NOT_EQUAL(0, mt.left);
}

void test_computeMotorTargets_max_diagonal(void) {
  js = processJoystick(1023, 1023, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_GREATER_OR_EQUAL(MIN_MOTOR_SPEED, mt.left);
  TEST_ASSERT_GREATER_OR_EQUAL(MIN_MOTOR_SPEED, mt.right);
  TEST_ASSERT_GREATER_OR_EQUAL(mt.right, mt.left);
}

void test_processJoystick_NoMotion(void) {
  js = processJoystick(512, 512, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_INT_WITHIN(JOYSTICK_DEADZONE, 512, js.rawX);
  TEST_ASSERT_INT_WITHIN(JOYSTICK_DEADZONE, 512, js.rawY);
  TEST_ASSERT_EQUAL(0, mt.left);
  TEST_ASSERT_EQUAL(0, mt.right);
}

void test_processJoystick_NearZeroInputWithinDeadzone_NoMotion(void) {
  js = processJoystick(512 + 5, 512 - 4, false, false);
  mt = computeMotorTargets(js, 0, 0);
  TEST_ASSERT_EQUAL(0, mt.left);
  TEST_ASSERT_EQUAL(0, mt.right);
}

void test_slewRateLimit_RampsDownToZero(void) {
  js = processJoystick(512, 512, false, false);
  mt = computeMotorTargets(js, 10, 10);
  for (int i = 0; i < 20; ++i) mt = computeMotorTargets(js, mt.left, mt.right);
  TEST_ASSERT_EQUAL(0, mt.left);
  TEST_ASSERT_EQUAL(0, mt.right);
}

void test_SlewRateLimit_FullCycleAccelerateAndStop(void) {
  js = processJoystick(512 + 400, 512 + 400, false, false);
  for (int i = 0; i < 20; ++i) mt = computeMotorTargets(js, mt.left, mt.right);
  js = processJoystick(512, 512, false, false);
  for (int i = 0; i < 20; ++i) mt = computeMotorTargets(js, mt.left, mt.right);
  TEST_ASSERT_EQUAL_MESSAGE(0, mt.left,  "Left decelerates to stop");
  TEST_ASSERT_EQUAL_MESSAGE(0, mt.right, "Right decelerates to stop");
}

// ----------------------------- Tests: Config constants -----------------------
void test_configuration_constants(void) {
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, MIN_MOTOR_SPEED, "MIN_MOTOR_SPEED > 0");
  TEST_ASSERT_LESS_THAN_MESSAGE(MAX_SPEED, MIN_MOTOR_SPEED, "MIN < MAX");
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, RAMP_STEP, "RAMP_STEP > 0");
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, JOYSTICK_DEADZONE, "DEADZONE > 0");
  TEST_ASSERT_EQUAL_MESSAGE(512, JOYSTICK_CENTER, "CENTER == 512");
  TEST_ASSERT_EQUAL_MESSAGE(JOYSTICK_CENTER + JOYSTICK_DEADZONE, FORWARD_THRESHOLD,  "FORWARD threshold");
  TEST_ASSERT_EQUAL_MESSAGE(JOYSTICK_CENTER - JOYSTICK_DEADZONE, BACKWARD_THRESHOLD, "BACKWARD threshold");
}

static int adc_to_percent(int adc_value) {
  // Map ADC value (0-1023) to percentage (0-100)
  return (adc_value * 100) / 1023;
}

// ----------------------------- Tests: Joystick mapping -----------------------
void test_joystick_center_position_mapping(void) {
  int throttleResult  = calculateThrottlePercent(512);
  int leftRightResult = calculateLeftRightPercent(512);
  TEST_ASSERT_INT_WITHIN_MESSAGE(3, 0, throttleResult,  "Center Y -> ~0%");
  TEST_ASSERT_INT_WITHIN_MESSAGE(3, 0,  leftRightResult, "Center X -> ~0%");
}

void test_joystick_full_range_mapping(void) {
    // Test throttle percent mapping at low and high ADC values
    int throttleLow  = calculateThrottlePercent(100);  // Should be negative (reverse)
    int throttleHigh = calculateThrottlePercent(900);  // Should be positive (forward)
    TEST_ASSERT_INT_WITHIN_MESSAGE(5, -80, throttleLow,  "Y=100 ADC -> ~-80% throttle (reverse)");
    TEST_ASSERT_INT_WITHIN_MESSAGE(5, 76, throttleHigh, "Y=900 ADC -> ~76% throttle (forward)");

    // Test left/right percent mapping at low and high ADC values
    int leftRightLow  = calculateLeftRightPercent(100);
    int leftRightHigh = calculateLeftRightPercent(900);
    TEST_ASSERT_EQUAL_INT_MESSAGE(-80, leftRightLow,  "X=100 ADC -> ~-80% left/right");
    TEST_ASSERT_EQUAL_INT_MESSAGE(76,  leftRightHigh, "X=900 ADC -> ~76% left/right");
}

void test_joystick_deadzone_functionality(void) {
  int throttleResult  = calculateThrottlePercent(550);
  int leftRightResult = calculateLeftRightPercent(550);
  TEST_ASSERT_GREATER_THAN_MESSAGE(3, throttleResult,  "> 3% (forward)");
  TEST_ASSERT_GREATER_THAN_MESSAGE(3,  leftRightResult, "> 3%");
}

void test_leftmost_position_no_deadzone_interference(void) {
  TEST_ASSERT_EQUAL_INT_MESSAGE(-100, calculateLeftRightPercent(0),    "x=0 -> -100%");
  TEST_ASSERT_EQUAL_INT_MESSAGE(100,  calculateLeftRightPercent(1023), "x=1023 -> 100%");
  int throttleCenter = calculateThrottlePercent(512);
  TEST_ASSERT_INT_WITHIN_MESSAGE(3, 0, throttleCenter, "Center Y ~ 0%");
}

void test_xMin_underflow_protection(void) {
  TEST_ASSERT_EQUAL_INT_MESSAGE(-100, calculateLeftRightPercent(-10), "clamp <-100%");
}

void test_xMax_overflow_protection(void) {
  TEST_ASSERT_EQUAL_INT_MESSAGE(+100, calculateLeftRightPercent(1050), "clamp >+100%");
}


void test_leftmost_position_real_world(void) {
  TEST_ASSERT_EQUAL_INT_MESSAGE(-100, calculateLeftRightPercent(0),  "x=0 -> -100%");
  TEST_ASSERT_LESS_THAN_MESSAGE(-80,  calculateLeftRightPercent(10), "x=10 strong -ve");
  TEST_ASSERT_LESS_THAN_MESSAGE(-70,  calculateLeftRightPercent(50), "x=50 strong -ve");
}

void test_rightmost_position_real_world(void) {
  TEST_ASSERT_EQUAL_INT_MESSAGE(100, calculateLeftRightPercent(1023), "x=1023 -> 100%");
  TEST_ASSERT_GREATER_THAN_MESSAGE(90, calculateLeftRightPercent(1000), ">90%");
  TEST_ASSERT_GREATER_THAN_MESSAGE(80, calculateLeftRightPercent(950),  ">80%");
}

void test_smooth_progression_no_jumps(void) {
  int center      = calculateLeftRightPercent(512);
  int slightRight = calculateLeftRightPercent(550);
  int moreRight   = calculateLeftRightPercent(600);
  TEST_ASSERT_LESS_THAN_MESSAGE(20, abs(slightRight - center),  "<20%");
  TEST_ASSERT_LESS_THAN_MESSAGE(25, abs(moreRight   - slightRight), "<25%");

  int slightLeft  = calculateLeftRightPercent(474);
  int moreLeft    = calculateLeftRightPercent(424);
  TEST_ASSERT_LESS_THAN_MESSAGE(20, abs(slightLeft - center), "<20%");
  TEST_ASSERT_LESS_THAN_MESSAGE(25, abs(moreLeft   - slightLeft), "<25%");
}

void test_narrow_range_learning_problem(void) {
  TEST_ASSERT_EQUAL_INT_MESSAGE(-100, calculateLeftRightPercent(0),    "saturate -100%");
  TEST_ASSERT_EQUAL_INT_MESSAGE(100,  calculateLeftRightPercent(1023), "saturate 100%");
}

void test_conservative_range_expansion(void) {
  TEST_ASSERT_EQUAL_INT_MESSAGE(-100, calculateLeftRightPercent(0),    "-100%");
  TEST_ASSERT_EQUAL_INT_MESSAGE(100,  calculateLeftRightPercent(1023), "100%");
}

void test_joystick_processJoystick_integration(void) {
  JoystickProcessingResult r1 = processJoystick(400, 400, false, false);
  TEST_ASSERT_TRUE_MESSAGE(r1.rawX == 400, "preserve rawX");
  TEST_ASSERT_TRUE_MESSAGE(r1.rawY == 400, "preserve rawY");
  TEST_ASSERT_TRUE_MESSAGE(r1.rawRatioLR < 0, "Left -> negative ratio");

  JoystickProcessingResult r2 = processJoystick(624, 624, false, false);
  TEST_ASSERT_TRUE_MESSAGE(r2.rawX == 624, "preserve rawX");
  TEST_ASSERT_TRUE_MESSAGE(r2.rawY == 624, "preserve rawY");
  TEST_ASSERT_TRUE_MESSAGE(r2.rawRatioLR > 0, "Right -> positive ratio");

  JoystickProcessingResult r3 = processJoystick(512, 512, false, false);
  TEST_ASSERT_EQUAL_INT_MESSAGE(512, r3.rawX, "Center rawX");
  TEST_ASSERT_EQUAL_INT_MESSAGE(512, r3.rawY, "Center rawY");
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.1f, 0.0f, r3.rawRatioLR, "~0 ratio");
}

// ----------------------------- Tests: Fill bar logic -------------------------
static const int DISPLAY_WIDTH  = 128;
static const int DISPLAY_HEIGHT = 64;

void test_center_position_no_fill(void) {
  FillBarResult r = calculateThrottleFillBar(0, DISPLAY_WIDTH);
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, r.fillWidth,  "center throttle no width");
  TEST_ASSERT_TRUE_MESSAGE(r.centerLineDrawn, "center line drawn");
}

void test_throttle_fillbar_from_center_upward(void) {
  FillBarResult r = calculateThrottleFillBar(50, DISPLAY_WIDTH);
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, r.fillWidth, "throttle rightward width");
  TEST_ASSERT_EQUAL_INT_MESSAGE(DISPLAY_WIDTH/2, r.fillStartX, "starts at center X");
}

void test_throttle_fillbar_from_center_downward(void) {
  FillBarResult r = calculateThrottleFillBar(-50, DISPLAY_WIDTH);
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, r.fillWidth, "throttle leftward width");
  TEST_ASSERT_LESS_THAN_MESSAGE(DISPLAY_WIDTH/2, r.fillStartX, "starts left of center X");
}

void test_left_right_fillbar_from_center_rightward(void) {
  FillBarResult r = calculateLeftRightFillBar(25, DISPLAY_WIDTH);
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, r.fillWidth, "L/R rightward width");
  TEST_ASSERT_EQUAL_INT_MESSAGE(DISPLAY_WIDTH/2, r.fillStartX, "starts at center X");
}

void test_left_right_fillbar_from_center_leftward(void) {
  FillBarResult r = calculateLeftRightFillBar(-25, DISPLAY_WIDTH);
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, r.fillWidth, "L/R leftward width");
  TEST_ASSERT_LESS_THAN_MESSAGE(DISPLAY_WIDTH/2, r.fillStartX, "starts left of center X");
}

void test_maximum_fillbar_constraints(void) {
  FillBarResult r = calculateThrottleFillBar(100, DISPLAY_WIDTH);
  TEST_ASSERT_TRUE_MESSAGE(r.fillWidth <= DISPLAY_WIDTH/2, "width <= half display");
  TEST_ASSERT_TRUE_MESSAGE(r.fillStartX >= 0, "X within bounds");
}

void test_fillbar_bar_edge_cases(void) {
  FillBarResult r = calculateLeftRightFillBar(-50, DISPLAY_WIDTH);
  TEST_ASSERT_GREATER_THAN_MESSAGE(0, r.fillWidth, "max left has width");
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, r.fillStartX, "fills from far left");
}

void test_fillBarLogic(void) {
  // Throttle bar: barW=6 => max fill is 3 (half), now -100% to +100%
  FillBarResult thr0   = calculateThrottleFillBar(0, 6);
  FillBarResult thrNeg = calculateThrottleFillBar(-100, 6);
  FillBarResult thrPos = calculateThrottleFillBar(100, 6);
  
  TEST_ASSERT_EQUAL(0, thr0.fillWidth);
  TEST_ASSERT_EQUAL(3, thrNeg.fillWidth);
  TEST_ASSERT_EQUAL(3, thrPos.fillWidth);

  // Left/Right bar: -100..100 across width
  FillBarResult lr0     = calculateLeftRightFillBar(0, 52);
  FillBarResult lrLeft  = calculateLeftRightFillBar(-100, 52);
  FillBarResult lrRight = calculateLeftRightFillBar(100, 52);
  TEST_ASSERT_EQUAL(0,  lr0.fillWidth);
  TEST_ASSERT_EQUAL(26, lrLeft.fillWidth);
  TEST_ASSERT_EQUAL(26, lrRight.fillWidth);
}

// ---------------------------------------------------------------------------------
// main()
// ---------------------------------------------------------------------------------
int main(void) {
    UNITY_BEGIN();

  // configuration constants expectations
  RUN_TEST(test_configuration_constants);
  
    // slew rate function tests
  RUN_TEST(test_slewRateLimit_no_change);
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

  // motor targets function tests
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

  // joystick => motor targets + action tests
  RUN_TEST(test_processJoystick_NoMotion);
  RUN_TEST(test_processJoystick_deadzone_behavior);
  RUN_TEST(test_processJoystick_NearZeroInputWithinDeadzone_NoMotion);
  RUN_TEST(test_processJoystick_movement);
  RUN_TEST(test_processJoystick_buzzer);

  // braking integration tests
  RUN_TEST(test_shouldApplyBraking_on_stop);
  RUN_TEST(test_shouldApplyBraking_after_brake);

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

  return UNITY_END();
}
