#include "helpers.h"
#include <stdlib.h> // for abs
#include <Arduino.h> // for min, max, constrain


// Helper function for slew-rate control (formerly lambda in manualMode)
/// @brief Limits the rate of change between the current and target speed values to ensure smooth acceleration and deceleration.
/// @param current The current speed value.
/// @param target The desired target speed value.
/// @return The new speed value after applying the slew rate limit.
static int slewRateLimit(int current, int target)
{
  int uCurrent = abs(current), uTarget = abs(target);
  int uResult = 0;

  if (uTarget < MIN_MOTOR_SPEED && uCurrent < MIN_MOTOR_SPEED)
  { // floor to zero if both current and target are below MIN_MOTOR_SPEED
    uResult = 0;
  }
  else if (uCurrent < uTarget)
  { // Ramp up; only apply MIN_MOTOR_SPEED threshold if target speed is above MIN_MOTOR_SPEED
    int next = min(uCurrent + RAMP_STEP, uTarget);
    uResult = constrain(next, MIN_MOTOR_SPEED, MAX_SPEED);
  }
  else if (uCurrent > uTarget)
  { // Ramp down or stay at target
    int next = max(uCurrent - RAMP_STEP, uTarget);
    uResult = (uTarget < MIN_MOTOR_SPEED && next <= MIN_MOTOR_SPEED) ? 0 : constrain(next, MIN_MOTOR_SPEED, MAX_SPEED);
  }

  return (target < 0) ? -uResult : uResult; // negative if target < 0, else positive or zero
}


