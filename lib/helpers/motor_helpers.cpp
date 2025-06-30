#include "helpers.h"
#include <stdlib.h>
#include <algorithm> // for std::min, std::max

// For native builds, define constrain if not available
#ifndef ARDUINO
template<typename T>
T constrain(T val, T min_val, T max_val) {
    return std::min(std::max(val, min_val), max_val);
}
#endif

// Helper function for slew-rate control (formerly lambda in manualMode)
/// @brief Limits the rate of change between the current and target speed values to ensure smooth acceleration and deceleration.
/// @param current The current speed value.
/// @param target The desired target speed value.
/// @return The new speed value after applying the slew rate limit.
int slewRateLimit(int current, int target)
{
    int uCurrent = abs(current), uTarget = abs(target);
    int uResult = 0;
    if (uCurrent < uTarget) {
        int next = std::min(uCurrent + RAMP_STEP, uTarget);
        uResult = (next < MIN_MOTOR_SPEED) ? MIN_MOTOR_SPEED : constrain(next, MIN_MOTOR_SPEED, MAX_SPEED);
    } else if (uCurrent > uTarget) {
        int next = std::max(uCurrent - RAMP_STEP, uTarget);
        uResult = (uTarget < MIN_MOTOR_SPEED && next <= MIN_MOTOR_SPEED) ? 0 : constrain(next, MIN_MOTOR_SPEED, MAX_SPEED);
    } else {
        uResult = uCurrent;
    }
    return (target < 0) ? -uResult : uResult;
}


