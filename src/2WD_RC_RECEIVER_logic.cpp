// Shared calculation logic for receiver
#include <stdint.h>
#include <cmath>

extern uint16_t xMin, xMax, yMin, yMax;
extern uint16_t xCenter, yCenter;

int calculateThrottlePercent(int y) {
    // Clamp input to valid range
    if (y < yMin) y = yMin;
    if (y > yMax) y = yMax;
    float throttlePercent = 50.0f;
    float denom_up = (float)(yMax - yCenter);
    float denom_down = (float)(yCenter - yMin);
    if (y > yCenter && denom_up > 0.0f) {
        throttlePercent = 50.0f + ((float)(y - yCenter) / denom_up) * 50.0f;
    } else if (y <= yCenter && denom_down > 0.0f) {
        throttlePercent = ((float)(y - yMin) / denom_down) * 50.0f;
    }
    bool nearCenterRaw = std::abs(y - (int)yCenter) < 15;
    bool nearCenterPercent = std::fabs(throttlePercent - 50.0f) <= 2.0f;
    if (nearCenterRaw && nearCenterPercent) {
        throttlePercent = 50.0f;
    }
    // Clamp output to [0, 100]
    if (throttlePercent < 0.0f) throttlePercent = 0.0f;
    if (throttlePercent > 100.0f) throttlePercent = 100.0f;
    return (int)round(throttlePercent);
}

int calculateLeftRightPercent(int x) {
    float leftRightPercent = 0.0f;
    float denom_right = (float)(xMax - xCenter);
    float denom_left = (float)(xCenter - xMin);
    // Clamp output for out-of-bounds inputs
    if (x < xMin) {
        leftRightPercent = -100.0f;
    } else if (x > xMax) {
        leftRightPercent = 100.0f;
    } else if (x > xCenter && denom_right > 0.0f) {
        leftRightPercent = ((float)(x - xCenter) / denom_right) * 100.0f;
    } else if (x <= xCenter && denom_left > 0.0f) {
        leftRightPercent = ((float)(x - xMin) / denom_left) * 100.0f - 100.0f;
    }
    bool nearCenterRaw = std::abs(x - (int)xCenter) < 15;
    bool nearCenterPercent = std::fabs(leftRightPercent) <= 2.0f;
    if (nearCenterRaw && nearCenterPercent) {
        leftRightPercent = 0.0f;
    }
    // Clamp output to [-100, 100]
    if (leftRightPercent < -100.0f) leftRightPercent = -100.0f;
    if (leftRightPercent > 100.0f) leftRightPercent = 100.0f;
    return (int)round(leftRightPercent);
}
