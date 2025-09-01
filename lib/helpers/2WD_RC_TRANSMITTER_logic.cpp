#include "2WD_RC_TRANSMITTER_logic.h"
#include <stdint.h>
#include <math.h>
#include <stdlib.h>

extern uint16_t xMin, xMax, xCenter;
extern uint16_t yMin, yMax, yCenter;

int calculateThrottlePercent(int y) {
    if (y < yMin) y = yMin;
    if (y > yMax) y = yMax;
    float throttlePercent = ((float)(y - yMin) / (float)(yMax - yMin)) * 100.0f;
    bool nearCenterRaw = abs(y - (int)yCenter) < 15;
    bool nearCenterPercent = abs(throttlePercent - 50.0f) <= 2.0f;
    if (nearCenterRaw && nearCenterPercent) {
        throttlePercent = 50.0f;
    }
    return (int)round(throttlePercent);
}

int calculateLeftRightPercent(int x) {
    float leftRightPercent;
    if (x > xCenter) {
        leftRightPercent = ((float)(x - xCenter) / (float)(xMax - xCenter)) * 100.0f;
    } else {
        leftRightPercent = ((float)(x - xMin) / (float)(xCenter - xMin)) * 100.0f - 100.0f;
    }
    bool nearCenterRaw = abs(x - (int)xCenter) < 15;
    bool nearCenterPercent = abs(leftRightPercent) <= 2.0f;
    if (nearCenterRaw && nearCenterPercent) {
        leftRightPercent = 0.0f;
    }
    // Clamp output
    if (leftRightPercent > 100.0f) leftRightPercent = 100.0f;
    if (leftRightPercent < -100.0f) leftRightPercent = -100.0f;
    return (int)round(leftRightPercent);
}
