// Shared calculation logic for transmitter
#include <stdint.h>
#include <math.h>

extern uint16_t xMin, xMax, yMin, yMax;
extern uint16_t xCenter, yCenter;

int calculateThrottlePercent(int y) {
    float throttlePercent;
    if (y > yCenter) {
        throttlePercent = 50.0f + ((float)(y - yCenter) / (float)(yMax - yCenter)) * 50.0f;
    } else {
        throttlePercent = ((float)(y - yMin) / (float)(yCenter - yMin)) * 50.0f;
    }
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
    return (int)round(leftRightPercent);
}
