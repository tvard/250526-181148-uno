#include "2WD_RC_TRANSMITTER_logic.h"
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

extern int16_t xMin, xMax, yMin, yMax;
extern uint16_t xCenter, yCenter;

int calculateThrottlePercent(int y) {
    // Debug print
    // printf("[DEBUG] y input: %d, yMin: %d, yCenter: %d, yMax: %d\n", y, yMin, yCenter, yMax);

    if (y < yMin) {
        y = yMin;
    }
    if (y > yMax) {
        y = yMax;
    }

    float throttlePercent = ((float)(y - yMin) / (float)(yCenter - yMin)) * 100.0f - 100.0f;


    // Adjust deadzone logic
    int deadzoneThreshold = 8; // Fixed deadzone threshold
    if (abs(y - (int)yCenter) <= deadzoneThreshold) {
        throttlePercent = 0.0f;
    }

    // Guarantee full range mapping
    if (y <= yMin + 1) throttlePercent = -100.0f;
    if (y >= yMax - 1) throttlePercent = 100.0f;

    // Clamp output
    if (throttlePercent > 100.0f) throttlePercent = 100.0f;
    if (throttlePercent < -100.0f) throttlePercent = -100.0f;

    return (int)round(throttlePercent);
}

int calculateLeftRightPercent(int x) {
    if (x < xMin) {
        x = xMin;
    }
    if (x > xMax) {
        x = xMax;
    }

    float leftRightPercent;
    if (x > xCenter) {
        leftRightPercent = ((float)(x - xCenter) / (float)(xMax - xCenter)) * 100.0f;
    } else {
        leftRightPercent = ((float)(x - xMin) / (float)(xCenter - xMin)) * 100.0f - 100.0f;
    }

    // Adjust deadzone logic
    if (abs(x - (int)xCenter) < 10) {
        leftRightPercent = 0.0f;
    }

    // Ensure full range mapping
    if (x == xMin) leftRightPercent = -100.0f;
    if (x == xMax) leftRightPercent = 100.0f;

    return (int)round(leftRightPercent);
}
