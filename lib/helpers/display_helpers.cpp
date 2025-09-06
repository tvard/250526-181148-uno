
#include "display_helpers.h"


#include "display_helpers.h"

// Horizontal throttle fill bar calculation
FillBarResult calculateThrottleFillBar(int throttlePercent, int displayWidth) {
    FillBarResult result = {-1, -1, false};
    int centerX = displayWidth / 2;
    if (throttlePercent == 0) {
        result.fillWidth = 0;
        result.centerLineDrawn = true;
    } else if (throttlePercent > 0) {
        // Forward throttle: positive values
        int fillPixels = throttlePercent * centerX / 100;
        fillPixels = (fillPixels < 0) ? 0 : ((fillPixels > centerX) ? centerX : fillPixels);
        result.fillStartX = centerX;
        result.fillWidth = fillPixels;
        if (result.fillStartX + fillPixels > displayWidth) result.fillWidth = displayWidth - result.fillStartX;
    } else {
        // Reverse throttle: negative values
        int fillPixels = (-throttlePercent) * centerX / 100;
        fillPixels = (fillPixels < 0) ? 0 : ((fillPixels > centerX) ? centerX : fillPixels);
        result.fillStartX = centerX - fillPixels;
        result.fillWidth = fillPixels;
        if (result.fillStartX < 0) {
            result.fillWidth += result.fillStartX;
            result.fillStartX = 0;
            if (result.fillWidth < 0) result.fillWidth = 0;
        }
    }
    return result;
}

// Horizontal left/right fill bar calculation
FillBarResult calculateLeftRightFillBar(int leftRightPercent, int displayWidth) {
    FillBarResult result = {-1, -1, false};
    int centerX = displayWidth / 2;
    if (leftRightPercent == 0) {
        result.fillWidth = 0;
        result.centerLineDrawn = true;
    } else if (leftRightPercent > 0) {
        int fillPixels = leftRightPercent * centerX / 50;
        fillPixels = (fillPixels < 0) ? 0 : ((fillPixels > centerX) ? centerX : fillPixels);
        result.fillStartX = centerX;
        result.fillWidth = fillPixels;
        if (result.fillStartX + fillPixels > displayWidth) result.fillWidth = displayWidth - result.fillStartX;
    } else {
        int fillPixels = (-leftRightPercent) * centerX / 50;
        fillPixels = (fillPixels < 0) ? 0 : ((fillPixels > centerX) ? centerX : fillPixels);
        result.fillStartX = centerX - fillPixels;
        result.fillWidth = fillPixels;
        if (result.fillStartX < 0) {
            result.fillWidth += result.fillStartX;
            result.fillStartX = 0;
            if (result.fillWidth < 0) result.fillWidth = 0;
        }
    }
    return result;
}
