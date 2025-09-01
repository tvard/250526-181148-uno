#pragma once

struct FillBarResult {
    int fillStartX;
    int fillWidth;
    // int fillStartY;
    // int fillHeight;
    bool centerLineDrawn;
};

FillBarResult calculateThrottleFillBar(int throttlePercent, int displayHeight = 64);
FillBarResult calculateLeftRightFillBar(int leftRightPercent, int displayWidth = 128);
