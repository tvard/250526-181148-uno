/*
 * Test-specific implementation of calculation functions
 * This allows us to test the actual calculation logic without the full firmware
 */

#include <cstdint>

// Test-controlled variables (will be set by test functions)
uint16_t xMin = 200, xMax = 800, yMin = 200, yMax = 800;
uint16_t xCenter = 512, yCenter = 512;

// Arduino compatibility
#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define map(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

// Calculation functions (copied from main firmware)
int calculateThrottlePercent(int y) {
  int throttlePercent;
  
  // First, calculate the raw percentage mapping
  if (y > yCenter) {
    // Forward: map from center to max -> 50% to 100%
    throttlePercent = map(y, yCenter, yMax, 50, 100);
  } else {
    // Reverse: map from min to center -> 0% to 50%
    throttlePercent = map(y, yMin, yCenter, 0, 50);
  }
  
  // Apply percentage-based deadzone (±3% around center = 47%-53%)
  if (abs(throttlePercent - 50) <= 3) {
    throttlePercent = 50; // Force to center if within 47%-53%
  }
  
  return constrain(throttlePercent, 0, 100);
}

int calculateLeftRightPercent(int x) {
  int leftRightPercent;
  
  // First, calculate the raw percentage mapping
  if (x > xCenter) {
    // Right: map from center to max -> 0% to +100%
    leftRightPercent = map(x, xCenter, xMax, 0, 100);
  } else {
    // Left: map from min to center -> -100% to 0%
    leftRightPercent = map(x, xMin, xCenter, -100, 0);
  }
  
  // Apply percentage-based deadzone (±3% around center)
  if (abs(leftRightPercent) <= 3) {
    leftRightPercent = 0; // Force to center if within ±3%
  }
  
  return constrain(leftRightPercent, -100, 100);
}
