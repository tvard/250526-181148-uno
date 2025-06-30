#pragma once

#include <Arduino.h>  // for String type

// Constants
const int MAX_SPEED        = 255;
const int MIN_MOTOR_SPEED  = 70; // minimum speed to avoid stalling, can be adjusted

const int LEFT_OFFSET      = +5;  // Offset for left motor speed
const int RIGHT_OFFSET     = -5;  // Offset for right motor speed

const int MIN_DISTANCE     = 30;   // Minimum distance in cm before turning
const int TURN_TIME        = 800;     // Time to turn in milliseconds
const int SCAN_INTERVAL    = 300; // Time between distance measurements

const int LOOP_DELAY_MS = 1; // how often we run the main loop
const int RAMP_STEP = 30;    // how many speed units we change per loop, less = smoother but slower response


// function declarations
int slewRateLimit(int current, int target);
String pad5(int val);
String pad5f(float val);
