#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include <RH_ASK.h>  // Include the RadioHead library
#include <SPI.h>     // Include the SPI library (required for RadioHead)
#include "MPU6050.h" // Changed to MPU6050 library which is more stable

// Pin Definitions
#define RIGHT_MOTOR_IN1 2   // L293D In  (pin 2)
#define RIGHT_MOTOR_IN2 3   // L293D In (pin 7)
#define RIGHT_MOTOR_EN 5    // L293D Enable1 (pin 1)
#define LEFT_MOTOR_IN1 4    // L293D In (pin 10)
#define LEFT_MOTOR_IN2 7    // L293D In (pin 15)
#define LEFT_MOTOR_EN 6     // L293D Enable2 (pin 9)

#define CS_PIN 7
#define BUZZER_PIN 10      // Active buzzer (as suggested)
#define MODE_BUTTON_PIN 6  // Push button for mode toggle -- CHANGED TO PIN 6 AS PER YOUR COMMENT
#define ULTRASONIC_TRIG 12 // HC-SR04 Trigger pin
#define ULTRASONIC_ECHO 13 // HC-SR04 Echo pin

// Constants
#define MAX_SPEED 255
#define MIN_MOTOR_SPEED 70 // minimum speed to avoid stalling, can be adjusted

#define LEFT_OFFSET +5     // Offset for left motor speed
#define RIGHT_OFFSET -5     // Offset for right motor speed

#define MIN_DISTANCE 30   // Minimum distance in cm before turning
#define TURN_TIME 800     // Time to turn in milliseconds
#define SCAN_INTERVAL 300 // Time between distance measurements

const int LOOP_DELAY_MS = 1;    // how often we run the main loop
const int RAMP_STEP     = 30;   // how many speed units we change per loop, less = smoother but slower response

// Initialize the RH_ASK driver - FM 433 MHz
// default: 2000 bps - must match transmitter speed
// slower = better range, but less responsive
RH_ASK rf_driver(1200);

// Structure for joystick data - MUST MATCH THE TRANSMITTER EXACTLY
struct JoystickData
{
  int xValue;
  int yValue;
  bool buttonPressed;
};

struct PackedData {
  uint16_t x : 10;  // 10 bits for X (0-1023)
  uint16_t y : 10;  // 10 bits for Y (0-1023)
  uint8_t btn : 1;   // Button state
  uint8_t crc : 5;   // Simple checksum (5 bits)
};

PackedData rcvData;


// Control variables
bool autoMode = false;            // Start in manual mode
unsigned long lastModeChange = 0; // Debounce button
unsigned long lastDistanceCheck = 0;
int lastButtonState = HIGH; // Previous state of button
bool buzzerEnabled = false;

// RF Signal variables (these will be updated directly from received JoystickData)
int joystickX = 512; // Center position (range: 0-1023)
int joystickY = 512; // Center position (range: 0-1023)
bool joystickButton = false;

// IMU setup - Using MPU6050 library (compatible with MPU6500/9250)
MPU6050 mpu;

float ax, ay, az;
float gx, gy, gz;

// Function prototypes (good practice) => must be declared here before usage below
// void checkModeButton();
void manualMode();
// void autonomousMode();
// void readIMU();
// long getDistance();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void stopMotors();
// void turnLeft(int angle);
// void turnRight(int angle);
void beep(int duration);
// void beepPattern(int beeps, int beepDuration);
bool readRFSignals(); 

String pad5(int val);
String pad5f(float val);

void setup()
{
  // Initialize serial communication for debugging
  Serial.begin(9600);
  Serial.println("RC Car Control System Starting...");

  // Initialize RF driver
  if (!rf_driver.init())
  {
    Serial.println("RF driver failed to initialize!");
  }
  else
  {
    Serial.println("RF driver initialized.");
  }

  // Initialize motor control pins
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_EN, OUTPUT);

  // Initialize ultrasonic sensor pins
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);

  // Initialize mode toggle button with internal pullup
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);

  // Initialize buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialize IMU - Using MPU6050 library (works with MPU6500/9250)
  Wire.begin();
  Wire.setClock(100000); // Start with slower I2C speed (100kHz)

  delay(500); // Longer delay for MPU to stabilize

  Serial.println("Starting I2C bus...");

  // Scan for I2C devices first
  Serial.println("Scanning for I2C devices...");
  byte error, address;
  int deviceCount = 0;

  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
      deviceCount++;
    }
  }

  if (deviceCount == 0)
  {
    Serial.println("No I2C devices found!");
  }
  else
  {
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" I2C device(s)");
  }

  // Initialize MPU6050/6500/9250
  Serial.println("Initializing MPU...");
  mpu.initialize();

  delay(100);

  // Test connection
  if (mpu.testConnection())
  {
    Serial.println("MPU connection successful!");

    // Configure similar to your working setup
    // Set sample rate divider for 100 Hz (similar to your ConfigSrd)
    mpu.setRate(9); // rate = 1000/(1+9) = 100Hz

    // Set accelerometer range
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4); // ±4g

    // Set gyroscope range
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500); // ±500°/s

    // Set digital low pass filter
    mpu.setDLPFMode(MPU6050_DLPF_BW_42); // 42Hz bandwidth

    Serial.print("Sample rate set to: ");
    Serial.print((int)(1000 / (9 + 1)));
    Serial.println(" Hz");

    Serial.println("MPU configured successfully");
    // Short beep to indicate successful initialization
    beep(100);
  }
  else
  {
    Serial.println("MPU connection failed!");
    Serial.println("Check wiring and power supply");
    // Error tone - long beep
    beep(1000);
  }

  // Stop all motors initially
  stopMotors();

  Serial.println("Setup complete");
}

void loop()
{
  // checkModeButton(); // Enable mode switching
  manualMode(); // This will be called inside the if/else for autoMode
  delay(LOOP_DELAY_MS);
  
  // // Control based on current mode
  // if (autoMode) {
  //   autonomousMode();
  // } else {
  //   manualMode();
  // }
}

// void checkModeButton()
// {
//   int reading = digitalRead(MODE_BUTTON_PIN);

//   // Debouncing
//   if (reading != lastButtonState)
//   {
//     lastButtonState = reading;

//     // If the button is pressed (LOW due to pull-up)
//     if (reading == LOW)
//     {
//       // Debounce timing - prevent multiple toggles
//       if (millis() - lastModeChange > 300)
//       { // Increased debounce to 300ms
//         autoMode = !autoMode;
//         lastModeChange = millis();

//         // Indicate mode change with beep
//         if (autoMode)
//         {
//           Serial.println("Switching to AUTO mode");
//           beep(300); // Long beep for auto mode
//         }
//         else
//         {
//           Serial.println("Switching to MANUAL mode");
//           beepPattern(2, 100); // Two short beeps for manual mode
//         }
//         // Stop motors immediately on mode change for safety
//         stopMotors();
//       }
//     }
//   }
// }

void manualMode()
{
  // 1) Read RF & joystick, invert X and Y (90 deg) then swap direction (180 deg)
  //    this is to account for joystick position + direction on physical controller.
  bool isRead = readRFSignals();
  bool skipSlewRate = false; // Flag to skip slew-rate control (e.g for sharp turns)

  int correctedX = 1023 - joystickY;  // 0-1023 inverted Y
  int correctedY = 1023 - joystickX;  // 0-1023 inverted X

  correctedX = 1023 - correctedX; // Invert X again to match expected direction

  static int leftSpeed       = 0;
  static int rightSpeed      = 0;
  static int targetLeft      = 0;
  static int targetRight     = 0;
  static int rfLostCounter   = 0;

  /**
   * Joystick center variance
   * X	    Y 	
   * 547	  523	  MIN
   * 565	  540	  MAX
   * 18	    17	  Variance
   */

  if (isRead)
  {
    // 2) Compute target speeds in some basic scenarios => FWD/BWD, L/R (sharp turn)
    //    constaint to MIN_MOTOR_SPEED, maxMotorSpeed and apply ramping
    const int joystickDeadzone = 18 ; // deadzone around center based on joystick variance
    const int forwardThreshold = 512 + joystickDeadzone; // center = 512, forward = 1023  (center can change with usage)
    const int backwardThreshold= 512 - joystickDeadzone; // center = 512, backward = 0
    

    float rawRatioLR = ((float)correctedX - 512.0) / 512.0; // ratio of left/right stick deflection, centered at 0
    rawRatioLR = constrain(rawRatioLR, -1.0, 1.0);

    // Quantize to nearest 0.n (n steps between -1 and +1)
    float quantizeStep = 0.10f;
    float steppedRatioLR = round(rawRatioLR / quantizeStep) * quantizeStep;

    // Serial.print(" | PRE-CHECK: Y="); Serial.print(correctedY);
    // Serial.print(" Y-512="); Serial.print(correctedY - 512.0);
    // Serial.print(" Ratio="); Serial.println(steppedRatioLR);

    // Serial.print(" | Sharp Turn: ");
    // Serial.print((abs(correctedY - joystickXCenter) < (0.20 * 1023.0) && fabs(steppedRatioLR) >= 0.15  ? "Yes" : "No"));
    // Serial.println();

    // in-place turn (L/R sharp turn) scenario (within n% of center )
    if (abs(correctedY - 512) < (0.20 * 1023.0) && fabs(steppedRatioLR) >= quantizeStep * 1.0)
    {
      // map to speed range (capped because both wheels will move in opposite directions)
      int sp = map((int)(fabs(steppedRatioLR) * 100), 0, 100, MIN_MOTOR_SPEED, MIN_MOTOR_SPEED + 5); 

      // -ve ratio => left turn 
      if (steppedRatioLR < 0) {
        targetRight = sp;
        
        // large ratio bias => pivot turn, reverse other wheel 
        if (steppedRatioLR <= -0.9) {
          targetLeft = -sp;
        }
      }
      // +ve ratio => right turn
      else if (steppedRatioLR > 0) {
        targetLeft = sp;

        // large ratio bias => pivot turn, reverse other wheel 
        if (steppedRatioLR >= +0.9) {
          targetRight = -sp;
        }
      }

      skipSlewRate = true; // Mark as skip slew-rate
    }
    // forward/backward scenario, constraints and ramping
    else if (correctedY > forwardThreshold)
    {
      int sp = map(constrain(correctedY, forwardThreshold, 1023), forwardThreshold, 1023, 0, MAX_SPEED);
      sp = constrain(sp, MIN_MOTOR_SPEED, MAX_SPEED);
      targetLeft  = sp;
      targetRight = sp;

      if (fabs(steppedRatioLR) >= 0.60) {
        // If joystick is tilted left/right, adjust speeds for turning
        targetLeft  *= (1.0 + steppedRatioLR / 2);
        targetRight *= (1.0 - steppedRatioLR / 2);
      }
      
      skipSlewRate = false; // Reset skip slew-rate flag
      // TODO: use IMU to adjust targetLeft and targetRight to account for drift if it keep doing so, test first
    }
    else if (correctedY < backwardThreshold)
    {
      int sp = map(constrain(correctedY, 0, backwardThreshold), backwardThreshold, 0, 0, MAX_SPEED);
      sp = constrain(sp, MIN_MOTOR_SPEED, MAX_SPEED);
      targetLeft  = -sp;
      targetRight = -sp;

      if (fabs(steppedRatioLR) >= 0.60) {
        // If joystick is tilted left/right, adjust speeds for turning
        targetLeft  *= (1.0 + steppedRatioLR / 2);
        targetRight *= (1.0 - steppedRatioLR / 2);
      }
      
      skipSlewRate = false; // Reset skip slew-rate flag
    }
    else
    { // deadzone → targets zero
      targetLeft  = 0;
      targetRight = 0;
    }

    if (!skipSlewRate) {
      // 3) Apply slew-rate (ramp-up/down) to move leftSpeed → targetLeft
      auto slew = [&](int current, int target){
        int uCurrent = abs(current), uTarget = abs(target);
        int uResult = 0;
        if (uCurrent < uTarget)  uResult = constrain(min(uCurrent + RAMP_STEP, uTarget), MIN_MOTOR_SPEED, MAX_SPEED);    // ramp-up forwards
        else uResult = uTarget; // ramp-down or stay at target

        if (target < 0) return -uResult;      // if target is negative, return negative speed
        else if (target > 0) return uResult;  // if target is positive, return positive speed
        else return uResult;                  // if target is zero, return zero
      };

      if (abs(correctedY - 512) < joystickDeadzone) {
        targetLeft = 0;
        targetRight = 0;
      }

      leftSpeed  = slew(leftSpeed,  targetLeft);
      rightSpeed = slew(rightSpeed, targetRight);
    }
    else {
      // If we skipped slew-rate, just set to target immediately
      leftSpeed  = targetLeft;
      rightSpeed = targetRight;
    }



    static int prevLeftSpeed = 0;
    static int prevRightSpeed = 0;
    static bool brakingApplied = false;

    // 4) Set motor speeds, apply braking if needed7
    if (targetLeft == 0 && targetRight == 0 && 
        (abs(prevLeftSpeed) > 0 || abs(prevRightSpeed) > 0)) 
    {
        if (!brakingApplied) {
            // Apply reverse pulse for braking
            setMotorSpeeds(-prevLeftSpeed/4, -prevRightSpeed/4);
            delay(15);
            brakingApplied = true;
            
            // Immediately zero speeds after braking
            leftSpeed = 0;
            rightSpeed = 0;
        }
    } else {
        brakingApplied = false;
    }

    // Only set motor speeds if braking was not just applied
    if (!brakingApplied) {
        // Apply minimum speed threshold only at output stage
        int outputLeft = (abs(leftSpeed) >= MIN_MOTOR_SPEED) ? leftSpeed : 0;
        int outputRight = (abs(rightSpeed) >= MIN_MOTOR_SPEED) ? rightSpeed : 0;
        setMotorSpeeds(outputLeft, outputRight);
    }
    
    prevLeftSpeed = leftSpeed;
    prevRightSpeed = rightSpeed;

    // DEBUG
  Serial.print("Dir: ");
  if (leftSpeed > 0 && rightSpeed > 0) {
    Serial.print("FORWD");
  } else if (leftSpeed < 0 && rightSpeed < 0) {
    Serial.print("BCKWD");
  } else if (leftSpeed == 0 && rightSpeed == 0) {
    Serial.print("STOP ");
  } else if (leftSpeed < rightSpeed) {
    Serial.print("LEFT ");
  } else if (rightSpeed < leftSpeed) {
    Serial.print("RIGHT");
  } else {
    Serial.print("MIXED"); // not ok good
  }

    Serial.print(" Q LR: ");
    Serial.print(pad5f(steppedRatioLR)); // stepped ratio
    Serial.print(" (");
    Serial.print(pad5f(rawRatioLR));
    Serial.print(")");

    Serial.print(" Targ Spd: ");
    Serial.print(pad5(targetLeft));
    Serial.print("");
    Serial.print(pad5(targetRight));

    Serial.print(" Curr Spd: ");
    Serial.print(pad5(leftSpeed));
    Serial.print(" | ");
    Serial.print(pad5(rightSpeed));

    Serial.print(" Slew: ");
    Serial.print(!skipSlewRate ? "Y" : "N");
    Serial.print(" | ");
    Serial.print("Brk: ");
    Serial.print(brakingApplied ? "Y" : "N");

    Serial.print(" XY: ");
    Serial.print(pad5(correctedX));
    Serial.print(" | ");
    Serial.print(pad5(correctedY));

    Serial.println();

    rfLostCounter = 0;
  }
  else
  {
    // RF-lost ramp-down to avoid judder, signal considered lost after ~0.5s
    if (rfLostCounter++ * LOOP_DELAY_MS > 440 || skipSlewRate)
    {
      leftSpeed  = 0;
      rightSpeed = 0;
      
      setMotorSpeeds(leftSpeed, rightSpeed);
      rfLostCounter = 0;
      Serial.println("RF SIGNAL LOSS: STOPPING...");  
    }
  }

  // buzzer beep boop
  if (joystickButton && !buzzerEnabled)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerEnabled = true;
  }
  else if (!joystickButton && buzzerEnabled)
  {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerEnabled = false;
  }
}

// Simple CRC calculation (XOR-based)
uint8_t calcCRC(uint16_t x, uint16_t y, bool btn) {
  return (x ^ y ^ btn) & 0x1F; // Use 5-bit CRC
}

/// @brief 
/// Reads RF signals from the joystick transmitter and updates global joystick variables.
/// @return 
/// Returns true if data was successfully received and updated, false otherwise.
bool readRFSignals()
{
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);

  if (rf_driver.recv(buf, &buflen)) {
    // Copy to struct
    memcpy(&rcvData, buf, min(buflen, sizeof(rcvData)));
    
    // Verify CRC
    if (rcvData.crc == calcCRC(rcvData.x, rcvData.y, rcvData.btn)) {
      // Valid data!
      joystickX = rcvData.x;
      joystickY = rcvData.y;
      joystickButton = rcvData.btn;

      return true;

    }
  }

  return false;
}


// /// @brief 
// /// WIP
// void autonomousMode()
// {
//   // Get distance from ultrasonic sensor
//   long distance = getDistance();

//   // Use IMU to help with navigation
//   readIMU();

//   // Simple obstacle avoidance logic
//   if (distance < MIN_DISTANCE)
//   {
//     // Obstacle detected - stop and determine turn direction
//     stopMotors();
//     delay(200);

//     // Use IMU data to decide which way to turn
//     // For example, turn to the side with less tilt or acceleration
//     if (ax > 0)
//     { // Sample logic - can be refined
//       turnRight(180);
//     }
//     else
//     {
//       turnLeft(180);
//     }
//   }
//   else
//   {
//     // No obstacle, move forward with slight adjustments based on IMU
//     // This can help keep the car moving straight
//     int baseSpeed = 180; // 70% of max speed for better control
//     int leftSpeed = baseSpeed;
//     int rightSpeed = baseSpeed;

//     // Slight steering correction based on gyro data
//     if (gz > 100)
//     { // Drifting right, correct left
//       leftSpeed += 20;
//       rightSpeed -= 20;
//     }
//     else if (gz < -100)
//     { // Drifting left, correct right
//       leftSpeed -= 20;
//       rightSpeed += 20;
//     }

//     // Set motor speeds
//     setMotorSpeeds(leftSpeed, rightSpeed);
//   }
// }

// /// WIP lib issues
// void readIMU()
// {
//   // Read the sensor values (similar to your working approach)

//   // Get accelerometer values (in g's, convert to m/s²)
//   int16_t ax_raw, ay_raw, az_raw;
//   mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);

//   // Convert to m/s² (assuming ±4g range: 32768 = 4g = 39.24 m/s²)
//   ax = (float)ax_raw / 32768.0 * 4.0 * 9.81;
//   ay = (float)ay_raw / 32768.0 * 4.0 * 9.81;
//   az = (float)az_raw / 32768.0 * 4.0 * 9.81;

//   // Get gyroscope values (in degrees/s, convert to rad/s)
//   int16_t gx_raw, gy_raw, gz_raw;
//   mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);

//   // Convert to rad/s (assuming ±500°/s range: 32768 = 500°/s)
//   gx = (float)gx_raw / 32768.0 * 500.0 * PI / 180.0;
//   gy = (float)gy_raw / 32768.0 * 500.0 * PI / 180.0;
//   gz = (float)gz_raw / 32768.0 * 500.0 * PI / 180.0;

//   // Print IMU values for debugging (can be removed in final code)
//   // You might want to print these less frequently or on demand
//   // Serial.print("IMU: ");
//   // Serial.print(ax); Serial.print("\t");
//   // Serial.print(ay); Serial.print("\t");
//   // Serial.print(az); Serial.print("\t");
//   // Serial.print(gx); Serial.print("\t");
//   // Serial.print(gy); Serial.print("\t");
//   // Serial.println(gz);
// }

// long getDistance()
// {
//   // Trigger ultrasonic pulse
//   digitalWrite(ULTRASONIC_TRIG, LOW);
//   delayMicroseconds(2);
//   digitalWrite(ULTRASONIC_TRIG, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(ULTRASONIC_TRIG, LOW);

//   // Read the echo time
//   long duration = pulseIn(ULTRASONIC_ECHO, HIGH);

//   // Calculate distance in cm
//   // Speed of sound is 343m/s or 0.0343cm/µs
//   // Time is round-trip, so divide by 2
//   long distance = (duration * 0.0343) / 2;

//   return distance;
// }

/// @brief  
/// Sets the speeds of the left and right motors based on the provided speed values.
/// Speeds are constrained to the range of -MAX_SPEED to MAX_SPEED.
/// @param leftSpeed 
/// The speed for the left motor. Positive values move the motor backward, negative values move it forward (RWD).
/// @param rightSpeed 
/// The speed for the right motor. Positive values move the motor backward, negative values move it forward (RWD).
void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
  // Constrain speeds to valid range
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  if ((abs(leftSpeed) >= MIN_MOTOR_SPEED || abs(rightSpeed) >= MIN_MOTOR_SPEED) && 
      ((leftSpeed > 0 && rightSpeed > 0) || (leftSpeed < 0 && rightSpeed < 0))){
    // Apply offsets to adjust for motor differences (applies only if both wheels in motion and in same direction)
    leftSpeed += leftSpeed > 0 ? LEFT_OFFSET : -LEFT_OFFSET;
    rightSpeed += rightSpeed > 0 ? RIGHT_OFFSET : -RIGHT_OFFSET;

    // if any of the speeds is below the minimum, but the other is above, creep both up until threshold reached for both
    if (abs(leftSpeed) < MIN_MOTOR_SPEED || abs(rightSpeed) < MIN_MOTOR_SPEED){
      int diff = MIN_MOTOR_SPEED - min(abs(leftSpeed), abs(rightSpeed));  // smallest one is the one below threshold

      leftSpeed += leftSpeed > 0 ? diff : -diff;
      rightSpeed += rightSpeed > 0 ? diff : -diff;
    }

    // Maintain the offset: keep the difference between left and right speeds constant after adjustments
    int diff = leftSpeed - rightSpeed;

    // Constrain leftSpeed and rightSpeed, but preserve their difference
    if (leftSpeed > MAX_SPEED) {
      leftSpeed = MAX_SPEED;
      rightSpeed = leftSpeed - diff;
    } else if (leftSpeed < -MAX_SPEED) {
      leftSpeed = -MAX_SPEED;
      rightSpeed = leftSpeed - diff;
    }

    if (rightSpeed > MAX_SPEED) {
      rightSpeed = MAX_SPEED;
      leftSpeed = rightSpeed + diff;
    } else if (rightSpeed < -MAX_SPEED) {
      rightSpeed = -MAX_SPEED;
      leftSpeed = rightSpeed + diff;
    }

    // Final clamp in case both exceeded in opposite directions (failsafe)
    leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
  }

  // Set left motor direction and speed (RWD: positive = backward, negative = forward)
  if (leftSpeed >= 0)
  {
    // Backward (RWD)
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    analogWrite(LEFT_MOTOR_EN, leftSpeed);
  }
  else
  {
    // Forward (RWD)
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    analogWrite(LEFT_MOTOR_EN, -leftSpeed);
  }

  // Set right motor direction and speed (RWD: positive = backward, negative = forward)
  if (rightSpeed >= 0)
  {
    // Backward (RWD)
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    analogWrite(RIGHT_MOTOR_EN, rightSpeed);
  }
  else
  {
    // Forward (RWD)
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    analogWrite(RIGHT_MOTOR_EN, -rightSpeed);
  }
}

/// @brief 
/// Stops both motors by setting their control pins to LOW and disabling PWM.
void stopMotors()
{
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_EN, 0);

  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(RIGHT_MOTOR_EN, 0);
}

// void turnLeft(int angle)
// {
//   // Simple timed turn - can be refined with IMU for more precise angles
//   stopMotors();
//   delay(100);

//   // Tank turn left - right wheel forward, left wheel backward
//   digitalWrite(LEFT_MOTOR_IN1, LOW);
//   digitalWrite(LEFT_MOTOR_IN2, HIGH);
//   analogWrite(LEFT_MOTOR_EN, MAX_SPEED * 0.7);

//   digitalWrite(RIGHT_MOTOR_IN1, HIGH);
//   digitalWrite(RIGHT_MOTOR_IN2, LOW);
//   analogWrite(RIGHT_MOTOR_EN, MAX_SPEED * 0.7);

//   // Time-based turn - approximately 90 degrees
//   delay(TURN_TIME * angle / 90);

//   stopMotors();
// }

// void turnRight(int angle)
// {
//   // Simple timed turn - can be refined with IMU for more precise angles
//   stopMotors();
//   delay(100);

//   // Tank turn right - left wheel forward, right wheel backward
//   digitalWrite(LEFT_MOTOR_IN1, HIGH);
//   digitalWrite(LEFT_MOTOR_IN2, LOW);
//   analogWrite(LEFT_MOTOR_EN, MAX_SPEED * 0.7);

//   digitalWrite(RIGHT_MOTOR_IN1, LOW);
//   digitalWrite(RIGHT_MOTOR_IN2, HIGH);
//   analogWrite(RIGHT_MOTOR_EN, MAX_SPEED * 0.7);

//   // Time-based turn - approximately 90 degrees
//   delay(TURN_TIME * angle / 90);

//   stopMotors();
// }

void beep(int duration)
{
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  digitalWrite(BUZZER_PIN, LOW);
}

// void beepPattern(int beeps, int beepDuration)
// {
//   for (int b = 0; b < beeps; b++)
//   { // Changed loop variable from 'i' to 'b' to avoid conflict with global 'i'
//     digitalWrite(BUZZER_PIN, HIGH);
//     delay(beepDuration);
//     digitalWrite(BUZZER_PIN, LOW);
//     delay(beepDuration);
//   }
// }


String pad5(int val) {
  char buf[7];
  snprintf(buf, sizeof(buf), "%5d", val);
  return String(buf);
}

String pad5f(float val) {
  char buf[7]; // Make sure this buffer is large enough for your output
  // dtostrf(float_value, total_width, decimal_places, char_array);
  dtostrf(val, 5, 2, buf); // 5 total width, 2 decimal places
  return String(buf);
}

