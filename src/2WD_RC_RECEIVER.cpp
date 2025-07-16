// For compiling with Unity (unit testing framework), you may need to mock or guard hardware-specific includes.
// Example for Unity compatibility:

// For PlatformIO/Arduino
#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include <RH_ASK.h>  // RadioHead library for RF
#include <SPI.h>     // SPI required by RadioHead
#include "MPU6050.h" // MPU6050 IMU library
#include "helpers.h"
#include "pitches.h"

// Pin Definitions
const int RIGHT_MOTOR_IN1 = 2;
const int RIGHT_MOTOR_IN2 = 3;
const int RIGHT_MOTOR_EN = 5;
const int LEFT_MOTOR_IN1 = 4;
const int LEFT_MOTOR_IN2 = 7;
const int LEFT_MOTOR_EN = 6;

const int BUZZER_PIN = 8;
const int MODE_BUTTON_PIN = 6;
const int ULTRASONIC_TRIG = 12;
const int ULTRASONIC_ECHO = 13;

// Initialize the RH_ASK driver - FM 433 MHz
// Default: 2000 bps - must match transmitter speed.  Slower = better range / integrity at range, but less responsive
// Default Pins: RX on pin 11, TX on pin 12 (RH_ASK, change as needed in ctor)
RH_ASK rf_driver(1200);

// Structure for joystick data - MUST MATCH THE TRANSMITTER EXACTLY
struct JoystickData
{
  int xValue;
  int yValue;
  bool buttonPressed;
};

struct PackedData
{
  uint16_t x : 10; // 10 bits for X (0-1023)
  uint16_t y : 10; // 10 bits for Y (0-1023)
  uint8_t btn : 1; // Button state
  uint8_t crc : 5; // Simple checksum (5 bits)
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
void manualModeSerialPrint(int leftSpeed, int rightSpeed, JoystickProcessingResult &js, ManualModeOutputs &out, bool brakingApplied);
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
static int slewRateLimit(int current, int target);

struct EntertainerState {
    int noteIndex = 0;
    unsigned long phaseStart = 0;
    int phase = 0; // 0: play note, 1: wait for note to finish, 2: pause between notes
    bool playing = false; // Whether the entertainer is currently playing
};

static EntertainerState entertainerState;

bool playEntertainerStep(EntertainerState &state, bool shouldInterrupt) {
  // ——————————————————————————————————————————————————————————————
  // 1) Melody data

// Frequencies (Hz) for each pitch:
//   G4 = 392, C5 = 523, E5 = 659, G5 = 784, C6 = 1047, E6 = 1319
static const int melody[] = {
  NOTE_G4, NOTE_E5, NOTE_C5, NOTE_E5,  // pickup + beat 1
  NOTE_G5, NOTE_G5,                    // end of bar 1
  NOTE_E5, NOTE_G5, NOTE_E5, NOTE_C5,  // bar 2 first four eighths
  NOTE_E5, NOTE_G5                    // bar 2 next two eighths (~15 s mark)
};

// Duration type: 8 = eighth note, 4 = quarter note
static const int noteType[] = {
     8,     8,     8,     8,
     4,     4,
     8,     8,     8,     8,
     8,     8
};

  const int totalNotes = sizeof(melody) / sizeof(melody[0]); // Total number of notes in the melody
  const int tempo = 180;
  const unsigned long baseNote = (60000UL / tempo) * 4;  // duration of a whole note

  // Serial.print("Playing melody (TOTAL: " + (String)totalNotes + "): ");
  // Serial.print(melody[state.noteIndex]);
  // Serial.print(" (" + (String)state.noteIndex + ")" );
  // Serial.println();

  // ——————————————————————————————————————————————————————————————
  // 2) Handle interruptions or end‐of‐melody
  if (shouldInterrupt || state.noteIndex >= totalNotes) {
    noTone(BUZZER_PIN);
    state.noteIndex  = 0;
    state.phase      = 0;   // reset to “play” phase
    state.phaseStart = 0;
    return false;
  }

  unsigned long now = millis();

  // ——————————————————————————————————————————————————————————————
  // 3) Compute this note’s on‑time and pause‑time
  unsigned long noteOnTime   = baseNote / noteType[state.noteIndex];
  unsigned long pauseTime    = noteOnTime / 4;  // 25% rest between notes

  switch (state.phase) {
    case 0: // start the note
      tone(BUZZER_PIN, melody[state.noteIndex]);
      state.phaseStart = now;
      state.phase      = 1;
      return true;

    case 1: // waiting for note‐on to finish
      if (now - state.phaseStart >= noteOnTime) {
        noTone(BUZZER_PIN);
        state.phaseStart = now;
        state.phase      = 2;
      }
      return true;

    case 2: // pause between notes
      if (now - state.phaseStart >= pauseTime) {
        state.noteIndex++;
        state.phase = 0;
      }
      return true;
  }

  return false; // should never hit
}




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
    beep(100);
    beep(100);
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
    beep(100);
  }
  else
  {
    Serial.println("MPU connection failed!");
    Serial.println("Check wiring and power supply");
    // Error tone - long beep
    // beep(1000);
  }

  // Stop all motors initially
  stopMotors();

  // beep buzzer to indicate setup completion
  while (playEntertainerStep(entertainerState, false)) {
    // no other tasks will run until the melody completes
    delay(LOOP_DELAY_MS);
  }

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
  static int leftSpeed = 0, rightSpeed = 0, prevLeftSpeed = 0, prevRightSpeed = 0;
  static int rfLostCounter = 0;
  static bool brakingApplied = false;

  bool isRead = readRFSignals();

  JoystickProcessingResult js = processJoystick(joystickX, joystickY, joystickButton);

  if (isRead)
  {
    ManualModeInputs in = {
        js, leftSpeed, rightSpeed, prevLeftSpeed, prevRightSpeed};

    ManualModeOutputs out = manualModeStep(in);

    leftSpeed = out.leftSpeed;
    rightSpeed = out.rightSpeed;
    brakingApplied = out.brakingApplied;

    setMotorSpeeds(out.outputLeft, out.outputRight);

    prevLeftSpeed = leftSpeed;
    prevRightSpeed = rightSpeed;

    // Buzzer
    // entertainerState.playing = playEntertainerStep(entertainerState, buzzerEnabled);

    manualModeSerialPrint(leftSpeed, rightSpeed, js, out, brakingApplied);
    Serial.println();
  }
  else if (rfLostCounter++ * LOOP_DELAY_MS > 440) // RF-lost ramp-down to avoid judder, signal considered lost after ~0.5s
  {
    leftSpeed = 0;
    rightSpeed = 0;

    setMotorSpeeds(leftSpeed, rightSpeed);
    rfLostCounter = 0;
    Serial.println("RF SIGNAL LOSS: STOPPING...");
  }
}

void manualModeSerialPrint(int leftSpeed, int rightSpeed, JoystickProcessingResult &js, ManualModeOutputs &out, bool brakingApplied)
{
  // DEBUG
  Serial.print("Dir: ");
  if (leftSpeed > 0 && rightSpeed > 0)
  {
    Serial.print("FORWD");
  }
  else if (leftSpeed < 0 && rightSpeed < 0)
  {
    Serial.print("BCKWD");
  }
  else if (leftSpeed == 0 && rightSpeed == 0)
  {
    Serial.print("STOP ");
  }
  else if (leftSpeed < rightSpeed)
  {
    Serial.print("RIGHT");
  }
  else if (rightSpeed < leftSpeed)
  {
    Serial.print("LEFT ");
  }
  else
  {
    Serial.print("MIXED"); // not ok good
  }

  Serial.print("    Q LR: ");
  Serial.print(pad5f(js.steppedRatioLR)); // stepped ratio
  Serial.print(" (");
  Serial.print(pad5f(js.rawRatioLR));
  Serial.print(")");

  Serial.print("    Targ Spd: ");
  Serial.print(pad5(out.outputLeft));
  Serial.print("");
  Serial.print(pad5(out.outputRight));

  Serial.print("    Curr Spd: ");
  Serial.print(pad5(leftSpeed));
  Serial.print(" | ");
  Serial.print(pad5(rightSpeed));

  Serial.print("    Slew: ");
  Serial.print(!out.skipSlewRate ? "Y" : "N");
  Serial.print(" | ");
  Serial.print("Brk: ");
  Serial.print(brakingApplied ? "Y" : "N");

  Serial.print("    Corr. XY: ");
  Serial.print(pad5(js.correctedX));
  Serial.print(" | ");
  Serial.print(pad5(js.correctedY));

}

// Simple CRC calculation (XOR-based)
uint8_t calcCRC(uint16_t x, uint16_t y, bool btn)
{
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

  if (rf_driver.recv(buf, &buflen))
  {
    // Copy to struct
    memcpy(&rcvData, buf, min(buflen, sizeof(rcvData)));

    // Verify CRC
    if (rcvData.crc == calcCRC(rcvData.x, rcvData.y, rcvData.btn))
    {
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
  if (abs(leftSpeed) < JOYSTICK_DEADZONE && abs(rightSpeed) < JOYSTICK_DEADZONE) {
      leftSpeed = 0;
      rightSpeed = 0;
  }

  // Constrain speeds to valid range
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  if ((abs(leftSpeed) >= MIN_MOTOR_SPEED || abs(rightSpeed) >= MIN_MOTOR_SPEED) &&
      ((leftSpeed > 0 && rightSpeed > 0) || (leftSpeed < 0 && rightSpeed < 0)))
  {
    // Apply offsets to adjust for motor differences (applies only if both wheels in motion and in same direction)
    leftSpeed += leftSpeed > 0 ? LEFT_OFFSET : -LEFT_OFFSET;
    rightSpeed += rightSpeed > 0 ? RIGHT_OFFSET : -RIGHT_OFFSET;

    // if any of the speeds is below the minimum, but the other is above, creep both up until threshold reached for both
    if (abs(leftSpeed) < MIN_MOTOR_SPEED || abs(rightSpeed) < MIN_MOTOR_SPEED)
    {
      int diff = MIN_MOTOR_SPEED - min(abs(leftSpeed), abs(rightSpeed)); // smallest one is the one below threshold

      leftSpeed += leftSpeed > 0 ? diff : -diff;
      rightSpeed += rightSpeed > 0 ? diff : -diff;
    }

    // Maintain the offset: keep the difference between left and right speeds constant after adjustments
    int diff = leftSpeed - rightSpeed;

    // Constrain leftSpeed and rightSpeed, but preserve their difference
    if (leftSpeed > MAX_SPEED)
    {
      leftSpeed = MAX_SPEED;
      rightSpeed = leftSpeed - diff;
    }
    else if (leftSpeed < -MAX_SPEED)
    {
      leftSpeed = -MAX_SPEED;
      rightSpeed = leftSpeed - diff;
    }

    if (rightSpeed > MAX_SPEED)
    {
      rightSpeed = MAX_SPEED;
      leftSpeed = rightSpeed + diff;
    }
    else if (rightSpeed < -MAX_SPEED)
    {
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

void beep(int duration) {
  tone(BUZZER_PIN, 2000); // 2kHz tone
  delay(duration);
  noTone(BUZZER_PIN);
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
