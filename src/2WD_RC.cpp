#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include <RH_ASK.h> // Include the RadioHead library
#include <SPI.h>    // Include the SPI library (required for RadioHead)
#include "MPU6050.h" // Changed to MPU6050 library which is more stable

// Pin Definitions
#define LEFT_MOTOR_IN1 2    // L293D In1 (pin 2)
#define LEFT_MOTOR_IN2 3    // L293D In2 (pin 7)
#define RIGHT_MOTOR_IN3 4   // L293D In3 (pin 10)
#define RIGHT_MOTOR_IN4 7   // L293D In4 (pin 15)
#define LEFT_MOTOR_EN 5     // L293D Enable1 (pin 1)
#define RIGHT_MOTOR_EN 6    // L293D Enable2 (pin 9)

#define CS_PIN 7
#define BUZZER_PIN 10       // Active buzzer (as suggested)
#define MODE_BUTTON_PIN 6   // Push button for mode toggle -- CHANGED TO PIN 6 AS PER YOUR COMMENT
#define ULTRASONIC_TRIG 12  // HC-SR04 Trigger pin
#define ULTRASONIC_ECHO 13  // HC-SR04 Echo pin

// Constants
#define MAX_SPEED 255
#define MIN_DISTANCE 30     // Minimum distance in cm before turning
#define TURN_TIME 800       // Time to turn in milliseconds
#define SCAN_INTERVAL 300   // Time between distance measurements

const int LOOP_DELAY_MS = 50;

// Initialize the RH_ASK driver
// RH_ASK(speed, receive_pin, transmit_pin, ptt_pin, ptt_inverted)
// For a receiver-only sketch, only receive_pin is strictly necessary.
RH_ASK rf_driver(1000); // Changed speed to 2000, common default, match your transmitter's actual speed.

// Structure for joystick data - MUST MATCH THE TRANSMITTER EXACTLY
struct JoystickData {
  int xValue;
  int yValue;
  bool buttonPressed;
};

// Control variables
bool autoMode = false;      // Start in manual mode
unsigned long lastModeChange = 0; // Debounce button
unsigned long lastDistanceCheck = 0;
int lastButtonState = HIGH; // Previous state of button
bool buzzerEnabled = false;

// RF Signal variables (these will be updated directly from received JoystickData)
int joystickX = 512;        // Center position (range: 0-1023)
int joystickY = 512;        // Center position (range: 0-1023)
bool joystickButton = false;

// IMU setup - Using MPU6050 library (compatible with MPU6500/9250)
MPU6050 mpu;

float ax, ay, az;
float gx, gy, gz;

// Function prototypes (good practice)
void checkModeButton();
void manualMode();
void autonomousMode();
void readIMU();
long getDistance();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void stopMotors();
void turnLeft(int angle);
void turnRight(int angle);
void beep(int duration);
void beepPattern(int beeps, int beepDuration);
bool readRFSignals(); // Added prototype

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  Serial.println("RC Car Control System Starting...");

  // Initialize RF driver
  if (!rf_driver.init()) {
    Serial.println("RF driver failed to initialize!");
  } else {
    Serial.println("RF driver initialized.");
  }

  // Initialize motor control pins
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN3, OUTPUT);
  pinMode(RIGHT_MOTOR_IN4, OUTPUT);
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
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found!");
  } else {
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" I2C device(s)");
  }

  // Initialize MPU6050/6500/9250
  Serial.println("Initializing MPU...");
  mpu.initialize();
  
  delay(100);
  
  // Test connection
  if (mpu.testConnection()) {
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
    
  } else {
    Serial.println("MPU connection failed!");
    Serial.println("Check wiring and power supply");
    // Error tone - long beep
    beep(1000);
  }

  // Stop all motors initially
  stopMotors();

  Serial.println("Setup complete");
}

void loop() {
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

void checkModeButton() {
  int reading = digitalRead(MODE_BUTTON_PIN);

  // Debouncing
  if (reading != lastButtonState) {
    lastButtonState = reading;

    // If the button is pressed (LOW due to pull-up)
    if (reading == LOW) {
      // Debounce timing - prevent multiple toggles
      if (millis() - lastModeChange > 300) { // Increased debounce to 300ms
        autoMode = !autoMode;
        lastModeChange = millis();

        // Indicate mode change with beep
        if (autoMode) {
          Serial.println("Switching to AUTO mode");
          beep(300); // Long beep for auto mode
        } else {
          Serial.println("Switching to MANUAL mode");
          beepPattern(2, 100); // Two short beeps for manual mode
        }
        // Stop motors immediately on mode change for safety
        stopMotors();
      }
    }
  }
}

void manualMode() {
  // Read and interpret RF signals from the receiver
  bool isRead = readRFSignals();
  // Invert joystick axes to correct orientation
  int correctedX = 1023 - joystickY; // X increases rightwards
  int correctedY = 1023 - joystickX; // Y increases upwards

  int leftSpeed = 0;
  int rightSpeed = 0;
  static int rfLostCounter = 0; // Counter for lost RF signals

  if (isRead)
  {
  // Global variables (declare these outside your loop)
  static int targetLeftSpeed = 0, targetRightSpeed = 0;
  static int currentLeftSpeed = 0, currentRightSpeed = 0;
  static float smoothingFactor = 0.5; 

  // In your main loop:
  // 1. First calculate base forward/backward speeds
  if (correctedY > 550) { // Forward
    int speed = map(correctedY, 550, 1023, 0, MAX_SPEED);
    leftSpeed = speed;
    rightSpeed = speed;
  } else if (correctedY < 470) { // Backward
    int speed = map(correctedY, 470, 0, 0, MAX_SPEED);
    leftSpeed = -speed;
    rightSpeed = -speed;
  }

  // 2. Then apply steering adjustments
  float forwardBackward = abs(correctedY - 512) / 511.0;
  float steeringScale = 1.0 - 0.6 * pow(forwardBackward, 1.5);

  if (correctedX > 550) { // Right turn
    int adjustment = map(correctedX, 550, 1023, 0, MAX_SPEED * steeringScale);
    rightSpeed -= adjustment;
    leftSpeed += adjustment * 0.3;
  } else if (correctedX < 470) { // Left turn
    int adjustment = map(correctedX, 470, 0, 0, MAX_SPEED * steeringScale);
    leftSpeed -= adjustment;
    rightSpeed += adjustment * 0.3;
  }

  // 3. Finally apply smoothing to the calculated target speeds
  targetLeftSpeed = leftSpeed;
  targetRightSpeed = rightSpeed;

  currentLeftSpeed += (targetLeftSpeed - currentLeftSpeed) * smoothingFactor;
  currentRightSpeed += (targetRightSpeed - currentRightSpeed) * smoothingFactor;

  // 4. Use the smoothed speeds for your motors
  leftSpeed = currentLeftSpeed;
  rightSpeed = currentRightSpeed;


    Serial.print(" ");
    Serial.print("Corr X: ");
    Serial.print(correctedX);
    Serial.print(" | ");
    Serial.print("Corr Y: ");
    Serial.print(correctedY);

    Serial.print(" ");
    Serial.print("L Spd: ");
    Serial.print(leftSpeed);
    Serial.print(" | ");
    Serial.print("R Spd: ");
    Serial.println(rightSpeed); 
    setMotorSpeeds(leftSpeed, rightSpeed);
    rfLostCounter = 0; // Reset counter if RF signal is received
  }
  else 
  {
     if(rfLostCounter++ * LOOP_DELAY_MS > 500) {
        setMotorSpeeds(0, 0); // Stop motors if no RF signal for (n)ms
        rfLostCounter = 0; // Reset counter to avoid overflow
     }
  }

    // Check if joystick button is pressed to activate buzzer
    if (joystickButton && !buzzerEnabled) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerEnabled = true;
    } else if (!joystickButton && buzzerEnabled) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerEnabled = false;
    }

}

// *** MODIFIED readRFSignals FUNCTION ***
bool readRFSignals() {
  JoystickData receivedData; // Create a struct to hold the received data
  uint8_t len = sizeof(receivedData); // Set the expected length of the struct

  if (rf_driver.recv((uint8_t *)&receivedData, &len)) {
    // If data was received, update our global joystick variables
    joystickX = receivedData.xValue;
    joystickY = receivedData.yValue;
    joystickButton = receivedData.buttonPressed;

    // Print the received values (not the raw buffer)
    Serial.print("Received RF: X=");
    Serial.print(joystickX);
    Serial.print(", Y=");
    Serial.print(joystickY);
    Serial.print(", Button=");
    Serial.print(joystickButton ? "Pressed" : "Released");

    return true;
  }

  return false;
}
// *** END OF MODIFIED readRFSignals FUNCTION ***

void autonomousMode() {
  // Get distance from ultrasonic sensor
  long distance = getDistance();

  // Use IMU to help with navigation
  readIMU();

  // Simple obstacle avoidance logic
  if (distance < MIN_DISTANCE) {
    // Obstacle detected - stop and determine turn direction
    stopMotors();
    delay(200);

    // Use IMU data to decide which way to turn
    // For example, turn to the side with less tilt or acceleration
    if (ax > 0) { // Sample logic - can be refined
      turnRight(180);
    } else {
      turnLeft(180);
    }
  } else {
    // No obstacle, move forward with slight adjustments based on IMU
    // This can help keep the car moving straight
    int baseSpeed = 180; // 70% of max speed for better control
    int leftSpeed = baseSpeed;
    int rightSpeed = baseSpeed;

    // Slight steering correction based on gyro data
    if (gz > 100) { // Drifting right, correct left
      leftSpeed += 20;
      rightSpeed -= 20;
    } else if (gz < -100) { // Drifting left, correct right
      leftSpeed -= 20;
      rightSpeed += 20;
    }

    // Set motor speeds
    setMotorSpeeds(leftSpeed, rightSpeed);
  }
}

void readIMU() {
  // Read the sensor values (similar to your working approach)
  
  // Get accelerometer values (in g's, convert to m/s²)
  int16_t ax_raw, ay_raw, az_raw;
  mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);
  
  // Convert to m/s² (assuming ±4g range: 32768 = 4g = 39.24 m/s²)
  ax = (float)ax_raw / 32768.0 * 4.0 * 9.81;
  ay = (float)ay_raw / 32768.0 * 4.0 * 9.81;
  az = (float)az_raw / 32768.0 * 4.0 * 9.81;

  // Get gyroscope values (in degrees/s, convert to rad/s)
  int16_t gx_raw, gy_raw, gz_raw;
  mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);
  
  // Convert to rad/s (assuming ±500°/s range: 32768 = 500°/s)
  gx = (float)gx_raw / 32768.0 * 500.0 * PI / 180.0;
  gy = (float)gy_raw / 32768.0 * 500.0 * PI / 180.0;
  gz = (float)gz_raw / 32768.0 * 500.0 * PI / 180.0;

  // Print IMU values for debugging (can be removed in final code)
  // You might want to print these less frequently or on demand
  // Serial.print("IMU: ");
  // Serial.print(ax); Serial.print("\t");
  // Serial.print(ay); Serial.print("\t");
  // Serial.print(az); Serial.print("\t");
  // Serial.print(gx); Serial.print("\t");
  // Serial.print(gy); Serial.print("\t");
  // Serial.println(gz);
}

long getDistance() {
  // Trigger ultrasonic pulse
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  // Read the echo time
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH);

  // Calculate distance in cm
  // Speed of sound is 343m/s or 0.0343cm/µs
  // Time is round-trip, so divide by 2
  long distance = (duration * 0.0343) / 2;

  return distance;
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Constrain speeds to valid range
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  // Set left motor direction and speed
  if (leftSpeed >= 0) {
    // Forward
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    analogWrite(LEFT_MOTOR_EN, leftSpeed);
  } else {
    // Backward
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    analogWrite(LEFT_MOTOR_EN, -leftSpeed);
  }

  // Set right motor direction and speed
  if (rightSpeed >= 0) {
    // Forward
    digitalWrite(RIGHT_MOTOR_IN3, HIGH);
    digitalWrite(RIGHT_MOTOR_IN4, LOW);
    analogWrite(RIGHT_MOTOR_EN, rightSpeed);
  } else {
    // Backward
    digitalWrite(RIGHT_MOTOR_IN3, LOW);
    digitalWrite(RIGHT_MOTOR_IN4, HIGH);
    analogWrite(RIGHT_MOTOR_EN, -rightSpeed);
  }
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_EN, 0);

  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
  analogWrite(RIGHT_MOTOR_EN, 0);
}

void turnLeft(int angle) {
  // Simple timed turn - can be refined with IMU for more precise angles
  stopMotors();
  delay(100);

  // Tank turn left - right wheel forward, left wheel backward
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  analogWrite(LEFT_MOTOR_EN, MAX_SPEED * 0.7);

  digitalWrite(RIGHT_MOTOR_IN3, HIGH);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
  analogWrite(RIGHT_MOTOR_EN, MAX_SPEED * 0.7);

  // Time-based turn - approximately 90 degrees
  delay(TURN_TIME * angle / 90);

  stopMotors();
}

void turnRight(int angle) {
  // Simple timed turn - can be refined with IMU for more precise angles
  stopMotors();
  delay(100);

  // Tank turn right - left wheel forward, right wheel backward
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_EN, MAX_SPEED * 0.7);

  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, HIGH);
  analogWrite(RIGHT_MOTOR_EN, MAX_SPEED * 0.7);

  // Time-based turn - approximately 90 degrees
  delay(TURN_TIME * angle / 90);

  stopMotors();
}

void beep(int duration) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  digitalWrite(BUZZER_PIN, LOW);
}

void beepPattern(int beeps, int beepDuration) {
  for (int b = 0; b < beeps; b++) { // Changed loop variable from 'i' to 'b' to avoid conflict with global 'i'
    digitalWrite(BUZZER_PIN, HIGH);
    delay(beepDuration);
    digitalWrite(BUZZER_PIN, LOW);
    delay(beepDuration);
  }
}