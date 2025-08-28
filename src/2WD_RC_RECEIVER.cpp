// For PlatformIO/Arduino
#include <Arduino.h>
#include <Wire.h>
#include <RF24.h>    // NRF24L01 library
#include "helpers.h"
#include "pitches.h"

// Packed data structure for sending battery voltage, mode, and movement state back to transmitter
struct PackedDataReceive {
  uint16_t voltage : 8; // 8 bits for voltage (0-255)
  uint8_t mode : 2;     // 2 bits for mode (0-3)
  uint8_t state : 2;    // 2 bits for movement state (0-3)
  uint8_t crc : 4;      // 4 bits for CRC (0-15)
};

// Simple CRC calculation (XOR-based)
uint8_t calcCRCReceive4(const PackedDataReceive &data) {
  return (data.voltage ^ data.mode ^ data.state) & 0x0F; // Use 4-bit CRC
}
// Pin Definitions
const int RIGHT_MOTOR_IN1 = 2;
const int RIGHT_MOTOR_IN2 = 3;
const int RIGHT_MOTOR_EN = 5;
const int LEFT_MOTOR_IN1 = 4;
const int LEFT_MOTOR_IN2 = 7;
const int LEFT_MOTOR_EN = 6;

const int BUZZER_PIN = 8;
// const int MODE_BUTTON_PIN = 6;
const int ULTRASONIC_TRIG = 12;
const int ULTRASONIC_ECHO = 13;

// NRF24L01 Pin Definitions
const int NRF_CE_PIN = 10;   // CE pin  
const int NRF_CSN_PIN = A0;   // CSN pin (changed from A0 to D9)
// const int NRF_IRQ_PIN = ??;  // IRQ pin (optional)
const int VOLTAGE_ADC_PIN = A2; // Analog voltage sensing pin

// Initialize the NRF24L01 driver
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

// Radio configuration
const byte addresses[][6] = {"00001", "00002"}; // 5-byte addresses for bidirectional communication
const int RADIO_CHANNEL = 76; // Channel 0-125 (2.4GHz + channel MHz)

// Structure for joystick data - MUST MATCH THE TRANSMITTER EXACTLY
struct JoystickData
{
  int xValue;
  int yValue;
  bool buttonPressed;
  uint8_t checksum; // Simple checksum for data integrity
};


// Control variables
bool autoMode = false;            // Start in manual mode
unsigned long lastModeChange = 0; // Debounce button
unsigned long lastDistanceCheck = 0;
int lastButtonState = HIGH; // Previous state of button
bool buzzerEnabled = false;

// Minimal packet history (reduced from 100 to 10)
const int PACKET_HISTORY_SIZE = 10;
uint16_t packetHistory = 0; // Use bitfield instead of array
uint8_t packetIndex = 0;

// RF Signal variables (these will be updated directly from received JoystickData)
int joystickX = 512; // Center position (range: 0-1023)
int joystickY = 512; // Center position (range: 0-1023)
bool joystickButton = false;

// Motor speed variables (global for access in loop)
int leftSpeed = 0, rightSpeed = 0;

float ax, ay, az;
float gx, gy, gz;

// Function prototypes (good practice) => must be declared here before usage below
void manualMode();
void manualModeSerialPrint(int leftSpeed, int rightSpeed, JoystickProcessingResult &js, ManualModeOutputs &out, bool brakingApplied);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void stopMotors();
void beep(int duration);
void beep(bool isActive);
bool readRFSignals();
uint8_t calculateChecksum(const JoystickData& data);
int freeMemory();
float getSuccessRate();

String pad5(int val);
String pad5f(float val);
int slewRateLimit(int current, int target);

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
//   G4 = 392, C5 = 523, E5 = 659, G5 = 784, C6 = 1047, E6 = 1319
static const int melody[] = {
  NOTE_G4, NOTE_E5, NOTE_C5, NOTE_E5,  // pickup + beat 1
  NOTE_G5, NOTE_G5,                    // end of bar 1
  NOTE_E5, NOTE_G5, NOTE_E5, NOTE_C5,  // bar 2 first four eighths
  NOTE_E5, NOTE_G5                    // bar 2 next two eighths (~15 s mark)
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
    state.phase      = 0;   // reset to "play" phase
    state.phaseStart = 0;
    return false;
  }

  unsigned long now = millis();

  // ——————————————————————————————————————————————————————————————
  // 3) Compute this note's on‑time and pause‑time
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

  // Initialize SPI for NRF24L01
  SPI.begin();

  // Initialize NRF24L01
  if (!radio.begin()) {
    Serial.println("NRF24L01 failed to initialize!");
    // Error beep pattern
    beep(1000);
  } else {
    Serial.println("NRF24L01 initialized successfully.");
    
    // Configure the radio
    radio.setChannel(RADIO_CHANNEL);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_HIGH);
    radio.setPayloadSize(sizeof(JoystickData));
    // Enable auto-acknowledgment for reliability
    radio.setAutoAck(true);              // Re-enable ACK
    radio.enableAckPayload();            // Re-enable ACK payloads
    
    // Set retry parameters (5 retries, 1500µs delay) - match transmitter
    radio.setRetries(5, 5);              // Restore retries
    
    // Open reading pipe (receiver) - listen on address "00001" 
    radio.openReadingPipe(0, addresses[0]);  // Listen on address "00001"
    radio.openWritingPipe(addresses[1]);     // Write responses on "00002"
    radio.startListening();
    
    // Print configuration details
    Serial.print("Channel: "); Serial.println(RADIO_CHANNEL);
    Serial.print("Data Rate: ");
    switch (radio.getDataRate()) {
      case RF24_250KBPS: Serial.println("250KBPS"); break;
      case RF24_1MBPS: Serial.println("1MBPS"); break;
      case RF24_2MBPS: Serial.println("2MBPS"); break;
      default: Serial.println("UNKNOWN"); break;
    }
    Serial.print("Power Level: ");
    switch (radio.getPALevel()) {
      case RF24_PA_MIN: Serial.println("MIN"); break;
      case RF24_PA_LOW: Serial.println("LOW"); break;
      case RF24_PA_HIGH: Serial.println("HIGH"); break;
      case RF24_PA_MAX: Serial.println("MAX"); break;
      default: Serial.println("UNKNOWN"); break;
    }
    
    // Success beeps
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

  // // Initialize mode toggle button with internal pullup
  // pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);

  // Initialize buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialize IMU - Using MPU6050 library (works with MPU6500/9250)
  Wire.begin();
  Wire.setClock(100000); // Start with slower I2C speed (100kHz)

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
  manualMode(); // This will be called inside the if/else for autoMode

  // Read battery voltage from VOLTAGE_ADC_PIN
  int voltageRaw = analogRead(VOLTAGE_ADC_PIN);
  // Map to 0-255 (assuming 0-5V, scale to 0-255 for 8 bits)
  uint8_t voltage8bit = map(voltageRaw, 0, 1023, 0, 255);

  // Determine mode and movement state (stubbed, replace with actual logic)
  uint8_t mode = autoMode ? 1 : 0; // 0 = manual, 1 = auto
  uint8_t state = 0; // 0 = stopped, 1 = forward, 2 = backward, 3 = turning (stub)
  if (leftSpeed > 0 && rightSpeed > 0) state = 1;
  else if (leftSpeed < 0 && rightSpeed < 0) state = 2;
  else if (leftSpeed != rightSpeed) state = 3;

  // Pack and send data to transmitter
  PackedDataReceive txData;
  txData.voltage = voltage8bit;
  txData.mode = mode;
  txData.state = state;
  txData.crc = calcCRCReceive4(txData);

  // Send via NRF24L01 (open writing pipe, send, then resume listening)
  radio.stopListening();
  bool success = radio.write(&txData, sizeof(PackedDataReceive));
  radio.startListening();
  
  // Optional: print status for debugging
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint > 1000) { // Every second
    Serial.print("TX to transmitter: ");
    Serial.print(success ? "OK" : "FAIL");
    Serial.print(" | Battery: ");
    Serial.print(voltage8bit);
    Serial.print(" | Mode: ");
    Serial.println(autoMode ? "AUTO" : "MANUAL");
    lastStatusPrint = millis();
    Serial.print("Free memory: ");  Serial.print(freeMemory()); Serial.print(" | ");
    // NRF signal report
    Serial.print("Packet History: "); Serial.print(packetHistory, BIN); Serial.print(" | ");
    Serial.print("Packet Index: "); Serial.print(packetIndex); Serial.print(" | ");
    Serial.print("Success Rate: "); Serial.println(getSuccessRate() * 100);
  }


  delay(LOOP_DELAY_MS);
}


void manualMode()
{
  static int prevLeftSpeed = 0, prevRightSpeed = 0;
  static int rfLostCounter = 0;
  static bool brakingApplied = false;

  bool isRead = readRFSignals();

  JoystickProcessingResult js = processJoystick(joystickX, joystickY, joystickButton, true);

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
    beep(out.buzzerOn);

    // uncomment for debugging movement
    // manualModeSerialPrint(leftSpeed, rightSpeed, js, out, brakingApplied);
    // Serial.println();
  }
  else if (rfLostCounter++ * LOOP_DELAY_MS > 440) // RF-lost ramp-down to avoid judder, signal considered lost after ~0.5s
  {
    leftSpeed = 0;
    rightSpeed = 0;

    setMotorSpeeds(leftSpeed, rightSpeed);
    
    // Only print RF loss message once, not continuously
    if (rfLostCounter == 441 / LOOP_DELAY_MS) {
      Serial.println("RF SIGNAL LOSS: STOPPING...");
    }
    rfLostCounter = 441 / LOOP_DELAY_MS; // Prevent overflow
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

  Serial.print(" Q LR: ");
  Serial.print(js.steppedRatioLR > 0 ? pad5f(js.steppedRatioLR) : "0");
  Serial.print(" (");
  Serial.print(js.rawRatioLR > 0 ? pad5f(js.rawRatioLR) : "0");
  Serial.print(")");

  Serial.print(" Targ Spd: ");
  Serial.print(out.outputLeft > 0 ? pad5(out.outputLeft) : "0");
  Serial.print(" ");
  Serial.print(out.outputRight > 0 ? pad5(out.outputRight) : "0");

  Serial.print(" Curr Spd: ");
  Serial.print(leftSpeed > 0 ? pad5(leftSpeed) : "0");
  Serial.print(" | ");
  Serial.print(rightSpeed > 0 ? pad5(rightSpeed) : "0");

  Serial.print(" Slew: ");
  Serial.print(!out.skipSlewRate ? "Y" : "N");
  Serial.print(" | ");
  Serial.print("Brk: ");
  Serial.print(brakingApplied ? "Y" : "N");

  Serial.print(" XY: ");
  Serial.print(js.correctedX > 0 ? pad5(js.correctedX) : "0");
  Serial.print(" | ");
  Serial.print(js.correctedY > 0 ? pad5(js.correctedY) : "0");

  // RF communication status
  Serial.print(" | RF: ");
  if (radio.isChipConnected()) {
    Serial.print("NRF OK");
  } else {
    Serial.print("NRF FAIL");
  }

  // Show last sent packet info
  Serial.print(" | TX: V=");
  Serial.print(leftSpeed);
  Serial.print(", ");
  Serial.print(rightSpeed);
  Serial.print(" | Mode: ");
  Serial.print(autoMode ? "AUTO" : "MAN");
}

// Calculate checksum for data integrity
uint8_t calculateChecksum(const JoystickData& data) {
  uint8_t checksum = 0;
  checksum ^= (data.xValue & 0xFF);
  checksum ^= ((data.xValue >> 8) & 0xFF);
  checksum ^= (data.yValue & 0xFF);
  checksum ^= ((data.yValue >> 8) & 0xFF);
  checksum ^= data.buttonPressed ? 0xFF : 0x00;
  return checksum;
}

/// @brief
/// Reads RF signals from the joystick transmitter and updates global joystick variables.
/// @return
/// Returns true if data was successfully received and updated, false otherwise.
bool readRFSignals()
{
  static unsigned long lastReceiveTime = 0;
  static int consecutiveFailures = 0;
  static unsigned long lastDebugPrint = 0;
  
  if (radio.available()) {
    JoystickData receivedData;
    
    // Read the payload
    radio.read(&receivedData, sizeof(JoystickData));
    
    // Verify checksum
    uint8_t calculatedChecksum = calculateChecksum(receivedData);
    if (receivedData.checksum == calculatedChecksum) {
      // Valid data received
      joystickX = receivedData.xValue;
      joystickY = receivedData.yValue;
      joystickButton = receivedData.buttonPressed;
      
      lastReceiveTime = millis();
      consecutiveFailures = 0;
      
      // Debug print every 100 valid packets
      static int validPacketCount = 0;
      if (++validPacketCount % 100 == 0) {
        Serial.print("RX: X=");
        Serial.print(joystickX);
        Serial.print(" Y=");
        Serial.print(joystickY);
        Serial.print(" Btn=");
        Serial.println(joystickButton ? "ON" : "OFF");
      }
      
      return true;
    } else {
      // Checksum mismatch - corrupted data
      Serial.print("Checksum error: expected ");
      Serial.print(calculatedChecksum);
      Serial.print(", received ");
      Serial.println(receivedData.checksum);
      consecutiveFailures++;
    }
  }
  
  // Print radio status occasionally
  if (millis() - lastDebugPrint > 5000) { // Every 5 seconds
    Serial.print("Radio status: ");
    if (radio.isChipConnected()) {
      Serial.print("Connected");
    } else {
      Serial.print("DISCONNECTED");
    }
    Serial.print(" | Available: ");
    Serial.println(radio.available() ? "YES" : "NO");
    lastDebugPrint = millis();
  }
  
  // Check for communication timeout
  if (millis() - lastReceiveTime > 1000) { // 1 second timeout
    if (consecutiveFailures > 10) {
      Serial.println("Too many consecutive RF failures - check transmitter");
      consecutiveFailures = 0; // Reset to avoid spam
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

  // Static variables to track previous states to avoid unnecessary PWM writes
  static int prevLeftSpeed = 0;
  static int prevRightSpeed = 0;

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

  // Only update motor states if they've changed
  if (leftSpeed != prevLeftSpeed) {
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
    prevLeftSpeed = leftSpeed;
  }

  if (rightSpeed != prevRightSpeed) {
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
    prevRightSpeed = rightSpeed;
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

void beep(int duration) {
  tone(BUZZER_PIN, 1000);  // Play 1kHz tone
  delay(duration);
  noTone(BUZZER_PIN);
}

// this is intended to run in loop()
void beep(bool isActive){
  if(isActive){
    tone(BUZZER_PIN, 1000);  // Play 1kHz tone
  }
  else{
    noTone(BUZZER_PIN);
  }
}

void beginNRF()
{
  // Initialize the NRF24L01 module
  radio.begin();
  const byte address[6] = "00001";
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);  // Maximum power for longer range
  radio.stopListening();
}

void endNRF()
{
  radio.stopListening();
  radio.powerDown();
}

// Free memory utility
int freeMemory() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


float getSuccessRate() {
  int count = 0;
  for (int i = 0; i < PACKET_HISTORY_SIZE; i++) {
    if (packetHistory & (1 << i)) count++;
  }
  return (float)count / PACKET_HISTORY_SIZE;
}

/*
  New Pin Configuration:

  | NRF24L01 pin | Arduino Pro Mini |                                 Notes                                 |
  |--------------|------------------|-----------------------------------------------------------------------|
  | VCC          | 3.3V             | Add a 10–47µF electrolytic (plus a 100nF ceramic) right at the module |
  | GND          | GND              | Common ground                                                         |
  | CE           | A0               | Any digital works; you chose A0 (ok)                                  |
  | CSN          | D10              | SPI chip-select (keep as OUTPUT)                                      |
  | SCK          | D13              | SPI clock                                                             |
  | MOSI         | D11              | SPI data out                                                          |
  | MISO         | D12              | SPI data in                                                           |
  | IRQ          | (optional)       | Only if you use interrupts; else leave NC                             |

  | Voltage Divider Output | A1      | Analog voltage sensing input                                          |
*/