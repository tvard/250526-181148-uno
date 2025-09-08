#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#endif
#include "helpers.h"
#include "pitches.h"
#include "2WD_RC_RECEIVER_logic.h"

// Pin Definitions
const int RIGHT_MOTOR_EN = 5;   // L293D PIN 1 (Enable 1)
const int RIGHT_MOTOR_IN1 = 3;  // L293D PIN 2 (Input 1)
const int RIGHT_MOTOR_IN2 = 2;  // L293D PIN 7 (Input 2)

const int LEFT_MOTOR_EN = 6;    // L293D PIN 9 (Enable 2)
const int LEFT_MOTOR_IN2 = 7;   // L293D PIN 10 (Input 4)
const int LEFT_MOTOR_IN1 = 9;   // L293D PIN 15 (Input 3)

const int BUZZER_PIN = 8;
const int NRF_CE_PIN = 10;   // CE pin  
const int NRF_CSN_PIN = A0;  // CSN pin - works perfectly as digital pin
const int VOLTAGE_ADC_PIN = A2; // Analog voltage sensing pin

// Radio configuration - MUST MATCH TRANSMITTER
const int RADIO_CHANNEL = 76;
const byte addresses[][6] = {"00001", "00002"};

// Packet history globals (definition)
uint32_t packetHistory = 32;
uint8_t packetIndex = 0;

// Packet structures - MUST MATCH TRANSMITTER EXACTLY
struct JoystickData
{
  int xValue;         // 16-bit int (matches transmitter)
  int yValue;         // 16-bit int (matches transmitter)
  bool buttonPressed; // 8-bit bool (matches transmitter)
  uint8_t checksum;   // 8-bit checksum (matches transmitter)
};

// Simplified response data for ACK payload - MATCHES what transmitter expects
struct __attribute__((packed)) RxData
{
  uint8_t voltage;     // Battery voltage (0-255)
  uint8_t successRate; // Rx success rate (0-255)
  uint8_t status;      // Combined mode + state info
  uint8_t crc;         // Simple CRC
};

// Global objects - dynamic allocation for memory efficiency
RF24 *radio = nullptr;

// Control variables
bool autoMode = false;
int leftSpeed = 0, rightSpeed = 0;

// RF Signal variables
int joystickX = 512; // Center position (range: 0-1023)
int joystickY = 512; // Center position (range: 0-1023)
bool joystickButton = false;

// Function prototypes
int freeMemory();
void initRadio();
bool readRFSignals();
void printStatusReport();
uint8_t calculateChecksum(const JoystickData &data);
uint8_t calculateRxCRC(const RxData &data);
void manualMode();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void stopMotors();
void beep(int duration);
void beep(bool isActive);

// Entertainer melody functions
struct EntertainerState
{
  int noteIndex = 0;
  unsigned long phaseStart = 0;
  int phase = 0;
  bool playing = false;
};
static EntertainerState entertainerState;
bool playEntertainerStep(EntertainerState &state, bool shouldInterrupt);

// Free memory utility
int freeMemory()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void setup()
{
  analogReference(3.3); // 3.3V reference
  pinMode(VOLTAGE_ADC_PIN, INPUT);

  Serial.begin(9600);
  delay(1000);

  Serial.println("=== RECEIVER ===");
  Serial.print("Free memory at startup: ");
  Serial.println(freeMemory());

  // Initialize SPI first
  SPI.begin();
  Serial.print("Free memory after SPI: ");
  Serial.println(freeMemory());

  // Initialize radio with dynamic allocation
  initRadio();

  // Initialize motor control pins
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_EN, OUTPUT);

  // Initialize buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialize I2C for future expansion
  Wire.begin();
  Wire.setClock(100000);

  // Stop all motors initially
  stopMotors();

  Serial.print("Final free memory: ");
  Serial.println(freeMemory());
  Serial.println("Setup complete!");
}

void initRadio()
{
  Serial.println("Initializing radio...");

  // Allocate radio object dynamically
  radio = new RF24(NRF_CE_PIN, NRF_CSN_PIN);

  if (radio && radio->begin())
  {
    Serial.println("NRF24L01 initialized successfully.");

    // Configure radio - MATCH TRANSMITTER EXACTLY
    radio->setChannel(RADIO_CHANNEL);
    radio->setDataRate(RF24_250KBPS);
    radio->setPALevel(RF24_PA_HIGH);
    radio->setPayloadSize(sizeof(JoystickData));
    radio->setAutoAck(true);   // Enable ACK
    radio->enableAckPayload(); // Enable ACK payloads
    radio->setRetries(5, 5);   // Match transmitter retry settings

    // Address configuration - RECEIVER LISTENS ON "00001", RESPONDS ON "00002"
    radio->openReadingPipe(0, addresses[0]); // Listen on address "00001"
    radio->openWritingPipe(addresses[1]);    // Write responses on "00002"
    radio->startListening();

    Serial.print("Channel: ");
    Serial.println(RADIO_CHANNEL);
    Serial.print("Data Rate: 250KBPS");
    Serial.print(" | Power Level: HIGH");
    Serial.print(" | Free memory after radio: ");
    Serial.println(freeMemory());

    // Success beeps
    beep(100);
    beep(100);
  }
  else
  {
    Serial.println("NRF24L01 failed to initialize!");
    beep(1000); // Error beep
  }
}

void loop()
{
  // Prepare response data for transmitter (do this FIRST)
  int voltageRaw = analogRead(VOLTAGE_ADC_PIN);
  uint8_t voltage8bit = map(voltageRaw, 0, MAX_ADC_VALUE, 0, 255);
  uint8_t mode = autoMode ? 1 : 0; // 0 = manual, 1 = auto
  uint8_t state = 0;               // 0 = stopped, 1 = forward, 2 = backward, 3 = turning
  if (leftSpeed > 0 && rightSpeed > 0)
    state = 1;
  else if (leftSpeed < 0 && rightSpeed < 0)
    state = 2;
  else if (leftSpeed != rightSpeed)
    state = 3;

  // Pack response data
  RxData txData;
  txData.voltage = voltage8bit;
  txData.status = (mode << 2) | state; // Pack mode and state
  txData.crc = calculateRxCRC(txData);
  txData.successRate = (uint8_t)(getSuccessRate() * 255); // Scale to 0-255

  // Pre-load ACK payload for next transmission (ALWAYS do this)
  if (radio)
  {
    radio->writeAckPayload(0, &txData, sizeof(RxData));
  }

  // Now handle manual mode (receiving and processing)
  manualMode();

  // Status reporting every n milliseconds
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 250)
  {
    printStatusReport();
    lastStatusTime = millis();
  }

  delay(LOOP_DELAY_MS);
}

void manualMode()
{
  static int prevLeftSpeed = 0, prevRightSpeed = 0;
  static int rfLostCounter = 0;
  static bool brakingApplied = false;

  bool isRead = readRFSignals();

  if (isRead)
  {
    rfLostCounter = 0; // Reset RF lost counter

    JoystickProcessingResult js = processJoystick(joystickX, joystickY, joystickButton);
    MotorTargets mt = computeMotorTargets(js, leftSpeed, rightSpeed);

    // Slew rate logic
    int nextLeft = leftSpeed;
    int nextRight = rightSpeed;
    if (mt.skipSlewRate)
    {
      nextLeft = mt.left;
      nextRight = mt.right;
    }
    else
    {
      if (nextLeft != mt.left)
        nextLeft = slewRateLimit(nextLeft, mt.left);
      if (nextRight != mt.right)
        nextRight = slewRateLimit(nextRight, mt.right);
    }

    // Braking logic
    brakingApplied = shouldApplyBraking(prevLeftSpeed, prevRightSpeed, mt.left, mt.right);
    int outputLeft, outputRight;
    if (brakingApplied)
    {
      outputLeft = -prevLeftSpeed / 4;
      outputRight = -prevRightSpeed / 4;
      nextLeft = 0;
      nextRight = 0;
    }
    else
    {
      outputLeft = (abs(nextLeft) >= MIN_MOTOR_SPEED) ? nextLeft : 0;
      outputRight = (abs(nextRight) >= MIN_MOTOR_SPEED) ? nextRight : 0;
    }

    leftSpeed = nextLeft;
    rightSpeed = nextRight;
    setMotorSpeeds(outputLeft, outputRight);
    prevLeftSpeed = leftSpeed;
    prevRightSpeed = rightSpeed;

    // Buzzer feedback
    beep(js.buzzerOn);

    // Update packet history with successful reception
    updatePacketHistory(true);
  }
  else if (rfLostCounter++ * LOOP_DELAY_MS > 440)
  { // RF signal lost after ~0.5s
    leftSpeed = 0;
    rightSpeed = 0;
    setMotorSpeeds(leftSpeed, rightSpeed);

    // Update packet history with failure
    updatePacketHistory(false);

    // Serial.println("RF SIGNAL LOSS: STOPPING...");
    rfLostCounter = 441; // Prevent overflow
  }
}

bool readRFSignals()
{
  // static unsigned long lastReceiveTime = 0; // (unused)
  static int consecutiveFailures = 0;

  if (radio && radio->available())
  {
    JoystickData receivedData;

    // Read the payload
    radio->read(&receivedData, sizeof(JoystickData));

    // Verify checksum
    uint8_t calculatedChecksum = calculateChecksum(receivedData);
    if (receivedData.checksum == calculatedChecksum)
    {
      // Valid data received
      joystickX = receivedData.xValue;
      joystickY = receivedData.yValue;
      joystickButton = receivedData.buttonPressed;

      // lastReceiveTime = millis();
      consecutiveFailures = 0;

      return true;
    }
    else
    {
      // Checksum mismatch
      Serial.println("");
      Serial.print("!!! Checksum error: expected ");
      Serial.print(calculatedChecksum);
      Serial.print(", received ");
      Serial.print(receivedData.checksum);
      Serial.println(" !!! ");
      consecutiveFailures++;
    }
  }

  return false;
}

// Calculate checksum - MUST MATCH TRANSMITTER
uint8_t calculateChecksum(const JoystickData &data)
{
  uint8_t checksum = 0;
  checksum ^= (data.xValue & 0xFF);
  checksum ^= ((data.xValue >> 8) & 0xFF);
  checksum ^= (data.yValue & 0xFF);
  checksum ^= ((data.yValue >> 8) & 0xFF);
  checksum ^= data.buttonPressed ? 0xFF : 0x00;
  return checksum;
}

// Calculate CRC for response data
uint8_t calculateRxCRC(const RxData &data)
{
  return (data.voltage ^ data.status) & 0xFF;
}

void updatePacketHistory(bool success) {
  if (success) {
    packetHistory |= (1UL << packetIndex);
  } else {
    packetHistory &= ~(1UL << packetIndex);
  }
  packetIndex = (packetIndex + 1) % PACKET_HISTORY_SIZE;
}

float getSuccessRate() {
  uint32_t hist = packetHistory;
  int count = 0;
  for (int i = 0; i < PACKET_HISTORY_SIZE; i++) {
    if (hist & 1UL) count++;
    hist >>= 1;
  }
  return (float)count / PACKET_HISTORY_SIZE;
}
// Motor control functions (unchanged for space - your existing implementations)
void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
  // Your existing implementation
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  static int prevLeftSpeed = 0;
  static int prevRightSpeed = 0;

  if ((abs(leftSpeed) >= MIN_MOTOR_SPEED || abs(rightSpeed) >= MIN_MOTOR_SPEED) &&
      ((leftSpeed > 0 && rightSpeed > 0) || (leftSpeed < 0 && rightSpeed < 0)))
  {
    if (abs(leftSpeed) < MIN_MOTOR_SPEED || abs(rightSpeed) < MIN_MOTOR_SPEED)
    {
      int diff = MIN_MOTOR_SPEED - min(abs(leftSpeed), abs(rightSpeed));
      leftSpeed += leftSpeed > 0 ? diff : -diff;
      rightSpeed += rightSpeed > 0 ? diff : -diff;
    }

    int diff = leftSpeed - rightSpeed;
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

    leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
  }

  if (leftSpeed != prevLeftSpeed)
  {
    if (leftSpeed >= 0)
    {
      digitalWrite(LEFT_MOTOR_IN1, LOW);
      digitalWrite(LEFT_MOTOR_IN2, HIGH);
      analogWrite(LEFT_MOTOR_EN, leftSpeed);
    }
    else
    {
      digitalWrite(LEFT_MOTOR_IN1, HIGH);
      digitalWrite(LEFT_MOTOR_IN2, LOW);
      analogWrite(LEFT_MOTOR_EN, -leftSpeed);
    }
    prevLeftSpeed = leftSpeed;
  }

  if (rightSpeed != prevRightSpeed)
  {
    if (rightSpeed >= 0)
    {
      digitalWrite(RIGHT_MOTOR_IN1, LOW);
      digitalWrite(RIGHT_MOTOR_IN2, HIGH);
      analogWrite(RIGHT_MOTOR_EN, rightSpeed);
    }
    else
    {
      digitalWrite(RIGHT_MOTOR_IN1, HIGH);
      digitalWrite(RIGHT_MOTOR_IN2, LOW);
      analogWrite(RIGHT_MOTOR_EN, -rightSpeed);
    }
    prevRightSpeed = rightSpeed;
  }
}

void stopMotors()
{
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_EN, 0);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(RIGHT_MOTOR_EN, 0);
}

void beep(int duration)
{
  tone(BUZZER_PIN, 1000);
  delay(duration);
  noTone(BUZZER_PIN);
}

void beep(bool isActive)
{
  if (isActive)
  {
    tone(BUZZER_PIN, 1000);
  }
  else
  {
    noTone(BUZZER_PIN);
  }
}

// Entertainer melody function (simplified for space)
bool playEntertainerStep(EntertainerState &state, bool shouldInterrupt)
{
  // Your existing implementation
  return false; // Simplified for this optimization
}

void printStatusReport()
{

  static bool isFirstReport = true;
  if (isFirstReport)
  {
    Serial.println("");
    Serial.println("=== RECEIVER STATUS REPORT ===");
    Serial.println("");
    isFirstReport = false;
  }

  // Radio status
  Serial.print("Radio:");
  if (radio && radio->isChipConnected())
  {
    Serial.print(padString("OK", 4));
    Serial.print(" | Data:");
    Serial.print(radio->available() ? " YES" : "  NO");

    // Display success rate
    float successRate = getSuccessRate();
    Serial.print(" | Success:");
    Serial.print(pad3s((int)(successRate * 100.0)));
    Serial.print("%");
  }
  else
  {
    Serial.print("FAIL | Data:  -- | Success: --%");
  }

  // Current joystick values
  Serial.print(" | JS X:");
  Serial.print(pad3s(joystickX));
  Serial.print(" Y:");
  Serial.print(pad3s(joystickY));
  Serial.print(" Btn:");
  Serial.print(joystickButton ? " ON" : "OFF");

  // Motor speeds
  Serial.print(" | Motors L:");
  Serial.print(pad4s(leftSpeed));
  Serial.print(" R:");
  Serial.print(pad4s(rightSpeed));

  // Direction indicator
  Serial.print(" Dir:");
  String direction;
  if (leftSpeed == 0 && rightSpeed == 0)
  {
    direction = "STOP";
  }
  else if (leftSpeed > 0 && rightSpeed > 0)
  {
    direction = "FWD";
  }
  else if (leftSpeed < 0 && rightSpeed < 0)
  {
    direction = "REV";
  }
  else if (leftSpeed > rightSpeed)
  {
    direction = "RIGHT";
  }
  else if (rightSpeed > leftSpeed)
  {
    direction = "LEFT";
  }
  else
  {
    direction = "MIX";
  }
  Serial.print(padString(direction, 6));

  // L/R Ratio
  Serial.print(" Ratio:");
  if (abs(leftSpeed) == 0 && abs(rightSpeed) == 0)
  {
    Serial.print(padString("--", 6));
  }
  else if (rightSpeed == 0)
  {
    Serial.print(padString("L-only", 6));
  }
  else if (leftSpeed == 0)
  {
    Serial.print(padString("R-only", 6));
  }
  else
  {
    float ratio = (float)leftSpeed / rightSpeed;
    Serial.print(padString(String(ratio), 6));
  }

  // Voltage
  Serial.print(" | Bat:");
  float adcValue = analogRead(VOLTAGE_ADC_PIN);
  float batteryVoltage = (adcValue * VOLTAGE_ADC_REFERENCE * VOLTAGE_CALIBRATION_BATTERY) / ((float)MAX_ADC_VALUE * VOLTAGE_CALIBRATION_ADC_PIN);
  Serial.print(pad5f(batteryVoltage));
  Serial.print("V");
  Serial.print(" (");
  Serial.print(pad5f(adcValue));
  Serial.print(")");

  // Memory
  extern int __heap_start, *__brkval;
  int v;
  int freeMemory = (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
  Serial.print(" | Mem:");
  Serial.print(pad4s(freeMemory));

  // System uptime
  unsigned long uptimeMs = millis();
  unsigned long uptimeSec = uptimeMs / 1000;
  unsigned long uptimeMin = uptimeSec / 60;
  Serial.print(" | Up:");
  Serial.print(pad2s(uptimeMin));
  Serial.print("m");
  Serial.print(pad2s(uptimeSec % 60));
  Serial.print("s");

  Serial.println("");
}