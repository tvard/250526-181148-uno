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
const int RIGHT_MOTOR_EN = 5;  // L293D PIN 1 (Enable 1)
const int RIGHT_MOTOR_IN1 = 3; // L293D PIN 2 (Input 1)
const int RIGHT_MOTOR_IN2 = 2; // L293D PIN 7 (Input 2)

const int LEFT_MOTOR_EN = 6;  // L293D PIN 9 (Enable 2)
const int LEFT_MOTOR_IN2 = 7; // L293D PIN 10 (Input 4)
const int LEFT_MOTOR_IN1 = 9; // L293D PIN 15 (Input 3)

const int BUZZER_PIN = 8;
const int NRF_CE_PIN = 10;      // CE pin
const int NRF_CSN_PIN = A0;     // CSN pin - works perfectly as digital pin
const int VOLTAGE_ADC_PIN = A2; // Analog voltage sensing pin

// Radio configuration - MUST MATCH TRANSMITTER
const int RADIO_CHANNEL = 76;
const byte addresses[][6] = {"00001", "00002"};

// Packet history globals (definition)
uint32_t packetHistory = 32;
uint8_t packetIndex = 0;

// Global objects - dynamic allocation for memory efficiency
RF24 *radio = nullptr;
RxData rxData(0, 0, 0, static_cast<RxRequestType>(0), 0);
MotorTargets prevMotorValues = {0, 0, 0, 0, false, false};

// RF Signal variables
int joystickX = 512; // Center position (range: 0-1023)
int joystickY = 512; // Center position (range: 0-1023)
bool joystickButton = false;

// In 2WD_RC_RECEIVER_logic.cpp
extern uint16_t xCenter;
extern uint16_t yCenter;

// Fast slew mode toggle
bool FAST_SLEW_MODE = false;

// Function prototypes
int freeMemory();
void initRadio();
void toggleFastSlewOnLongBeep(JoystickProcessingResult &js);
bool readRFSignals();
void successBeep();
void printStatusReport(const RxData &rxData, bool isRead, MotorTargets mt);
uint8_t calculateChecksum(const JoystickData &data);
uint8_t calculateRxCRC(const RxData &data);
void manualMode();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void stopMotors();
void beep(int duration);
void beep(bool isActive);
bool updateVoltageReading(RxData &rxData);
bool tryReadCalibrationPacket();
extern int calculateLeftRightPercent(int x);

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

  // Request calibration on startup  
  RxData rxData(0, 0, 0, RXREQ_REQUEST_CALIBRATION, 0); // Initial ACK payload

  if (radio) {
    radio->writeAckPayload(0, &rxData, sizeof(RxData));
    Serial.println("Initial ACK payload loaded");
  }

  Serial.println("Attempting to read calibration packet...");
  if (tryReadCalibrationPacket())
  {
    Serial.println("Calibration packet received during setup.");
    successBeep();
  }
  else
  {
    Serial.println("No calibration packet received during setup.");
    Serial.println("Using default calibration values: X Center = " + String(xCenter) + ", Y Center = " + String(yCenter));
    failBeep();
  }

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

    // Configure radio - FIXED VERSION
    radio->setChannel(RADIO_CHANNEL);
    radio->setDataRate(RF24_250KBPS);
    radio->setPALevel(RF24_PA_HIGH);
    radio->enableDynamicPayloads();
    radio->setAutoAck(true);
    radio->enableAckPayload();
    radio->setRetries(5, 5);

    // FIX: Proper pipe configuration
    radio->openReadingPipe(0, addresses[0]); // Listen on pipe 0 "00001"
    radio->openWritingPipe(addresses[1]);    // Not used, but set for consistency
    radio->startListening();

    // Clear any stale data
    radio->flush_rx();
    radio->flush_tx();

    Serial.print("Channel: ");
    Serial.println(RADIO_CHANNEL);
    Serial.print("Data Rate: 250KBPS");
    Serial.print(" | Power Level: HIGH");
    Serial.print(" | Free memory after radio: ");
    Serial.println(freeMemory());

    // Success beeps
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
  static int rfLostCounter = 0;
  MotorTargets mt = {};
  bool isRead = readRFSignals(); // ACK payload now written inside this function

  if (isRead)
  {
    rfLostCounter = 0; // Reset RF lost counter

    JoystickProcessingResult js = processJoystick(joystickX, joystickY, joystickButton);
    mt = computeMotorTargets(js, prevMotorValues);
    setMotorSpeeds(mt.outputLeft, mt.outputRight);

    // Update prevMotorValues for next cycle
    prevMotorValues = mt;

    // Buzzer feedback
    beep(js.buzzerOn);

    toggleFastSlewOnLongBeep(js);

    // Update packet history with successful reception
    updatePacketHistory(true);
  }
  else if (rfLostCounter++ * LOOP_DELAY_MS > 440)
  { // RF signal lost after ~0.5s
    setMotorSpeeds(0, 0);

    // Update prevMotorValues to stopped
    prevMotorValues = {0, 0, 0, 0, false, false};

    // Update packet history with failure
    updatePacketHistory(false);

    rfLostCounter = 441; // Prevent overflow
  }

  // Status reporting every n milliseconds
  static unsigned long lastStatusTime = 0;

  if ((millis() - lastStatusTime > 250 && isRead) || (millis() - lastStatusTime > 1000))
  {
    // Use last computed MotorTargets for status
    printStatusReport(rxData, isRead, mt);
    lastStatusTime = millis();
  }

  delay(LOOP_DELAY_MS);
}

void toggleFastSlewOnLongBeep(JoystickProcessingResult &js)
{
  static unsigned long lastBeepActive = 0;

  // Toggle fast slew mode with (n)s beep button hold
  if (js.buzzerOn)
  {
    if (millis() - lastBeepActive > 2000)
    {
      FAST_SLEW_MODE = !FAST_SLEW_MODE;
      Serial.print("Fast slew mode ");
      Serial.println(FAST_SLEW_MODE ? "ENABLED" : "DISABLED");
      tone(BUZZER_PIN, FAST_SLEW_MODE ? 2000 : 500);
      delay(200);
      noTone(BUZZER_PIN);
      lastBeepActive = millis(); // Reset to prevent multiple toggles
    }
  }
  else
  {
    lastBeepActive = millis(); // Reset if button not pressed
  }
}

// FIXED readRFSignals() - Move ACK payload writing here
bool readRFSignals()
{
  static int consecutiveFailures = 0;

  if (radio && radio->available())
  {
    JoystickData receivedData;
    uint8_t payloadSize = radio->getDynamicPayloadSize();

    if (payloadSize == sizeof(JoystickData))
    {
      radio->read(&receivedData, sizeof(JoystickData));

      // Verify checksum
      uint8_t calculatedChecksum = calculateChecksum(receivedData);
      if (receivedData.checksum == calculatedChecksum)
      {
        // Valid data received
        joystickX = receivedData.xValue;
        joystickY = receivedData.yValue;
        joystickButton = receivedData.buttonPressed;
        consecutiveFailures = 0;

        // FIX: Write ACK payload immediately after successful receive
        updateVoltageReading(rxData);
        uint8_t state = 0;
        if (prevMotorValues.targetLeft > 0 && prevMotorValues.targetRight > 0)
          state = 1;
        else if (prevMotorValues.targetLeft < 0 && prevMotorValues.targetRight < 0)
          state = 2;
        else if (prevMotorValues.targetLeft != prevMotorValues.targetRight)
          state = 3;

        uint8_t mode = 0;
        rxData.status = (mode << 2) | state;
        rxData.crc = calculateRxCRC(rxData);
        rxData.successRate = (uint8_t)(getSuccessRate() * 255);
        rxData.request = RXREQ_NONE;

        // Write ACK payload for next transmission
        radio->writeAckPayload(0, &rxData, sizeof(RxData));

        return true;
      }

      // Checksum mismatch
      if (Serial)
      {
        Serial.println("Checksum error");
      }
      consecutiveFailures++;
    }
    else if (payloadSize == sizeof(CalibrationPacket))
    {
      // Handle calibration packet as before
      CalibrationPacket calib;
      radio->read(&calib, sizeof(CalibrationPacket));
      uint8_t expected = calcCalibrationChecksum(calib.xCenter, calib.yCenter);
      if (calib.checksum == expected &&
          calib.xCenter <= MAX_ADC_VALUE &&
          calib.yCenter <= MAX_ADC_VALUE)
      {
        xCenter = calib.xCenter;
        yCenter = calib.yCenter;
        if (Serial)
        {
          Serial.println("Calibration applied");
        }
        successBeep();
      }
      return false;
    }
    else
    {
      // Discard unexpected payload
      uint8_t dummy[32];
      radio->read(&dummy, payloadSize);
      return false;
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

void updatePacketHistory(bool success)
{
  if (success)
  {
    packetHistory |= (1UL << packetIndex);
  }
  else
  {
    packetHistory &= ~(1UL << packetIndex);
  }
  packetIndex = (packetIndex + 1) % PACKET_HISTORY_SIZE;
}

float getSuccessRate()
{
  uint32_t hist = packetHistory;
  int count = 0;
  for (int i = 0; i < PACKET_HISTORY_SIZE; i++)
  {
    if (hist & 1UL)
      count++;
    hist >>= 1;
  }
  return (float)count / PACKET_HISTORY_SIZE;
}

void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
  // Handshake: disable motors if a character is received from serial monitor
  static bool motorsDisabledBySerial = false;
  
#ifdef ARDUINO
  if (Serial && Serial.available())
  {
    char c = Serial.read(); // Read the character from serial monitor
    if (c == 'D')
    { // 'D' to disable motors (must be uppercase)
      motorsDisabledBySerial = true;
      stopMotors();
      Serial.println("Motors disabled by serial monitor handshake.");
    }
    else if (c == 'E')
    { // 'E' to re-enable motors (must be uppercase)
      motorsDisabledBySerial = false;
      Serial.println("Motors re-enabled.");
    }
  }
  
  // MOVED OUTSIDE: Check if motors are disabled
  if (motorsDisabledBySerial)
  {
    return;
  }
#endif

  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  static int prevLeftSpeed = 0;
  static int prevRightSpeed = 0;

  // // Add debug output FIRST (before motor control logic)
  // if (leftSpeed != 0 || rightSpeed != 0) {
  //   Serial.print(">>> MOTOR DEBUG: L=");
  //   Serial.print(leftSpeed);
  //   Serial.print(" R=");
  //   Serial.print(rightSpeed);
  //   Serial.println(" <<<");
  // }

  // Enforce MIN_MOTOR_SPEED for both forward/reverse and pure turning
  bool sameDirection = (leftSpeed > 0 && rightSpeed > 0) || (leftSpeed < 0 && rightSpeed < 0);
  bool pureTurning = (leftSpeed * rightSpeed < 0) && (abs(leftSpeed) >= MIN_MOTOR_SPEED || abs(rightSpeed) >= MIN_MOTOR_SPEED);
  if ((abs(leftSpeed) >= MIN_MOTOR_SPEED || abs(rightSpeed) >= MIN_MOTOR_SPEED) && (sameDirection || pureTurning))
  {
    // For both cases, ensure each side is at least MIN_MOTOR_SPEED if nonzero
    if (abs(leftSpeed) < MIN_MOTOR_SPEED && leftSpeed != 0)
      leftSpeed = (leftSpeed > 0) ? MIN_MOTOR_SPEED : -MIN_MOTOR_SPEED;
    if (abs(rightSpeed) < MIN_MOTOR_SPEED && rightSpeed != 0)
      rightSpeed = (rightSpeed > 0) ? MIN_MOTOR_SPEED : -MIN_MOTOR_SPEED;

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
      digitalWrite(LEFT_MOTOR_IN1, HIGH);
      digitalWrite(LEFT_MOTOR_IN2, LOW);
      analogWrite(LEFT_MOTOR_EN, leftSpeed);
    }
    else
    {
      digitalWrite(LEFT_MOTOR_IN1, LOW);
      digitalWrite(LEFT_MOTOR_IN2, HIGH);
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

void successBeep()
{
  tone(BUZZER_PIN, 2000, 100);
  delay(150); // Short delay between beeps
  tone(BUZZER_PIN, 2000, 100);
  delay(150);
  noTone(BUZZER_PIN);
}

void failBeep()
{
  tone(BUZZER_PIN, 1000, 100);
  delay(150);
  tone(BUZZER_PIN, 1000, 100);
  delay(150);
  tone(BUZZER_PIN, 500, 1000);
  delay(150);
  noTone(BUZZER_PIN);
}

// Entertainer melody function (simplified for space)
bool playEntertainerStep(EntertainerState &state, bool shouldInterrupt)
{
  // Your existing implementation
  return false; // Simplified for this optimization
}

void printStatusReport(const RxData &rxData, bool isRead, MotorTargets mt)
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
  if (!isRead)
  {
    Serial.print(" | *** NO SIGNAL *** | ");
  }

  // Current joystick values
  Serial.print(" | JS X:");
  Serial.print(pad4s(joystickX));
  Serial.print(" Y:");
  Serial.print(pad4s(joystickY));
  Serial.print(" Btn:");
  Serial.print(joystickButton ? " ON" : "OFF");
  Serial.print(" | Cal X:");
  Serial.print(pad4s(xCenter));
  Serial.print(" Y:");
  Serial.print(pad4s(yCenter));

  // Motor speeds
  Serial.print(" | Motors L:");
  Serial.print(pad4s(mt.targetLeft));
  Serial.print(" R:");
  Serial.print(pad4s(mt.targetRight));
  Serial.print(" OutL:");
  Serial.print(pad4s(mt.outputLeft));
  Serial.print(" OutR:");
  Serial.print(pad4s(mt.outputRight));
  Serial.print(" |");
  Serial.print(mt.skipSlewRate ? "     " : " Slew");
  Serial.print(mt.brakingApplied ? " Braking" : "        ");

  // Direction indicator
  Serial.print(" Dir:");
  String direction;
  if (mt.targetLeft == 0 && mt.targetRight == 0)
    direction = "STOP";
  else if (mt.targetLeft > 0 && mt.targetRight > 0)
    direction = "FWD";
  else if (mt.targetLeft < 0 && mt.targetRight < 0)
    direction = "REV";
  else if (mt.targetLeft > mt.targetRight)
    direction = "RIGHT";
  else if (mt.targetRight > mt.targetLeft)
    direction = "LEFT";
  else
    direction = "MIX";

  Serial.print(padString(direction, 6));

  // L/R Ratio
  Serial.print(" L/R PERC:");
  Serial.print(padString(String(calculateLeftRightPercent(joystickX)), 6));

  // Voltage (8bit => to actual voltage)
  Serial.print(" | Bat:");
  float batteryVoltage = rxData.voltage * (VOLTAGE_ADC_REFERENCE * VOLTAGE_DIVIDER_RATIO) / 255.0;
  Serial.print(pad5f(batteryVoltage));
  Serial.print("V");
  Serial.print(" (ADC: ");
  Serial.print(pad5f(map(rxData.voltage, 0, 255, 0, MAX_ADC_VALUE)));
  Serial.print(")");

  Serial.println("");
}

// Function to handle voltage reading with averaging and spike/drop detection
bool updateVoltageReading(RxData &rxData)
{
  static unsigned long lastVoltageReading = 0;
  static uint16_t voltageBuffer[4] = {0};
  static uint8_t bufferIndex = 0;
  static bool bufferFull = false;
  static uint16_t lastStableReading = 0;
  static bool firstReading = true;
  static uint8_t consecutiveOutliers = 0;

  // Check timing - return false if not time yet
  if (millis() - lastVoltageReading < 250)
  {
    return false;
  }

  // Take the ADC reading
  int voltageRaw = analogRead(VOLTAGE_ADC_PIN);
  bool useReading = true;

  // Check for spikes/drops BEFORE storing in buffer
  if (!firstReading)
  {
    int16_t deviation = (int16_t)voltageRaw - (int16_t)lastStableReading;
    if (abs(deviation) > 80)
    {
      consecutiveOutliers++;
      if (consecutiveOutliers >= 3)
      {
        useReading = true;
        consecutiveOutliers = 0;
        Serial.println("Persistent voltage change detected - accepting");
      }
      else
      {
        useReading = false;
        Serial.print("Voltage spike/drop detected (");
        Serial.print(consecutiveOutliers);
        Serial.print("/3): ");
        Serial.println(deviation);
      }
    }
    else
    {
      consecutiveOutliers = 0;
    }
  }

  uint16_t averagedRaw;

  if (useReading)
  {
    voltageBuffer[bufferIndex] = voltageRaw;
    bufferIndex = (bufferIndex + 1) % 4;
    if (bufferIndex == 0)
      bufferFull = true;

    uint32_t sum = 0;
    uint8_t count = bufferFull ? 4 : bufferIndex;
    for (uint8_t i = 0; i < count; i++)
    {
      sum += voltageBuffer[i];
    }
    averagedRaw = sum / count;
    lastStableReading = averagedRaw;
  }
  else
  {
    averagedRaw = lastStableReading;
  }

  uint8_t voltage8bit = map(averagedRaw, 0, MAX_ADC_VALUE, 0, 255);
  rxData.voltage = voltage8bit;

  firstReading = false;
  lastVoltageReading = millis();

  return true;
}

// Try to read a calibration packet if requested
bool tryReadCalibrationPacket()
{
  static unsigned long calibrationStart = millis();
  static bool calibrationTimedOut = false;
  const unsigned long CALIBRATION_TIMEOUT_MS = 2000;

  if (!calibrationTimedOut && rxData.request == RXREQ_REQUEST_CALIBRATION)
  {
    if (radio && radio->available())
    {
      if (radio->getDynamicPayloadSize() == sizeof(CalibrationPacket))
      {
        CalibrationPacket calib;
        radio->read(&calib, sizeof(CalibrationPacket));
        uint8_t expected = calcCalibrationChecksum(calib.xCenter, calib.yCenter);
        if (calib.checksum == expected)
        {
          xCenter = calib.xCenter;
          yCenter = calib.yCenter;
          rxData.request = RXREQ_NONE;
          Serial.print("Calibration received: xCenter=");
          Serial.print(xCenter);
          Serial.print(", yCenter=");
          Serial.println(yCenter);
          return true;
        }
      }
    }
    if (millis() - calibrationStart > CALIBRATION_TIMEOUT_MS)
    {
      rxData.request = RXREQ_NONE;
      calibrationTimedOut = true;
      Serial.println("Calibration request timed out. Using default center values.");
    }
  }
  return false;
}