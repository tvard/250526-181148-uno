#ifdef ARDUINO
// Optimized version with memory-efficient NRF24 usage
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <RF24.h>
#endif

#include <helpers.h>
#include "../lib/helpers/display_helpers.h"
#include "2WD_RC_TRANSMITTER_logic.h"
#include "Arduino.h"

// Pin definitions
#define JOY_BUTTON_PIN A0
#define JOY_X_PIN A2
#define JOY_Y_PIN A1
#define VOLTAGE_SENSOR_PIN A3
#define NRF_CE_PIN 9
#define NRF_CSN_PIN 10

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define SCREEN_ADDRESS 0x3C
#define OLED_RESET_PIN -1

const int RADIO_CHANNEL = 76;
const byte addresses[][6] = {"00001", "00002"};

// Packet history globals (definition)
uint32_t packetHistory = 32;
uint8_t packetIndex = 0;

// Global objects - delay initialization to save startup RAM
// RF24* radio = nullptr;

RF24* radio = new RF24(NRF_CE_PIN, NRF_CSN_PIN);
bool radioInitialized = false;

Adafruit_SSD1306* display = nullptr;

// Calibration values
int16_t xCalib = 0, yCalib = 0;
bool displayReady = false;

// Actual definitions
int16_t xMin = 0, xMax = MAX_ADC_VALUE, yMin = 0, yMax = MAX_ADC_VALUE;
uint16_t xCenter = JOYSTICK_CENTER, yCenter = 494;

// Range expansion factors to be more aggressive in capturing full range
const int16_t RANGE_EXPANSION_FACTOR = 0; // More aggressive range expansion for better joystick response

// Function prototypes
int freeMemory();
void initDisplay();
void initRadio();
uint8_t calculateChecksum(const JoystickData& data);
void displayInfo(uint16_t x, uint16_t y, bool btnPressed, float txVoltage, float rxVoltage, float txSuccessRate, float rxSuccessRate);
void drawRssi(float successRate, int barX, int barY, bool showLabel, const char* suffix, int offsetX);
void drawBattery(float voltage, int barX, int barY, int barW, float maxVoltage, float minVoltage, bool showLabel, const char* suffix, int offsetX);
void drawThrottle(int x, int y, int barX, int barY, int offsetX);
void drawLeftRightBar(int x, int y, int barX, int barY, int offsetX);
void sendCalibReq();

// Calculation functions (separated from display for testing)
extern int calculateThrottlePercent(int y);
extern int calculateLeftRightPercent(int x);

// Free memory utility
int freeMemory() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void setup() {
  Serial.begin(9600);
  delay(1000);

  Serial.println("=== TRANSMITTER ===");
  Serial.print("Free memory at startup: ");
  Serial.println(freeMemory());

  pinMode(JOY_BUTTON_PIN, INPUT_PULLUP);
  
  // Initialize SPI and radio FIRST (when we have most RAM)
  SPI.begin();
  Serial.print("Free memory after SPI: ");
  Serial.println(freeMemory());
  
  initRadio();  // Move this before display
  
  // Initialize I2C and display AFTER radio
  Wire.begin();
  Wire.setClock(100000);
  Serial.print("Free memory after I2C: ");
  Serial.println(freeMemory());
  
  initDisplay();
  if (display) {
    display->println("OLED Ready.");
    display->display();
  }
  
  // Set min/max for percent mapping (accounting for Y-axis inversion in loop)
  xMin = -5;
  xMax = 1018;
  yMin = 0;    
  yMax = 1012; 

  // Calibration with fewer samples
  Serial.println("Quick calibration...");
  int32_t xSum = 0, ySum = 0;
  const int samples = 10; // Reduced from 40
  for (int i = 0; i < samples; i++) {
    xSum += analogRead(JOY_X_PIN);
    ySum += analogRead(JOY_Y_PIN);
    delay(20);
  }
  xCalib = JOYSTICK_CENTER - xSum / samples;
  yCalib = JOYSTICK_CENTER - ySum / samples;
  sendCalibReq();
  Serial.println(freeMemory());

  // Set stable center positions after calibration (use actual measured, post-inversion value)
  // xCenter = JOYSTICK_CENTER;  // X axis: keep as before
  // int rawYIdle = ySum / samples + yCalib;
  // int yIdleConstrained = constrain(rawYIdle, yMin, yMax);
  // int yIdleTransformed = yMax - yIdleConstrained; // Apply same transform as in loop
  // yCenter = 494;
  // Serial.print("[CALIB] rawYIdle: "); Serial.println(rawYIdle);
  // Serial.print("[CALIB] yIdleConstrained: "); Serial.println(yIdleConstrained);
  // Serial.print("[CALIB] yIdleTransformed (main loop y at rest): "); Serial.println(yIdleTransformed);
  Serial.print("[CALIB] yCenter set to: "); Serial.println(yCenter);
  
  // Debug output to verify the ranges
  Serial.print("Range settings - X: "); Serial.print(xMin); Serial.print(" to "); Serial.print(xMax);
  Serial.print(", Y: "); Serial.print(yMin); Serial.print(" to "); Serial.println(yMax);
  
  display->println("Calibration Complete");
  display->print("Final X Calibration: ");
  display->print((xCalib >= 0) ? "+" : "-");
  display->println(abs(xCalib));
  display->print("Final Y Calibration: ");
  display->print((yCalib >= 0) ? "+" : "-");
  display->println(abs(yCalib));
  display->display();

  Serial.print("Final free memory: ");
  Serial.println(freeMemory());
  Serial.println("Setup complete!");

  display->print("Setup Complete. Ready To Roll!");
  display->display();
}

void initDisplay() {
  Serial.println("Initializing display...");
  
  // Allocate display object dynamically only if we have enough memory
  if (freeMemory() > 200) { // Reduced threshold since radio is already init
    display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);
    
    if (display && display->begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      displayReady = true;
      Serial.print("Display OK! Free memory: ");
      Serial.println(freeMemory());
      
      display->clearDisplay();
      display->setTextSize(1);
      display->setTextColor(SSD1306_WHITE);
      display->setCursor(0, 0);
      display->display();
    } else {
      Serial.println("Display allocation failed");
      displayReady = false;
    }
  } else {
    Serial.print("Insufficient memory for display: ");
    Serial.println(freeMemory());
    displayReady = false;
  }
}

void initRadio() {
  Serial.println("Initializing radio...");
  
  // radio = new RF24(NRF_CE_PIN, NRF_CSN_PIN);
  if (radio && radio->begin()) {
    Serial.println("Radio initialized");
    radio->setChannel(RADIO_CHANNEL);
    radio->setDataRate(RF24_250KBPS);
    radio->setPALevel(RF24_PA_HIGH);           // Changed from HIGH to test
    radio->enableDynamicPayloads();
    radio->setAutoAck(true);                  // Re-enable ACK
    radio->enableAckPayload();                // Enable ACK payloads
    radio->setRetries(5, 5);                  // Restore retries
    radio->openReadingPipe(1, addresses[1]);
    radio->openWritingPipe(addresses[0]);     // Write on address "00001"
    radio->stopListening();
    radio->flush_rx();
    radio->flush_tx();
    radioInitialized = true;
    Serial.print("Radio configured. Free memory: ");
    Serial.println(freeMemory());
  } else {
    Serial.println("Radio initialization failed!");
    radioInitialized = false;
  }
}

// Tune send interval (ms)
const uint16_t CONTROL_SEND_INTERVAL_MS = 15;  // ~66 Hz
// Track next send time
static uint32_t nextControlSend = 0;
// Separate counters (optional)
static uint32_t txWrites = 0;
static uint32_t txAckGood = 0;
static uint32_t lastHwAckTime = 0;

void loop() {
  if (!radio) return;

  static float rxVoltage = 0.0f;
  static float rxSuccessRate = 0.0f;
  static uint32_t lastAckTime = 0;

  // Cached last transmitted packet & raw readings
  static JoystickData lastTxData = { JOYSTICK_CENTER, JOYSTICK_CENTER, 0, 0 };
  static int lastRawX = JOYSTICK_CENTER;
  static int lastRawY = JOYSTICK_CENTER;
  static bool lastBtn = false;

  uint32_t now = millis();
  bool timeToSend = (now >= nextControlSend);

  if (timeToSend) {
    nextControlSend = now + CONTROL_SEND_INTERVAL_MS;

    int rawX = analogRead(JOY_X_PIN);
    int rawY = analogRead(JOY_Y_PIN);
    bool btnPressed = (digitalRead(JOY_BUTTON_PIN) == LOW);

    int16_t x = constrain(rawX + xCalib, 0, 1023); 
    int16_t y = constrain(rawY + yCalib, yMin, yMax);
    y = yMax - (y - yMin);      // Proper inversion
    y = constrain(y, 0, 1023);  // Ensure final result is valid

    JoystickData txData;
    txData.xValue = x;
    txData.yValue = y;
    txData.buttonPressed = btnPressed;
    txData.checksum = calculateChecksum(txData);

    // Store for display / serial
    lastTxData = txData;
    lastRawX = rawX;
    lastRawY = rawY;
    lastBtn  = btnPressed;

    if (radioInitialized) {
      bool success = radio->write(&txData, sizeof(JoystickData));
      txWrites++;

      if (success) {
        lastHwAckTime = now;
        bool gotPayload = false;
        RxData ackData;
        while (radio->isAckPayloadAvailable()) {
          radio->read(&ackData, sizeof(RxData));
          gotPayload = true;
        }

        if (gotPayload) {
          uint8_t expectedCrc = (ackData.voltage ^ ackData.status) & 0xFF;
          if (ackData.crc == expectedCrc) {
            txAckGood++;
            rxVoltage     = ackData.voltage * (VOLTAGE_ADC_REFERENCE * VOLTAGE_DIVIDER_RATIO) / 255.0f;
            rxSuccessRate = (float)ackData.successRate / 255.0f;
            if (ackData.request == RXREQ_REQUEST_CALIBRATION) {
              display->clearDisplay();
              display->setCursor(0, 0);
              display->println("Calibration requested by Rx");
              display->display();
              sendCalibReq();
            }
            lastAckTime = now;               // payload-based timestamp
            updatePacketHistory(true);
          } else {
            // Payload CRC bad, still count hardware ACK as success
            updatePacketHistory(true);
          }
        } else {
          // Hardware ACK only (no payload). Count as success for RF quality.
          updatePacketHistory(true);
        }
      } else {
        // No hardware ACK
        updatePacketHistory(false);
      }
    } else {
      // Radio not initialized, treat as failure
      updatePacketHistory(false);
    }

    // Timeout display is only meaningful for payload-age; separate metric optional
    if (now - lastAckTime > 500) {
      rxVoltage = 0.0f;
      rxSuccessRate = 0.0f;
    }
  }

  // Display update
  static uint32_t lastDisplay = 0;
  if (displayReady && millis() - lastDisplay > 200) {
    float txVoltage = analogRead(VOLTAGE_SENSOR_PIN) * 5.0f / (float)MAX_ADC_VALUE + 0.04f;
    float txSuccessRate = getSuccessRate();
    displayInfo(lastTxData.xValue, lastTxData.yValue, lastTxData.buttonPressed,
                txVoltage, rxVoltage, txSuccessRate, rxSuccessRate);
    lastDisplay = millis();
  }

  if (Serial) {
    static uint32_t lastMemReport = 0;
    if (millis() - lastMemReport > 250) {
      bool payloadTimedOut = (millis() - lastAckTime > 500);
      bool hwTimedOut = (millis() - lastHwAckTime > 300);
      Serial.print("TX Success Rate: "); Serial.print(getSuccessRate() * 100); Serial.print("% | ");
      Serial.print("ACK Rx: ");
      Serial.print(hwTimedOut ? "*No HW ACK*" : (payloadTimedOut ? "No Payload" : "Payload OK")); Serial.print(" | ");
      Serial.print("X: "); Serial.print(lastTxData.xValue);
      Serial.print(" (ADC: "); Serial.print(lastRawX); Serial.print(") | ");
      Serial.print("Y: "); Serial.print(lastTxData.yValue);
      Serial.print(" (ADC: "); Serial.print(lastRawY); Serial.print(") | ");
      Serial.print("Btn: "); Serial.print(lastBtn ? "Pressed" : "Released"); Serial.print(" | ");
      Serial.print("RX V: "); Serial.print(rxVoltage); Serial.print(" | ");
      Serial.print("Writes: "); Serial.print(txWrites); Serial.print(" AckOK: "); Serial.print(txAckGood);
      // Serial.print(" ST=0x"); Serial.print((radio->getStatusFlags()), HEX);
      
      // Add these lines for proper status debugging:
      bool tx_ok, tx_fail, rx_ready;
      radio->whatHappened(tx_ok, tx_fail, rx_ready);
      Serial.print(" TX_OK="); Serial.print(tx_ok);
      Serial.print(" TX_FAIL="); Serial.print(tx_fail);
      Serial.print(" RX_RDY="); Serial.print(rx_ready);
      
      Serial.println();
      lastMemReport = millis();
    }
  }
}

// Update sendCalibReq to pause normal send pacing
void sendCalibReq() {
  CalibrationPacket calib;
  calib.xCenter = xCenter;
  calib.yCenter = yCenter;
  calib.checksum = calcCalibrationChecksum(calib.xCenter, calib.yCenter);

  // Briefly slow normal traffic
  uint32_t start = millis();
  for (int i = 0; i < 3; ++i) {
    radio->write(&calib, sizeof(CalibrationPacket));
    delay(8);
  }
  // Push next control send a bit forward to avoid immediate collision
  nextControlSend = millis() + 25;

  if (Serial) {
    Serial.print("Sent calibration packet: xCenter=");
    Serial.print(xCenter);
    Serial.print(" yCenter=");
    Serial.println(yCenter);
  }
}

// Calculate checksum for data integrity (MUST MATCH RECEIVER)
 uint8_t calculateChecksum(const JoystickData& data) {
  uint8_t checksum = 0;
  checksum ^= (data.xValue & 0xFF);
  checksum ^= ((data.xValue >> 8) & 0xFF);
  checksum ^= (data.yValue & 0xFF);
  checksum ^= ((data.yValue >> 8) & 0xFF);
  checksum ^= data.buttonPressed ? 0xFF : 0x00;
  return checksum;
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
  if (!radioInitialized) return 0.0f;
  uint32_t hist = packetHistory;
  int count = 0;
  for (int i = 0; i < PACKET_HISTORY_SIZE; i++) {
    if (hist & 1UL) count++;
    hist >>= 1;
  }
  return (float)count / PACKET_HISTORY_SIZE;
}

void displayInfo(uint16_t x, uint16_t y, bool btnPressed, float txVoltage, float rxVoltage, float txSuccessRate, float rxSuccessRate) {
  display->clearDisplay();
  display->setTextSize(1);
  display->setTextColor(SSD1306_WHITE);

  const int offsetX = 30; // Text prefix width

  // Line 1: Battery indicators
  drawBattery(txVoltage, 0, 1, 28, VBATT_MAX, VBATT_MIN, true, "tx", offsetX);              // Transmitter battery
  drawBattery(rxVoltage, 75, 1, 28, VBATT_MAX * 3.0f, VBATT_MIN * 3.0f, false, "rx", 0);    // Receiver battery - same size, closer label (3S[n]P config)

  // Line 2: RF Quality with signal bars and percentage
  drawRssi(txSuccessRate, 0, 10, true, "tx", offsetX);
  drawRssi(rxSuccessRate, 75, 10, false, "rx", 0);

  // Line 3: Throttle indicator bar (50% = center, 0% = full reverse, 100% = full forward)
  drawThrottle(x, y, 0, 18, offsetX);

  // Line 5: Left/Right percent (ratio) with fillbar
  drawLeftRightBar(x, y, 0, 26, offsetX);

  display->display();
}

void drawRssi(float rssi, int barX, int barY, bool showLabel, const char* suffix, int offsetX)
{
  if (showLabel) {
    display->setCursor(barX, barY);
    display->print("RSSI");
  }

  const int MAX_BARS = 6;
  const int BAR_WIDTH = 2;
  const int BAR_SPACING = 1;
  const int MAX_HEIGHT = 7;

  int bars = map(constrain(int(rssi * 100), 0, 100), 0, 100, 0, MAX_BARS);
  
  // Always show outline bars, but fill only the active ones
  for (int i = 0; i < MAX_BARS; i++) {
    // Scale heights progressively from 2 to MAX_HEIGHT
    int h = map(i + 1, 1, MAX_BARS, 2, MAX_HEIGHT);
    int barPosX = barX + offsetX + i * (BAR_WIDTH + BAR_SPACING);
    int barPosY = barY + 6 - h; // align bottom at barY

      // Filled bars for active signal strength
    display->fillRect(barPosX, barPosY, BAR_WIDTH, h, i < bars ? SSD1306_WHITE : SSD1306_BLACK);
    display->drawRect(barPosX, barPosY, BAR_WIDTH, h, i <= bars ? SSD1306_WHITE : SSD1306_BLACK);
  }

  // Fixed position for percentage text
  display->setCursor(barX + offsetX + (MAX_BARS * (BAR_WIDTH + BAR_SPACING)) + 2, barY);
  display->print(suffix);
}

void drawThrottle(int x, int y, int barX, int barY, int offsetX) {
  const int barW = 52;
  const int barH = 6;
  int throttlePercent = calculateThrottlePercent(y);
  FillBarResult r = calculateThrottleFillBar(throttlePercent, barW);
  display->setCursor(barX, barY);
  display->print("THR");
  display->drawRect(barX + offsetX, barY, barW, barH, SSD1306_WHITE);
  int centerX = barX + offsetX + (barW / 2);
  display->drawFastVLine(centerX, barY, barH, SSD1306_WHITE);
  if (r.fillWidth > 0) {
    if (throttlePercent > 0) {
      // Forward throttle: fill to the right of center
      display->fillRect(centerX + 1, barY + 1, r.fillWidth, barH - 2, SSD1306_WHITE);
    } else {
      // Reverse throttle: fill to the left of center
      display->fillRect(centerX - r.fillWidth, barY + 1, r.fillWidth, barH - 2, SSD1306_WHITE);
    }
  }
  display->setCursor(offsetX + barW + 2, barY);
  display->print(throttlePercent);
  display->print('%');
}

void drawLeftRightBar(int x, int y, int barX, int barY, int offsetX) {
  const int barW = 52;
  const int barH = 6;
  int leftRightPercent = calculateLeftRightPercent(x);
  FillBarResult r = calculateLeftRightFillBar(leftRightPercent, barW);
  // Serial.print("| [L/R] percent:"); Serial.print(leftRightPercent); Serial.print(" | ");
  // Serial.print("fillWidth:"); Serial.print(r.fillWidth); Serial.print(" | ");
  display->setCursor(barX, barY);
  display->print("L/R");
  display->drawRect(barX + offsetX, barY, barW, barH, SSD1306_WHITE);
  int centerX = barX + offsetX + (barW / 2);
  display->drawFastVLine(centerX, barY, barH, SSD1306_WHITE);
  if (r.fillWidth > 0) {
    if (leftRightPercent > 0) {
      display->fillRect(centerX + 1, barY + 1, r.fillWidth, barH - 2, SSD1306_WHITE);
    } else {
      display->fillRect(centerX - r.fillWidth, barY + 1, r.fillWidth, barH - 2, SSD1306_WHITE);
    }
  }
  display->setCursor(offsetX + barW + 2, barY);
  display->print(leftRightPercent);
  display->print('%');
}

void drawBattery(float voltage, int barX, int barY, int barW, float maxVoltage, float minVoltage, bool showLabel, const char* suffix, int offsetX) {
  const int barH = 6;

  int MAX_VOLTAGE = int(maxVoltage * 100); // maxVoltage in V
  int MIN_VOLTAGE = int(minVoltage * 100); // minVoltage in V

  // Map voltage to bar fill width with proper bounds checking
  int fillW = map(constrain(int(voltage * 100), MIN_VOLTAGE, MAX_VOLTAGE), 
                  MIN_VOLTAGE, MAX_VOLTAGE, 0, barW - 2);


  if (showLabel) {
    display->setCursor(barX, barY);
    display->print("BAT");
  }

  display->drawRect(barX + offsetX, barY, barW, barH, SSD1306_WHITE);
  display->fillRect(barX + offsetX + 1, barY + 1, fillW, barH - 2, SSD1306_WHITE);

  display->setCursor(barX + offsetX + (barW + 2), barY);
  display->print(suffix); // tx, rx...
}