// Optimized version with memory-efficient NRF24L01 usage
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <RF24.h>
#include <helpers.h>

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

// Packet structures - MUST MATCH RECEIVER EXACTLY
struct JoystickData {
  int xValue;           // 16-bit int (matches receiver)
  int yValue;           // 16-bit int (matches receiver)  
  bool buttonPressed;   // 8-bit bool (matches receiver)
  uint8_t checksum;     // 8-bit checksum (matches receiver)
};

struct __attribute__((packed)) RxData {
  uint8_t voltage;        // Battery voltage (0-255)
  uint8_t successRate;    // Rx success rate (0-255)
  uint8_t status;         // Combined mode + state info
  uint8_t crc;            // Simple CRC
};

// Global objects - delay initialization to save startup RAM
RF24* radio = nullptr;
Adafruit_SSD1306* display = nullptr;

// Minimal packet history (reduced from 100 to 10)
const int PACKET_HISTORY_SIZE = 10;
uint16_t packetHistory = 0; // Use bitfield instead of array
uint8_t packetIndex = 0;

// Calibration values
int16_t xCalib = 0, yCalib = 0;
bool displayReady = false;

// Dynamic range tracking for better mapping
uint16_t xMin = 512, xMax = 512; // Start at center, expand as we see actual values
uint16_t yMin = 512, yMax = 512;

// Stable center positions (set during calibration)
uint16_t xCenter = 512, yCenter = 512;

// Range expansion factors to be more aggressive in capturing full range
const uint16_t RANGE_EXPANSION_FACTOR = 50; // More aggressive range expansion for better joystick response

// Function prototypes
int freeMemory();
void initDisplay();
void initRadio();
uint8_t calculateChecksum(const JoystickData& data);
void updatePacketHistory(bool success);
float getSuccessRate();
void displayInfo(uint16_t x, uint16_t y, bool btnPressed, float txVoltage, float rxVoltage, float txSuccessRate, float rxSuccessRate);
void drawRssi(float successRate, int barX, int barY, bool showLabel, const char* suffix, int offsetX);
void drawBattery(float voltage, int barX, int barY, int barW, float maxVoltage, bool showLabel, const char* suffix, int offsetX);
void drawThrottle(int x, int y, int barX, int barY, int offsetX);
void drawLeftRightBar(int x, int y, int barX, int barY, int offsetX);

// Calculation functions (separated from display for testing)
int calculateThrottlePercent(int y);
int calculateLeftRightPercent(int x);

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
  
  // Initialize I2C and display first (when we have most RAM)
  Wire.begin();
  Wire.setClock(100000);
  Serial.print("Free memory after I2C: ");
  Serial.println(freeMemory());
  
  initDisplay();
  display->println("OLED Ready.");
  display->display();
  
  // Initialize SPI and radio (more memory-hungry)
  SPI.begin();
  Serial.print("Free memory after SPI: ");
  Serial.println(freeMemory());
  
  initRadio();
  
  // Calibration with fewer samples
  Serial.println("Quick calibration...");
  int32_t xSum = 0, ySum = 0;
  const int samples = 10; // Reduced from 40
  for (int i = 0; i < samples; i++) {
    xSum += analogRead(JOY_X_PIN);
    ySum += analogRead(JOY_Y_PIN);
    delay(20);
  }
  xCalib = 512 - xSum / samples;
  yCalib = 512 - ySum / samples;

  // Set stable center positions after calibration
  xCenter = 512;  // This is our target center after calibration
  yCenter = 512;  // This is our target center after calibration
  
  // Initialize range with full ADC range for maximum responsiveness
  // Use the absolute extremes with tiny margins only for electrical noise
  xMin = 10;       // Almost full range
  xMax = 1013;     // Almost full range
  yMin = 10;       // Almost full range
  yMax = 1013;     // Almost full range

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
  if (freeMemory() > 600) { // Need ~514 bytes for buffer + overhead
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
      if (display) {
        delete display;
        display = nullptr;
      }
    }
  } else {
    Serial.print("Insufficient memory for display: ");
    Serial.println(freeMemory());
  }
}

void initRadio() {
  Serial.println("Initializing radio...");
  
  // Allocate radio object dynamically
  radio = new RF24(NRF_CE_PIN, NRF_CSN_PIN);
  
  if (radio && radio->begin()) {
    Serial.println("Radio initialized");
    radio->setChannel(RADIO_CHANNEL);
    radio->setDataRate(RF24_250KBPS);
    radio->setPALevel(RF24_PA_HIGH);
    radio->setPayloadSize(sizeof(JoystickData));
    radio->setAutoAck(true);                // Re-enable ACK
    radio->enableAckPayload();              // Enable ACK payloads
    radio->setRetries(5, 5);                // Restore retries
    radio->openWritingPipe(addresses[0]);    // Write on address "00001"
    radio->openReadingPipe(0, addresses[1]); // Listen on address "00002"
    radio->startListening();
    Serial.print("Radio configured. Free memory: ");
    Serial.println(freeMemory());
  } else {
    Serial.println("Radio initialization failed!");
  }
}

void loop() {
  // Read joystick
  int rawX = analogRead(JOY_X_PIN) + xCalib;
  int rawY = analogRead(JOY_Y_PIN) + yCalib;
  bool btnPressed = (digitalRead(JOY_BUTTON_PIN) == LOW);
  
  uint16_t x = constrain(rawX, 0, 1023);
  uint16_t y = constrain(rawY, 0, 1023);
  y = 1023 - y; // Invert Y axis

  // Update dynamic range tracking for better percentage mapping
  if (x < xMin) {
    xMin = (uint16_t)max(0, (int)x - RANGE_EXPANSION_FACTOR);
  }
  if (x > xMax) {
    xMax = min(1023, (int)x + RANGE_EXPANSION_FACTOR); // Expand beyond current value
  }
  if (y < yMin) {
    yMin = max(0, (int)y - RANGE_EXPANSION_FACTOR);  // Expand beyond current value
  }
  if (y > yMax) {
    yMax = min(1023, (int)y + RANGE_EXPANSION_FACTOR); // Expand beyond current value
  }

  static float rxVoltage = 0.0f;
  static float rxSuccessRate = 0.0f;
  static uint32_t lastAckTime = 0;

  // Send data continuously if radio is available
  if (radio) {
    JoystickData txData;
    txData.xValue = x;
    txData.yValue = y; 
    txData.buttonPressed = btnPressed;
    txData.checksum = calculateChecksum(txData);
    
    radio->stopListening();
    bool success = radio->write(&txData, sizeof(JoystickData));
    
    // Check for ACK payload from receiver
    if (success && radio->isAckPayloadAvailable()) {
      RxData ackData;
      radio->read(&ackData, sizeof(RxData));
      
      // Validate CRC of received ACK payload
      uint8_t expectedCrc = (ackData.voltage ^ ackData.status) & 0xFF;
      if (ackData.crc == expectedCrc) {
        // Process valid receiver data using shared voltage calibration constants
        // ackData.voltage is scaled 0-255, convert to actual battery voltage
        rxVoltage = map(ackData.voltage, 0, 255, 0, (int)(VOLTAGE_CALIBRATION_BATTERY * 100)) / 100.0f;
        rxSuccessRate = (float)ackData.successRate / 255.0f;        // Scale back to 0-1 (fraction)

        uint8_t receiverMode = (ackData.status >> 2) & 0x03;
        uint8_t receiverState = ackData.status & 0x03;
        
        // Use the values to avoid warning (can be expanded later for mode display)
        (void)receiverMode;   // Suppress unused variable warning
        (void)receiverState;  // Suppress unused variable warning
        
        lastAckTime = millis(); // Update timestamp when we receive valid ACK
      }
    }
    
    radio->startListening();
    updatePacketHistory(success);
  }

  // Check for timeout - reset RX values if no ACK received for n milliseconds (200ms is typical for RF modules)
  if (millis() - lastAckTime > 200) {
    rxVoltage = 0.0f;
    rxSuccessRate = 0.0f;
  }
  
  // Update display occasionally to save processing
  static uint32_t lastDisplay = 0;
  if (displayReady && millis() - lastDisplay > 200) { // 5Hz update
    float txVoltage = analogRead(VOLTAGE_SENSOR_PIN) * 5.0f / 1023.0f + 0.04f;
    float txSuccessRate = getSuccessRate();
    displayInfo(x, y, btnPressed, txVoltage, rxVoltage, txSuccessRate, rxSuccessRate);
    lastDisplay = millis();
  }

  if (Serial) { // Check if serial is connected (monitored by PC)
    // Periodic memory report

    static uint32_t lastMemReport = 0;
    if (millis() - lastMemReport > 1000) {
      Serial.print(" | Free memory: ");  Serial.print(freeMemory()); Serial.print(" | ");

      // NRF signal report
      Serial.print("Packet History: "); Serial.print(packetHistory, BIN); Serial.print(" | ");
      Serial.print("Packet Index: "); Serial.print(packetIndex); Serial.print(" | ");
      Serial.print(" | TX Success Rate: "); Serial.print(" | ");
      Serial.print(getSuccessRate() * 100);
      Serial.print("% | ACK Rx: ");
      Serial.print(millis() - lastAckTime > 200 ? "*Timeout*" : "Active"); Serial.print(" | ");

      // XY Axis + Button with range info
      Serial.print("X: "); Serial.print(x); 
      Serial.print(" ("); Serial.print(xMin); Serial.print("-"); Serial.print(xMax); Serial.print(")");
      Serial.print(" L/R: "); Serial.print(calculateLeftRightPercent(x)); Serial.print("%");
      Serial.print(" | Y: "); Serial.print(y);
      Serial.print(" ("); Serial.print(yMin); Serial.print("-"); Serial.print(yMax); Serial.print(")");
      Serial.print(" THR: "); Serial.print(calculateThrottlePercent(y)); Serial.print("%");
      Serial.print(" | Button: "); Serial.print(btnPressed ? "Pressed" : "Released"); Serial.print(" | ");

      // Voltage
      Serial.print("TX Voltage: ");
      Serial.print(analogRead(VOLTAGE_SENSOR_PIN) * 5.0f / 1023.0f + 0.04f);
      Serial.print(" (");
      Serial.print(analogRead(VOLTAGE_SENSOR_PIN));
      Serial.print(")"); Serial.print(" | ");

      Serial.println("");

      lastMemReport = millis();
    }
  }
  
  delay(LOOP_DELAY_MS);
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
    packetHistory |= (1 << packetIndex);
  } else {
    packetHistory &= ~(1 << packetIndex);
  }
  packetIndex = (packetIndex + 1) % PACKET_HISTORY_SIZE;
}

float getSuccessRate() {
  int count = 0;
  for (int i = 0; i < PACKET_HISTORY_SIZE; i++) {
    if (packetHistory & (1 << i)) count++;
  }
  return (float)count / PACKET_HISTORY_SIZE; // Success rate as a fraction
}

// Calculation functions (separated from display for testing)
int calculateThrottlePercent(int y) {
  float throttlePercent;
  
  // Use float math for better precision, especially important for small movements
  if (y > yCenter) {
    // Forward: map from center to max -> 50% to 100%
    throttlePercent = 50.0f + ((float)(y - yCenter) / (float)(yMax - yCenter)) * 50.0f;
  } else {
    // Reverse: map from min to center -> 0% to 50%
    throttlePercent = ((float)(y - yMin) / (float)(yCenter - yMin)) * 50.0f;
  }
  
  // Very restrictive deadzone: only apply if BOTH percentage is tiny AND raw input is very close to center
  // This should prevent any extreme positions from being snapped to center
  bool nearCenterRaw = abs(y - (int)yCenter) < 15;  // Raw input within 15 ADC units of center (very tight)
  bool nearCenterPercent = abs(throttlePercent - 50.0f) <= 2.0f;  // Percentage within 48%-52% (very tight)
  
  if (nearCenterRaw && nearCenterPercent) {
    throttlePercent = 50.0f; // Only force to center if both conditions are true
  }
  
  return constrain((int)round(throttlePercent), 0, 100);
}

int calculateLeftRightPercent(int x) {
  float leftRightPercent;
  
  // Use float math for better precision, especially important for small movements
  if (x > xCenter) {
    // Right: map from center to max -> 0% to +100%
    leftRightPercent = ((float)(x - xCenter) / (float)(xMax - xCenter)) * 100.0f;
  } else {
    // Left: map from min to center -> -100% to 0%
    leftRightPercent = ((float)(x - xMin) / (float)(xCenter - xMin)) * 100.0f - 100.0f;
  }
  
  // Very restrictive deadzone: only apply if BOTH percentage is tiny AND raw input is very close to center
  // This should prevent any extreme positions from being snapped to center
  bool nearCenterRaw = abs(x - (int)xCenter) < 15;  // Raw input within 15 ADC units of center (very tight)
  bool nearCenterPercent = abs(leftRightPercent) <= 2.0f;  // Percentage within ±2% (very tight)
  
  if (nearCenterRaw && nearCenterPercent) {
    leftRightPercent = 0.0f; // Only force to center if both conditions are true
  }
  
  return constrain((int)round(leftRightPercent), -100, 100);
}

void displayInfo(uint16_t x, uint16_t y, bool btnPressed, float txVoltage, float rxVoltage, float txSuccessRate, float rxSuccessRate) {
  display->clearDisplay();
  display->setTextSize(1);
  display->setTextColor(SSD1306_WHITE);

  const int offsetX = 30; // Text prefix width

  // Line 1: Battery indicators
  drawBattery(txVoltage, 0, 1, 28, 4.2f, true, "tx", offsetX);   // Transmitter battery
  drawBattery(rxVoltage, 75, 1, 28, 12.6f, false, "rx", 0);      // Receiver battery - same size, closer label

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
  const int barH = 6;  // height of bar
  const int barW = 52; // total width of bar

  // Use the separated calculation function
  int throttlePercent = calculateThrottlePercent(y);

  display->setCursor(barX, barY);
  display->print("THR");

  // Draw outline
  display->drawRect(barX + offsetX, barY, barW, barH, SSD1306_WHITE);

  // Draw center line at 50%
  int centerX = barX + offsetX + (barW / 2);
  display->drawFastVLine(centerX, barY, barH, SSD1306_WHITE);

  // Fill based on throttle percentage (50% = center)
  if (throttlePercent > 50) {
    // Forward: fill from center to right
    int fillW = map(throttlePercent, 50, 100, 0, (barW / 2) - 1);
    fillW = constrain(fillW, 0, (barW / 2) - 1);
    display->fillRect(centerX + 1, barY + 1, fillW, barH - 2, SSD1306_WHITE);
  } else if (throttlePercent < 50) {
    // Reverse: fill from center to left (NOT from left edge)
    int fillW = map(throttlePercent, 0, 50, (barW / 2) - 1, 0);
    fillW = constrain(fillW, 0, (barW / 2) - 1);
    display->fillRect(centerX - fillW, barY + 1, fillW, barH - 2, SSD1306_WHITE);
  }
  // At exactly 50%, no fill (just center line)
  
  display->setCursor(offsetX + barW + 2, barY);
  display->print(throttlePercent);
  display->print('%');
}

void drawLeftRightBar(int x, int y, int barX, int barY, int offsetX) {
  const int barW = 52;    // total width of bar
  const int barH = 6;

  // Use the separated calculation function
  int leftRightPercent = calculateLeftRightPercent(x);

  display->setCursor(barX, barY);
  display->print("L/R");

  // Draw outline
  display->drawRect(barX + offsetX, barY, barW, barH, SSD1306_WHITE);

  // Draw center line at 0%
  int centerX = barX + offsetX + (barW / 2);
  display->drawFastVLine(centerX, barY, barH, SSD1306_WHITE);

  // Fill based on left/right percentage (-100% to +100%, 0% = center)
  if (leftRightPercent > 0) {
    // Right: fill from center to right
    int fillW = map(leftRightPercent, 0, 100, 0, (barW / 2) - 1);
    fillW = constrain(fillW, 0, (barW / 2) - 1);
    display->fillRect(centerX + 1, barY + 1, fillW, barH - 2, SSD1306_WHITE);
  } else if (leftRightPercent < 0) {
    // Left: fill from center to left (NOT from left edge)
    int fillW = map(-leftRightPercent, 0, 100, 0, (barW / 2) - 1);
    fillW = constrain(fillW, 0, (barW / 2) - 1);
    display->fillRect(centerX - fillW, barY + 1, fillW, barH - 2, SSD1306_WHITE);
  }
  // At exactly 0%, no fill (just center line)

  display->setCursor(offsetX + barW + 2, barY);
  display->print(leftRightPercent);
  display->print('%');
}

void drawBattery(float voltage, int barX, int barY, int barW, float maxVoltage, bool showLabel, const char* suffix, int offsetX) {
  const int barH = 6;

  const int MIN_VOLTAGE = 280; // 2.8V * 100
  int MAX_VOLTAGE = int(maxVoltage * 100); // maxVoltage in V

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

