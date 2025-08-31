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
void drawThrottle(int throttlePercent, int barX, int barY, int offsetX);
void drawLeftRightBar(int leftRightPercent, int barX, int barY, int offsetX);

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
    
    Serial.println();
    Serial.println("**********  RX data timeout - resetting values **********");
    Serial.println();
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
      
      // XY Axis + Button
      Serial.print("X Axis: "); Serial.print(x); Serial.print(" | ");
      Serial.print("Y Axis: "); Serial.print(y); Serial.print(" | ");
      Serial.print("Button: "); Serial.print(btnPressed ? "Pressed" : "Released"); Serial.print(" | ");

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

  // Line 3: Throttle indicator bar
  int throttlePercent = map(y, 0, 1023, 0, 100);
  drawThrottle(throttlePercent, 0, 18, offsetX);

  // Line 5: Left/Right percent (ratio) with fillbar
  int leftRightPercent = map(x, 0, 1023, -100, 100); // -100 = full left, 0 = center, 100 = full right
  drawLeftRightBar(leftRightPercent, 0, 26, offsetX);

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

void drawThrottle(int throttlePercent, int barX, int barY, int offsetX) {
  const int barH = 6;  // height of bar (reduced from 8)
  const int barW = 52; // total width of bar

  display->setCursor(barX, barY);
  display->print("THR");

  // Draw outline
  display->drawRect(barX + offsetX, barY, barW, barH, SSD1306_WHITE);

  // Fill proportional to throttlePercent
  int fillW = map(constrain(throttlePercent, 0, 100), 0, 100, 0, barW - 2);
  display->fillRect(barX + offsetX + 1, barY + 1, fillW, barH - 2, SSD1306_WHITE);
  
  display->setCursor(offsetX + barW + 2, barY);
  display->print(throttlePercent);
  display->print('%');
}

void drawLeftRightBar(int leftRightPercent, int barX, int barY, int offsetX) {
  const int barW = 52;    // total width of bar
  const int barH = 6;

  display->setCursor(barX, barY);
  display->print("L/R");

  // Draw outline
  display->drawRect(barX + offsetX, barY, barW, barH, SSD1306_WHITE);

  // Map leftRightPercent (-100..100) to fill position (0..barW)
  int fillPos = map(leftRightPercent, -100, 100, 0, barW);
  display->fillRect(barX + offsetX + 1, barY + 1, fillPos, barH - 2, SSD1306_WHITE);

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

