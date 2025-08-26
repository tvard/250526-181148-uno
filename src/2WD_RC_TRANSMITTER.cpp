// Optimized version with memory-efficient NRF24L01 usage
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <RF24.h>

// Pin definitions
#define JOY_X_PIN A0
#define JOY_Y_PIN A1
#define JOY_BUTTON_PIN A2
#define VOLTAGE_SENSOR_PIN A3
#define NRF_CE_PIN 9
#define NRF_CSN_PIN 10

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define SCREEN_ADDRESS 0x3C
#define OLED_RESET_PIN -1

const int LOOP_DELAY_MS = 10; // Increased to reduce processing overhead
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
  uint8_t voltage;
  uint8_t status; // Combined mode + state
  uint8_t crc;
};

// 8x8 battery icon
const uint8_t batteryIcon[] PROGMEM = {
  0b01111110, // ###### 
  0b11111111, // ########
  0b10011001, // #  ##  #
  0b10011001, // #  ##  #
  0b10011001, // #  ##  #
  0b10011001, // #  ##  #
  0b11111111, // ########
  0b01111110  //  ######
};

// 8x8 radio signal icon
const uint8_t radioIcon[] PROGMEM = {
  0b00011000, //    ##
  0b00111100, //   ####
  0b01111110, //  ######
  0b11111111, // ########
  0b01111110, //  ######
  0b00111100, //   ####
  0b00011000, //    ##
  0b00000000  //        
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
void displayInfo(uint16_t x, uint16_t y, bool btnPressed, float batteryVoltage);
void drawBattery(float voltage, int barX, int barY, int barW, float maxVoltage);
void drawSignal(float successRate, int baseX, int baseY);
void drawThrottle(int throttlePercent, int barX, int barY);

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
  
  Serial.print("Final free memory: ");
  Serial.println(freeMemory());
  Serial.println("Setup complete!");
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
      display->println("Optimized RC TX");
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
    radio->setAutoAck(true);
    radio->setRetries(5, 5); // Reduced retries
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
  
  // Send data if radio available
  if (radio) {
    JoystickData txData;
    txData.xValue = x;
    txData.yValue = y; 
    txData.buttonPressed = btnPressed;
    txData.checksum = calculateChecksum(txData);
    
    radio->stopListening();
    bool success = radio->write(&txData, sizeof(JoystickData));
    radio->startListening();
    updatePacketHistory(success);
  }
  
  // Update display occasionally to save processing
  static uint32_t lastDisplay = 0;
  if (displayReady && millis() - lastDisplay > 200) { // 5Hz update
    float batteryVoltage = analogRead(VOLTAGE_SENSOR_PIN) * 5.0f / 1023.0f + 0.04f;
    displayInfo(x, y, btnPressed, batteryVoltage);
    lastDisplay = millis();
  }
  
  // Periodic memory report
  static uint32_t lastMemReport = 0;
  if (millis() - lastMemReport > 5000) {
    Serial.print("Free memory: ");  Serial.print(freeMemory());
    // NRF signal report
    Serial.print("Packet History: "); Serial.print(packetHistory, BIN); Serial.print(" | ");
    Serial.print("Packet Index: "); Serial.print(packetIndex); Serial.print(" | ");
    Serial.print("Success Rate: "); Serial.println(getSuccessRate() * 100);

    lastMemReport = millis();
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
  return (float)count / PACKET_HISTORY_SIZE;
}

void displayInfo(uint16_t x, uint16_t y, bool btnPressed, float batteryVoltage) {
  display->clearDisplay();
  display->setTextSize(1);
  display->setTextColor(SSD1306_WHITE);

  // Line 1: Battery indicators
  // Transmitter battery
  display->drawBitmap(0, 0, batteryIcon, 8, 8, SSD1306_WHITE);
  display->setCursor(9, 0);
  display->print('T');
  drawBattery(batteryVoltage, 15, 1, 35, 4.2f); 

  // Receiver battery placeholder (would come from RxData in full version)
  display->drawBitmap(60, 0, batteryIcon, 8, 8, SSD1306_WHITE);
  display->setCursor(69, 0);
  display->print('R');
  drawBattery(3.7f, 75, 1, 35, 12.6f); // Placeholder voltage

  // Line 2: RF Quality with signal bars and percentage
  display->setCursor(0, 9);
  display->drawBitmap(0, 9, radioIcon, 8, 8, SSD1306_WHITE);
  drawSignal(getSuccessRate(), 12, 16);  
  display->setCursor(100, 9);
  display->print(int(getSuccessRate() * 100));
  display->print("%");

  // Line 3: Throttle indicator bar
  int throttlePercent = map(y, 0, 1023, 0, 100);
  drawThrottle(throttlePercent, 0, 18);

  // Line 4: Joystick values and button status
  display->setCursor(0, 26);
  display->print("X:");
  display->print(x);
  display->print(" Y:");
  display->print(y);
  if (btnPressed) {
    display->print(" BTN");
  }

  display->display();
}

void drawThrottle(int throttlePercent, int barX, int barY) {
  const int barH = 6;  // height of bar (reduced from 8)
  const int barW = 52; // total width of bar
  const int offsetX = 40; // text prefix width

  display->setCursor(barX, barY);
  display->print("Thrott");

  // Draw outline
  display->drawRect(barX + offsetX, barY, barW, barH, SSD1306_WHITE);

  // Fill proportional to throttlePercent
  int fillW = map(constrain(throttlePercent, 0, 100), 0, 100, 0, barW - 2);
  display->fillRect(barX + offsetX + 1, barY + 1, fillW, barH - 2, SSD1306_WHITE);
  
  display->setCursor(offsetX + barW + 2, barY);
  display->print(throttlePercent);
  display->print('%');
}

void drawBattery(float voltage, int barX, int barY, int barW, float maxVoltage) {
  const int barH = 6;
  const int MIN_VOLTAGE = 280; // 2.8V * 100
  int MAX_VOLTAGE = int(maxVoltage * 100); // maxVoltage in V

  // Map voltage to bar fill width with proper bounds checking
  int fillW = map(constrain(int(voltage * 100), MIN_VOLTAGE, MAX_VOLTAGE), 
                  MIN_VOLTAGE, MAX_VOLTAGE, 0, barW - 2);

  display->drawRect(barX, barY, barW, barH, SSD1306_WHITE);
  display->fillRect(barX + 1, barY + 1, fillW, barH - 2, SSD1306_WHITE);
}

void drawSignal(float successRate, int baseX, int baseY) {
  const int MAX_BARS = 8;
  const int BAR_WIDTH = 2;
  const int BAR_SPACING = 1;
  const int MAX_HEIGHT = 8;

  int bars = map(constrain(int(successRate * 100), 0, 100), 0, 100, 0, MAX_BARS);

  for (int i = 0; i < MAX_BARS; i++) {
    // Scale heights progressively from 2 to MAX_HEIGHT
    int h = map(i + 1, 1, MAX_BARS, 2, MAX_HEIGHT);
    int x = baseX + i * (BAR_WIDTH + BAR_SPACING);
    int y = baseY - h; // align bottom at baseY

    if (i < bars) {
      display->fillRect(x, y, BAR_WIDTH, h, SSD1306_WHITE);
    } else {
      display->drawRect(x, y, BAR_WIDTH, h, SSD1306_WHITE);
    }
  }
}
