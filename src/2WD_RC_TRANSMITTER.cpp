// pro16MHzatmega328 pinout: https://protosupplies.com/wp-content/uploads/2020/10/Pro-Mini-Board-Pinout.jpg

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RF24.h>   // Include the RF24 library for NRF24L01
#include <SPI.h>

#define VOLTAGE_SENSOR_PIN A0

#define NRF_CE_PIN A0
#define NRF_CSN_PIN 10
#define NRF_SCK_PIN 13
#define NRF_MOSI_PIN 11
#define NRF_MISO_PIN 12

#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define OLED_RESET_PIN   -1   // Reset pin (or -1 if sharing Arduino reset pin)

#define JOY_X_PIN A0      // Joystick X-axis 
#define JOY_Y_PIN A1      // Joystick Y-axis 
#define JOY_BUTTON_PIN 16  // Joystick button 

const int LOOP_DELAY_MS = 1;   // how often we run the main loop


// Use shim instead of cast
Adafruit_SSD1306 display(128, 32, &Wire, OLED_RESET_PIN);

// Define the battery icon
const uint8_t batteryIcon[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00,
  0x1f, 0x00, 0x00, 0x00,
  0x1f, 0x80, 0x00, 0x00,
  0x1f, 0xc0, 0x00, 0x00,
  0x1f, 0xe0, 0x00, 0x00,
  0x1f, 0xf0, 0x00, 0x00,
  0x1f, 0xf8, 0x00, 0x00,
  0x1f, 0xfc, 0x00, 0x00,
};

const uint8_t radioIcon[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00,
  0x18, 0x00, 0x00, 0x00,
  0x18, 0x18, 0x00, 0x00,
  0x18, 0x18, 0x18, 0x00,
  0x18, 0x18, 0x18, 0x18,
  0x18, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
};


// Packed data structure (4 bytes total)
struct PackedDataTransmit {
  uint16_t x : 10;  // 10 bits for X (0-1023)
  uint16_t y : 10;  // 10 bits for Y (0-1023)
  uint8_t btn : 1;   // Button state
  uint8_t crc : 5;   // Simple checksum (5 bits)
};

// From receiver: battery voltage, movement state, mode
struct PackedDataReceive {
  uint16_t voltage : 8 ; // 8 bits for voltage (0-255)
  uint8_t mode : 2;      // 2 bits for mode (0-3)
  uint8_t state : 2;     // 2 bits for movement state (0-3)
  uint8_t crc : 4;       // 4 bits for CRC (0-15)
};


RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte address[6] = "00001";

// declare prototypes
void nrfSendData(PackedDataTransmit &data);
PackedDataReceive nrfReceiveData();
uint8_t calcCRCTransmit(PackedDataTransmit &data);
uint8_t calcCRCReceive(PackedDataReceive &data);
void displayInfo(PackedDataReceive &data, int throttlePercent);
float calculatePacketSuccessRate();
void scanI2C();

int calibratedXCenterDrift = 0;
int calibratedYCenterDrift = 0;

// Packet history for RF signal quality
// This will help us calculate the success rate of RF packets
const int PACKET_HISTORY_SIZE = 100;  // Store last 100 packets
bool packetHistory[PACKET_HISTORY_SIZE];
int packetIndex = 0;
float rfPacketSuccessRate = 0.0;



void setup() {
  Serial.begin(9600);
  Serial.println("Initializing...");

  pinMode(JOY_BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(); 

  // scanI2C();

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 alloc failed");
    for (;;); // Stop here
  }

  Serial.println("SSD1306 alloc success");

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Hello!");
  display.display();

  Serial.println("SSD1306 display() success");

  // Configure pins
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);  // Maximum power for longer range
  radio.stopListening();
  
  // Calibration (unchanged)
  long xSum = 0, ySum = 0;
  const int samples = 40; // Fewer samples for faster startup
  Serial.println("Calibrating...");
  for (int i = 0; i < samples; ++i) {
    xSum += analogRead(JOY_X_PIN);
    ySum += analogRead(JOY_Y_PIN);
    delay(25);
  }

  calibratedXCenterDrift = 512 - xSum / samples;
  calibratedYCenterDrift = 512 - ySum / samples;
  Serial.print("Calibration X:");
  Serial.print(calibratedXCenterDrift);
  Serial.print(" Y:");
  Serial.println(calibratedYCenterDrift);
}

void loop() {
  // Read and calibrate
  int rawX = analogRead(JOY_X_PIN) + calibratedXCenterDrift;
  int rawY = analogRead(JOY_Y_PIN) + calibratedYCenterDrift;
  bool buttonPressed = (digitalRead(JOY_BUTTON_PIN) == LOW);
  
  // Constrain values to valid range
  uint16_t xValue = constrain(rawX, 0, 1023);
  uint16_t yValue = constrain(rawY, 0, 1023);

  // Prepare packed data with CRC
  PackedDataTransmit rData;
  rData.x = xValue;
  rData.y = yValue;
  rData.btn = buttonPressed;
  rData.crc = calcCRCTransmit(rData);
  int throttlePercent = map(yValue, 0, 1023, 0, 100); // Map Y value to throttle percentage

  // Send data via NRF24L01
  nrfSendData(rData);

  // receive data from NRF24L01: battery voltage, movement state, mode
  PackedDataReceive data = nrfReceiveData();
  displayInfo(data, throttlePercent);

  // Debug output
  Serial.print("X:");
  Serial.print(xValue);
  Serial.print(" Y:");
  Serial.print(yValue);
  Serial.print(" Btn:");
  Serial.print(buttonPressed);
  Serial.print(" CRC:");
  Serial.println(data.crc, BIN);

  delay(LOOP_DELAY_MS); // Overall loop delay
}

// function to read information and dislay it in oled module
/*
Signal Strength (RSSI-style bar + percentage)
Battery Voltage (Transmitter + Receiver if available)
Drive Mode (e.g. SLEW/FAST + FORW/REV/STOP/LEFT/RIGHT)
Throttle Position (0-100% bar)
Packet Success Rate (Transmission reliability)
*/
void displayInfo(PackedDataReceive &data, int throttlePercent) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Line 1: Battery Voltages (y=0)
  display.setCursor(0, 0);
  display.drawBitmap(0, 0, batteryIcon, 8, 8, SSD1306_WHITE);
  float receiverVoltage = map(data.voltage, 0, 255, 0, 4700) / 1000.0f;
  float transmitterVoltage = map(analogRead(VOLTAGE_SENSOR_PIN), 0, 1023, 0, 4700) / 1000.0f;
  display.print(receiverVoltage, 1);
  display.print("V/");
  display.print(transmitterVoltage, 1);
  display.print("V");

  // Line 2: RF Quality & Mode (y=8)
  display.setCursor(0, 8);
  display.drawBitmap(0, 8, radioIcon, 8, 8, SSD1306_WHITE);
  display.print(int(rfPacketSuccessRate * 100));
  display.print("% ");
  display.print(data.mode ? "FWD" : "REV");

  // Line 3: Throttle (y=16)
  display.setCursor(0, 16);
  display.print("Throttle: ");
  display.print(throttlePercent);
  display.print("%");

  // Line 4: Reserved for future use (y=24)

  display.display();
}
// Simple CRC calculation (XOR-based)
uint8_t calcCRCTransmit(PackedDataTransmit &data) {
  return (data.x ^ data.y ^ data.btn) & 0x1F; // Use 5-bit CRC
}

// Simple CRC calculation (XOR-based)
uint8_t calcCRCReceive(PackedDataReceive &data) {
  return (data.voltage ^ data.mode ^ data.state) & 0x1F; // Use 5-bit CRC
}

// Send joystick data via NRF24L01
void nrfSendData(PackedDataTransmit &data) {
    data.crc = calcCRCTransmit(data);
    
    // Store transmission result in history
    bool success = radio.write(&data, sizeof(PackedDataTransmit));
    packetHistory[packetIndex] = success;
    packetIndex = (packetIndex + 1) % PACKET_HISTORY_SIZE;
    
    // Update success rate
    rfPacketSuccessRate = calculatePacketSuccessRate();
    
    if (!success) {
        Serial.println("NRF24L01 send failed!");
    }
}

//receive data from NRF24L01: battery voltage, movement state, mode
PackedDataReceive nrfReceiveData() {
    PackedDataReceive data;
    if (radio.available()) {
        radio.read(&data, sizeof(PackedDataReceive));
        // Verify CRC
        if (data.crc == calcCRCReceive(data)) {
            return data; // Valid data, return it
        } else {
            Serial.println("CRC mismatch!");
        }
    }
    return {0, 0, 0, 0}; // Return empty data if no valid packet received
}

// Add this function to calculate success rate
float calculatePacketSuccessRate() {
    int successCount = 0;
    for(int i = 0; i < PACKET_HISTORY_SIZE; i++) {
        if(packetHistory[i]) successCount++;
    }
    return (float)successCount / PACKET_HISTORY_SIZE;
}

void scanI2C() {
  byte error, address;
  int devices = 0;
  
  Serial.println("Scanning I2C...");
  for(address = 1; address < 127; address++) {
    Serial.print("Scanning address: ");
    Serial.print(address);
    Wire.beginTransmission(address);
    Serial.print(" Beginning trans... ");
    error = Wire.endTransmission();
    Serial.println(" Ending trans... ");
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      return; // assume 1 connection only ( remove if multiple devices)
      devices++;
    }
  }
  if (devices == 0) Serial.println("No I2C devices found");
}