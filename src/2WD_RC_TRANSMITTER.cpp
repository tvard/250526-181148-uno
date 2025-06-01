// pro16MHzatmega328 pinout: https://protosupplies.com/wp-content/uploads/2020/10/Pro-Mini-Board-Pinout.jpg

#include <RH_ASK.h> // Include the RadioHead library
#include <SPI.h>    // Include the SPI library (required for RadioHead)

// Define Arduino Pro Mini Pins
const int JOY_X_PIN = A0;       // Joystick X-axis 
const int JOY_Y_PIN = A1;       // Joystick Y-axis 
const int JOY_BUTTON_PIN = 16;  // Joystick button 

// Create an instance of the rf_driver radio object with optimized settings
RH_ASK rf_driver(1000, 11, 12); // TX=11, RX=12 (RX unused in transmitter)

// Packed data structure (4 bytes total)
struct PackedData {
  uint16_t x : 10;  // 10 bits for X (0-1023)
  uint16_t y : 10;  // 10 bits for Y (0-1023)
  uint8_t btn : 1;   // Button state
  uint8_t crc : 5;   // Simple checksum (5 bits)
};

int calibratedXCenterDrift = 0;
int calibratedYCenterDrift = 0;

// Simple CRC calculation (XOR-based)
uint8_t calcCRC(uint16_t x, uint16_t y, bool btn) {
  return (x ^ y ^ btn) & 0x1F; // Use 5-bit CRC
}

void setup() {
  pinMode(JOY_BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(9600);
  
  // Initialize with critical optimizations
  if (rf_driver.init()) {
    rf_driver.crc();                   // Enable CRC checking
    rf_driver.setPreambleLength(16);   // Longer preamble for sync
    rf_driver.setModeIdle();           // Prevent spurious transmissions
    Serial.println("Radio initialized!");
  } else {
    Serial.println("Radio init failed!");
    while(1);
  }

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
  // 1. Read and calibrate
  int rawX = analogRead(JOY_X_PIN) + calibratedXCenterDrift;
  int rawY = analogRead(JOY_Y_PIN) + calibratedYCenterDrift;
  bool buttonPressed = (digitalRead(JOY_BUTTON_PIN) == LOW);
  
  // Constrain values to valid range
  uint16_t xValue = constrain(rawX, 0, 1023);
  uint16_t yValue = constrain(rawY, 0, 1023);

  // 2. Prepare packed data with CRC
  PackedData data;
  data.x = xValue;
  data.y = yValue;
  data.btn = buttonPressed;
  data.crc = calcCRC(xValue, yValue, buttonPressed);

  // 3. Send with triple retransmission
  for (int i = 0; i < 3; i++) {  // Send 3 times for redundancy
    rf_driver.send((uint8_t*)&data, sizeof(data));
    rf_driver.waitPacketSent();
    delay(5); // Short delay between transmits
  }

  // 4. Debug output
  Serial.print("X:");
  Serial.print(xValue);
  Serial.print(" Y:");
  Serial.print(yValue);
  Serial.print(" Btn:");
  Serial.print(buttonPressed);
  Serial.print(" CRC:");
  Serial.println(data.crc, BIN);

  delay(30); // Overall loop delay
}