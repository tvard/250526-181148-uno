#include <RH_ASK.h> // Include the RadioHead library
#include <SPI.h>    // Include the SPI library (required for RadioHead)

// Define Arduino Pro Mini Pins
const int JOY_X_PIN = A0;       // Joystick X-axis 
const int JOY_Y_PIN = A1;       // Joystick Y-axis 
const int JOY_BUTTON_PIN = 16;  // Joystick button 
// const int RF_DATA_PIN = 11;    // RFM69 CS (Chip Select) pin 

// Create an instance of the rf_driver radio object, at a certain bps rate (slower = longer range, faster = less range)
RH_ASK rf_driver(500); // Using pin 12 as RX (unused in transmitter) and RF_DATA_PIN as TX

// Define the structure for the data we want to send
struct JoystickData {
  int xValue;
  int yValue;
  bool buttonPressed;
};

const int digitalStart = 2;
const int digitalEnd = 13;

const int analogStart = A0;
const int analogEnd = A5;

void setup() {
 pinMode(JOY_BUTTON_PIN, INPUT_PULLUP); // Set joystick button pin as input with pullup

  Serial.begin(9600);         // Start serial communication for debugging

  while (!Serial); // Wait for serial connection (safe on most serial adapters)

  // Initialize the rf_driver radio module
  if (!rf_driver.init()) {
    Serial.println("rf_driver initialization failed!");
    while (1); // Stop if the radio doesn't initialize
  }

  Serial.println("rf_driver initialization successful!");
}


void loop() {
  // 1. Read Joystick and Button
  int xValue = analogRead(JOY_X_PIN);     // Read X-axis value (0-1023)
  int yValue = analogRead(JOY_Y_PIN);     // Read Y-axis value (0-1023)
  bool buttonPressed = digitalRead(JOY_BUTTON_PIN) == LOW; // LOW if pressed (due to pullup)

  // 2. Prepare Data to Send
  JoystickData data; // Create an instance of our data structure
  data.xValue = xValue;
  data.yValue = yValue;
  data.buttonPressed = buttonPressed;

  // 3. Send the Data
  // Send the data as a structured object.  This is much cleaner and more efficient.

  // Attempt to send data
  if (rf_driver.send((uint8_t *)&data, sizeof(data))) {
    // Serial.println("Data queued for transmission.");
    
    // Wait for transmission to complete (with timeout)
    unsigned long startTime = millis();
    while (!rf_driver.waitPacketSent()) {
      if (millis() - startTime > 2000) { // Timeout after 2 seconds
        Serial.println("Error: Transmission timed out!");
        break;
      }
    }
    // Serial.println("Data transmitted successfully!");
  } else {
    Serial.println("Error: Failed to queue data!");
  }


  // 4. Print to Serial Monitor (for debugging)
  Serial.print("X: ");
  Serial.print(xValue);
  Serial.print(", Y: ");
  Serial.print(yValue);
  Serial.print(", Button: ");
  Serial.println(buttonPressed);

  delay(25); // Small delay before the next reading
}

