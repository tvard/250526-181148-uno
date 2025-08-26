// Memory test version - minimal setup to test SSD1306 allocation
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define SCREEN_ADDRESS 0x3C
#define OLED_RESET_PIN -1

// Create display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);

// Function to measure available free memory
int freeMemory() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  Serial.println("=== MEMORY TEST VERSION ===");
  Serial.print("Free memory at startup: ");
  Serial.println(freeMemory());
  
  Wire.begin();
  Serial.println("I2C initialized");
  Serial.print("Free memory after I2C: ");
  Serial.println(freeMemory());
  
  Serial.println("Attempting SSD1306 initialization...");
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SUCCESS: SSD1306 initialized!");
    Serial.print("Free memory after display init: ");
    Serial.println(freeMemory());
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Memory Test OK!");
    display.display();
  } else {
    Serial.println("FAILED: SSD1306 allocation failed");
    Serial.print("Free memory after failed attempt: ");
    Serial.println(freeMemory());
  }
}

void loop() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 5000) {
    Serial.print("Free memory in loop: ");
    Serial.println(freeMemory());
    lastPrint = millis();
  }
  delay(100);
}
