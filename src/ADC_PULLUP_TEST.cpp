#include <Arduino.h>

void setup() {
    // analogReference(DEFAULT); // use 3.3V reference
  Serial.begin(9600);
  delay(1000);
    Serial.println("ADC Pullup Test");
//   pinMode(A2, INPUT); // ensure no pullup
pinMode (A3, INPUT);
pinMode (A2, INPUT);
  delay(200);
}
void loop() {
  int sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(A2);
    delay(1);
  }
  int v = sum / 10;
  float voltage = v * (3.3 / 1023.0);
  Serial.print("A2 Averaged: ");
  Serial.print(v);
  Serial.print(" Voltage: ");
  Serial.println(voltage, 3);
  delay(100);
}
