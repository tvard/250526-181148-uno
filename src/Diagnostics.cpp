// #include <SPI.h>
// #include <RF24.h>

// #define NRF_CE_PIN 9
// #define NRF_CSN_PIN 10

// RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

// void spiLoopbackTest() {
//   Serial.println("SPI Loopback Test: Connect MOSI to MISO pin!");
//   byte out = 0x55;
//   byte in = SPI.transfer(out);
//   Serial.print("Sent: 0x");
//   Serial.print(out, HEX);
//   Serial.print(" | Received: 0x");
//   Serial.println(in, HEX);
//   if (in == out) {
//     Serial.println("SPI loopback SUCCESS");
//   } else {
//     Serial.println("SPI loopback FAILED");
//   }
// }

// void setup() {
//   Serial.begin(9600);
//   delay(1000);
//   Serial.println("=== NRF24L01 DIAGNOSTIC ===");
//   SPI.begin();
//   delay(100);
//   spiLoopbackTest();
//   delay(1000);
//   Serial.print("Initializing radio...");
//   if (radio.begin()) {
//     Serial.println("SUCCESS");
//     radio.printDetails();
//   } else {
//     Serial.println("FAILED");
//     Serial.println("Check wiring, power, and module orientation.");
//   }
// }

// void loop() {
//   // Nothing to do in loop for basic diagnostic
// }


/*
  If your serial output has these values same then Your nrf24l01 module is in working condition :
  
  EN_AA          = 0x3f
  EN_RXADDR      = 0x02
  RF_CH          = 0x4c
  RF_SETUP       = 0x03
  CONFIG         = 0x0f
  n
 */

// #include <SPI.h>
// #include <RF24.h>
// #include <printf.h>

// RF24 radio(9, 10); // CE, CSN pins

// byte RADIO_ADDRESSES[][6] = {"1Node", "2Node"};


// void setup() {
//   radio.begin();
//   radio.setPALevel(RF24_PA_LOW);
  
//   radio.openWritingPipe(RADIO_ADDRESSES[0]);
//   radio.openReadingPipe(1, RADIO_ADDRESSES[1]); 
//   radio.startListening();
  
//   Serial.begin(9600);
//   printf_begin();

//   radio.printDetails();
  
// }

// void loop() {
// //  empty

// }





// #include <SPI.h>

// const int CE_PIN  = 9;
// const int CSN_PIN = 10;

// void setup() {
//   Serial.begin(9600);
//   delay(200);
//   // Force pins as outputs and try to drive them
//   pinMode(CE_PIN, OUTPUT);
//   pinMode(CSN_PIN, OUTPUT);

//   digitalWrite(CE_PIN, LOW);    // normal idle
//   digitalWrite(CSN_PIN, HIGH);  // CSN idle must be HIGH

//   Serial.println("Toggling CE/CSN and reading STATUS via SPI");

//   SPI.begin();           // init SPI hardware
//   SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
// }

// uint8_t readStatus() {
//   digitalWrite(CSN_PIN, LOW);     // select
//   uint8_t status = SPI.transfer(0xFF); // NOP returns STATUS
//   digitalWrite(CSN_PIN, HIGH);    // deselect
//   return status;
// }

// void loop() {
//   // 1) try to assert CSN HIGH from MCU (test if MCU can drive it)
//   digitalWrite(CSN_PIN, HIGH);
//   delay(50);
//   int v1 = analogRead(A0); // (optional) if you want to read pin via ADC; ignore if not used

//   // read status
//   uint8_t s = readStatus();
//   Serial.print("STATUS read: 0x");
//   Serial.println(s, HEX);

//   // 2) toggle CE a few times
//   digitalWrite(CE_PIN, HIGH);
//   delay(100);
//   digitalWrite(CE_PIN, LOW);
//   delay(100);

//   // 3) drive CSN low and attempt read (should be same, but helps observe)
//   digitalWrite(CSN_PIN, LOW);
//   delay(50);
//   digitalWrite(CSN_PIN, HIGH);

//   // wait before next loop
//   delay(1000);
// }



#include <SPI.h>

const int CE_PIN  = 9;
const int CSN_PIN = 10;
const int MISO_PIN = 12; // hardware MISO on AVR

void setup() {
  Serial.begin(9600);
  while(!Serial) { ; }
  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT);

  digitalWrite(CE_PIN, LOW);
  digitalWrite(CSN_PIN, HIGH);

  // very slow SPI (125 kHz)
  SPI.begin();
  SPI.beginTransaction(SPISettings(125000, MSBFIRST, SPI_MODE0));

  Serial.println("CSN high (idle). Now toggling CSN low and reading STATUS + MISO input.");
  delay(200);
}

uint8_t readStatusWithCSN(bool csnLow) {
  if (csnLow) digitalWrite(CSN_PIN, LOW);
  else digitalWrite(CSN_PIN, HIGH);

  // small settle time
  delayMicroseconds(50);

  // read raw STATUS using NOP
  uint8_t status = SPI.transfer(0xFF);

  // sample the MISO pin level (digital)
  int misoLevel = digitalRead(MISO_PIN);

  if (csnLow) digitalWrite(CSN_PIN, HIGH);
  return (status & 0xFF) | (misoLevel ? 0x100 : 0x000);
}

void loop() {
  // 1) CSN HIGH (idle) sample
  uint16_t r1 = readStatusWithCSN(false);
  uint8_t st1 = r1 & 0xFF; int m1 = (r1 & 0x100) ? 1 : 0;
  Serial.print("CSN=HIGH  STATUS=0x"); if (st1<16) Serial.print('0'); Serial.print(st1, HEX);
  Serial.print("  MISO_pin="); Serial.println(m1);

  delay(80);

  // 2) CSN LOW (selected) sample and SPI transfer
  uint16_t r2 = readStatusWithCSN(true);
  uint8_t st2 = r2 & 0xFF; int m2 = (r2 & 0x100) ? 1 : 0;
  Serial.print("CSN=LOW   STATUS=0x"); if (st2<16) Serial.print('0'); Serial.print(st2, HEX);
  Serial.print("  MISO_pin="); Serial.println(m2);

  delay(250);
}
