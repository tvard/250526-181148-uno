// Simple RF Communication Test
#include <SPI.h>
#include <RF24.h>

// NRF24L01 pins (adjust for your setup)
#define NRF_CE_PIN 10
#define NRF_CSN_PIN 9

// Radio setup
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte RADIO_ADDRESSES[][6] = {"00001", "00002"};
const int RADIO_CHANNEL = 76;

// Test packet structure
struct TestPacket {
  int counter;
  int xValue;
  int yValue;
  bool buttonPressed;
  uint8_t checksum;
};

// Function prototypes
void testTransmit();
void testReceive();

uint8_t calculateChecksum(const TestPacket& data) {
  uint8_t checksum = 0;
  checksum ^= (data.counter & 0xFF);
  checksum ^= ((data.counter >> 8) & 0xFF);
  checksum ^= (data.xValue & 0xFF);
  checksum ^= ((data.xValue >> 8) & 0xFF);
  checksum ^= (data.yValue & 0xFF);
  checksum ^= ((data.yValue >> 8) & 0xFF);
  checksum ^= data.buttonPressed ? 0xFF : 0x00;
  return checksum;
}

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  Serial.println("=== RF COMMUNICATION TEST ===");
  Serial.println("Send 'T' for transmit mode, 'R' for receive mode");
  
  // Initialize SPI and radio
  SPI.begin();
  if (!radio.begin()) {
    Serial.println("ERROR: Radio failed to initialize!");
    while(1) { delay(1000); }
  }
  
  // Configure radio
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setPayloadSize(sizeof(TestPacket));
  radio.setAutoAck(true);
  radio.setRetries(5, 5);
  
  Serial.println("Radio initialized successfully");
  Serial.print("Channel: "); Serial.println(RADIO_CHANNEL);
  Serial.print("Payload size: "); Serial.println(sizeof(TestPacket));
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    if (cmd == 'T' || cmd == 't') {
      testTransmit();
    } else if (cmd == 'R' || cmd == 'r') {
      testReceive();
    }
  }
}

void testTransmit() {
  Serial.println("\n--- TRANSMIT MODE ---");
  Serial.println("Sending test packets...");
  
  radio.openWritingPipe(RADIO_ADDRESSES[0]); // Send to "00001"
  radio.stopListening();
  
  int counter = 0;
  for (int i = 0; i < 10; i++) {
    TestPacket packet = {
      counter++,
      512 + i * 10,  // X value
      512 - i * 5,   // Y value
      i % 2 == 0,    // Button
      0
    };
    packet.checksum = calculateChecksum(packet);
    
    bool success = radio.write(&packet, sizeof(TestPacket));
    Serial.print("Packet "); Serial.print(i);
    Serial.print(": "); Serial.println(success ? "OK" : "FAILED");
    
    delay(100);
  }
  
  Serial.println("Transmit test complete. Send 'R' to switch to receive mode.");
}

void testReceive() {
  Serial.println("\n--- RECEIVE MODE ---");
  Serial.println("Listening for packets...");
  
  radio.openReadingPipe(0, RADIO_ADDRESSES[0]); // Listen on "00001"
  radio.startListening();
  
  unsigned long startTime = millis();
  int packetsReceived = 0;
  
  while (millis() - startTime < 10000) { // Listen for 10 seconds
    if (radio.available()) {
      TestPacket packet;
      radio.read(&packet, sizeof(TestPacket));
      
      uint8_t calculatedChecksum = calculateChecksum(packet);
      if (packet.checksum == calculatedChecksum) {
        packetsReceived++;
        Serial.print("RX: Counter="); Serial.print(packet.counter);
        Serial.print(" X="); Serial.print(packet.xValue);
        Serial.print(" Y="); Serial.print(packet.yValue);
        Serial.print(" Btn="); Serial.println(packet.buttonPressed ? "ON" : "OFF");
      } else {
        Serial.println("RX: Checksum error!");
      }
    }
    delay(10);
  }
  
  Serial.print("Received "); Serial.print(packetsReceived);
  Serial.println(" valid packets in 10 seconds");
  Serial.println("Send 'T' to switch to transmit mode.");
}
