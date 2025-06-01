// #include <RH_ASK.h>
// #include <SPI.h>

// RH_ASK driver(1000);

// void setup() {
//   Serial.begin(9600);
//   if (!driver.init())
//     Serial.println("init failed");
// }

// int i = 0;

// void loop() {
//   uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
//   uint8_t buflen = sizeof(buf); // buflen will store the actual received length

//   if (driver.recv(buf, &buflen)) {
//     // Ensure the received data is null-terminated before printing as a string.
//     // Check if buflen is less than the buffer's capacity to avoid overflow.
//     if (buflen < sizeof(buf)) {
//       buf[buflen] = '\0'; // Add the null terminator at the end of the received data
//     } else {
//       buf[sizeof(buf) - 1] = '\0';
//     }
//     // ------------------------------------

//     i = (i <= 5000) ? i + 1 : 0;
//     Serial.print("Received: ");
//     Serial.println((char*)buf); 
//   }
//   delay(10);
// }


/*
 *  Range Test Protocol
 *  Upload this diagnostic sketch to receiver:
 *  
 *  Good Results: >5 packets/s at 5m = working
 *  Bad Results: <2 packets/s = check antenna/capacitors
 *  This sketch will print RSSI and packets/s to Serial Monitor.
 *
*/
// void loop() {
//   static uint32_t last = 0;
//   static uint16_t count = 0;
  
//   if (driver.recv(buf, &buflen)) {
//     count++;
    
//     if (millis() - last > 1000) {
//       Serial.print("RSSI: "); Serial.print(driver.lastRssi());
//       Serial.print(" | Packets/s: "); Serial.println(count);
//       count = 0;
//       last = millis();
//     }
//   }
// }