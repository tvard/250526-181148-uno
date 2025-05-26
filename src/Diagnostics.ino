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