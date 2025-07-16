#include "helpers.h"
#ifdef ARDUINO
#include <Arduino.h>
#endif

#ifdef ARDUINO
String pad5(int val) {
  char buf[7];
  snprintf(buf, sizeof(buf), "%5d", val);
  return String(buf);
}

String pad5f(float val) {
  char buf[7];
  dtostrf(val, 5, 2, buf);
  return String(buf);
}
#else
#include <string>

std::string pad5(int val) {
  char buf[7];
  snprintf(buf, sizeof(buf), "%5d", val);
  return std::string(buf);
}

std::string pad5f(float val) {
  char buf[7];
  snprintf(buf, sizeof(buf), "%5.2f", val);
  return std::string(buf);
}


// class SerialSim {
// public:
//     static void print(const char* str) {}
//     static void print(int val) {}
//     static void println(const char* str) {}
//     static void println(int val) {}

//     SerialSim() {}
// };

// init class as 'Serial'
// SerialSim Serial;

#endif

