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

String pad3(int val) {
  char buf[4];
  snprintf(buf, sizeof(buf), "%03d", val);
  return String(buf);
}

String pad3f(float val) {
  char buf[7];
  dtostrf(val, 5, 2, buf);
  return String(buf);
}

String pad3s(int val) {
  char buf[4];
  snprintf(buf, sizeof(buf), "%3d", val);
  return String(buf);
}

String pad4s(int val) {
  char buf[5];
  snprintf(buf, sizeof(buf), "%4d", val);
  return String(buf);
}

String pad2s(int val) {
  char buf[3];
  snprintf(buf, sizeof(buf), "%2d", val);
  return String(buf);
}

String padString(String str, int width, char padChar, bool leftAlign) {
  int len = str.length();
  if (len >= width) return str;
  
  String padding = "";
  for (int i = 0; i < width - len; i++) {
    padding += padChar;
  }
  
  if (leftAlign) {
    return str + padding;
  } else {
    return padding + str;
  }
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

std::string pad3(int val) {
  char buf[4];
  snprintf(buf, sizeof(buf), "%03d", val);
  return std::string(buf);
}

std::string pad3f(float val) {
  char buf[7];
  snprintf(buf, sizeof(buf), "%5.2f", val);  // Use snprintf instead of dtostrf for native compilation
  return std::string(buf);
}

std::string padString(std::string str, int width, char padChar, bool leftAlign) {
  int len = str.length();
  if (len >= width) return str;
  
  std::string padding(width - len, padChar);
  
  if (leftAlign) {
    return str + padding;
  } else {
    return padding + str;
  }
}

#ifdef ARDUINO
String pad3s(int val) {
  char buf[4];
  snprintf(buf, sizeof(buf), "%3d", val);
  return String(buf);
}
#endif

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

