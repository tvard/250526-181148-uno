#include "helpers.h"
#include <Arduino.h>

/// @brief Pads an integer value with spaces to ensure it is 5 characters wide.
/// @param val The integer value to be padded.
/// @return A String object containing the padded value.
String pad5(int val)
{
  char buf[7];
  snprintf(buf, sizeof(buf), "%5d", val);
  return String(buf);
}

/// @brief Pads a float value with spaces to ensure it is 5 characters wide.
/// @param val The float value to be padded.
/// @return A String object containing the padded value.
String pad5f(float val)
{
  char buf[7]; // Make sure this buffer is large enough for your output
  // dtostrf(float_value, total_width, decimal_places, char_array);
  dtostrf(val, 5, 2, buf); // 5 total width, 2 decimal places
  return String(buf);
}

