/*
 * Mock Display for Testing
 * This allows us to test the calculation functions without actual display hardware
 */

#ifndef TEST_DISPLAY_MOCK_H
#define TEST_DISPLAY_MOCK_H

// Mock Adafruit_SSD1306 class for testing
class MockDisplay {
public:
    void clearDisplay() {}
    void setTextSize(int size) {}
    void setTextColor(int color) {}
    void setCursor(int x, int y) {}
    void print(const char* text) {}
    void print(int value) {}
    void drawRect(int x, int y, int w, int h, int color) {}
    void fillRect(int x, int y, int w, int h, int color) {}
    void drawFastVLine(int x, int y, int h, int color) {}
    void display() {}
};

// Global mock display instance for testing
extern MockDisplay* display;

// Arduino compatibility constants
#define SSD1306_WHITE 1
#define SSD1306_BLACK 0

#endif // TEST_DISPLAY_MOCK_H
