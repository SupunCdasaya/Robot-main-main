#ifndef ACTION_H
#define ACTION_H

#include <Arduino.h>
#include <Motor_control.h>
#include <Get_readings.h>
#include <encoder.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
extern Adafruit_SSD1306 display;


void handleCrossApproach();
void handleSpecialCross(const char* crossType, const int analog_readings[9], const int thresholds[9]);
void rearrangeCrossArray();
void reverseAndSwapSortedArray(const char* arraysorted, int length);

#endif // ACTION_H