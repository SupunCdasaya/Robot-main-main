#ifndef ACTION_H
#define ACTION_H

#include <Arduino.h>
#include <Motor_control.h>
#include <Get_readings.h>
#include <encoder.h>
#include <ir_detect.h>

void handleCrossApproach();
void handleSpecialCross(const char* crossType, const int analog_readings[9], const int thresholds[9]);
void rearrangeCrossArray();
void reverseAndSwapSortedArray(const char* arraysorted, int length);

#endif // ACTION_H
