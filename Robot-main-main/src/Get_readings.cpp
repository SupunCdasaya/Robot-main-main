// IR sensor pin definitions
#include <Arduino.h>
#include "Get_readings.h"

// Array of IR sensor pins: L4, L3, L2, L1, C0, R1, R2, R3, R4
const int IR_PINS[9] = {L4, L3, L2, L1, C0, R1, R2, R3, R4};

void IR_pin_configuration() {
    for (int i = 0; i < 9; i++) {
        pinMode(IR_PINS[i], INPUT);
    }
}

void read_IR_sensors(int readings[10]) {
    for (int i = 0; i < 9; i++) {
        readings[i] = analogRead(IR_PINS[i]);
        Serial.print(readings[i]);
        Serial.print(" ");
    }
    Serial.println("");
}

void print_IR_readings(const int readings[9]) {
    Serial.print("IR readings: ");
    for (int i = 0; i < 9; i++) {
        Serial.print(readings[i]);
        if (i < 8) Serial.print(", ");
    }
    Serial.println();
}

void calibrate_IR_sensors(int readings[9], int samples) {
    int min_readings[9];
    int max_readings[9];
    for (int i = 0; i < 9; i++) {
        min_readings[i] = 1023;
        max_readings[i] = 0;
    }
    for (int s = 0; s < samples; s++) {
        Serial.print("Calibrating sample: ");
        Serial.println(s + 1);
        for (int i = 0; i < 9; i++) {
            int val = analogRead(IR_PINS[i]);
            if (val < min_readings[i]) min_readings[i] = val;
            if (val > max_readings[i]) max_readings[i] = val;
        }
        delay(5);
    }
    for (int i = 0; i < 9; i++) {
        readings[i] = (min_readings[i] + max_readings[i]) / 2; // threshold for digital high/low
    }
    // Optionally print calibration results
    Serial.println("IR Sensor Calibration Results:");
    for (int i = 0; i < 9; i++) {
        Serial.print("Sensor "); Serial.print(i);
        Serial.print(": min="); Serial.print(min_readings[i]);
        Serial.print(", max="); Serial.print(max_readings[i]);
        Serial.print(", threshold="); Serial.println(readings[i]);
    }
}

void digitalize_with_calibrated_threshold(const int analog_readings[9], const int thresholds[9], int digital[9]) {
    for (int i = 0; i < 9; i++) {
        digital[i] = (analog_readings[i] > thresholds[i]) ? 1 : 0;
    }
    // Print the digitalized array
    // Serial.print("Digitalized: ");
    // for (int i = 0; i < 9; i++) {
    //     Serial.print(digital[i]);
    //     if (i < 8) Serial.print(", ");
    // }
    // Serial.println();
}