// Functions to turn the robot using encoder readings
#include <Arduino.h>
#include "Motor_control.h"
#include "Get_readings.h"
#include "encoder.h"
#include "PID_controller.h"

// Define motor speed
#define SPEED 150
#define turncount1 90
#define turncount2 100
#define turncount3 250


// Turn left: left motor 100, right motor 150 encoder counts
void turnLeft() {
    stopMotors();
    delay(500);
    encoderCount_Left = 0;
    encoderCount_Right = 0;
    right_motor(SPEED, 1, 0); // right forward
    left_motor(SPEED, 0, 0); // left backward

    while (encoderCount_Left > -turncount1 || encoderCount_Right < turncount1) {
        Serial.print("encoderCount_Left: ");
        Serial.print(encoderCount_Left);
        Serial.print(" ");
        Serial.print("encoderCount_Right: ");
        Serial.print(encoderCount_Right);
        Serial.println();
        if (encoderCount_Left <= -turncount1) left_motor(30, 1, 0);
        if (encoderCount_Right >= turncount1) right_motor(30, 0, 0);;
    }
    stopMotors();
}

// Turn right: right motor 100, left motor 150 encoder counts
void turnRight() {
    stopMotors();
    delay(500);
    encoderCount_Left = 0;
    encoderCount_Right = 0;

    right_motor(SPEED, 0, 0); // Right motor backward
    left_motor(SPEED, 1, 0); // Left motor forward

    while (encoderCount_Right > -turncount2 || encoderCount_Left < turncount2) {
        Serial.print("encoderCount_Right: ");
        Serial.print(encoderCount_Right);
        Serial.print(" ");
        Serial.print("encoderCount_Left: ");
        Serial.print(encoderCount_Left);
        Serial.println();
        if (encoderCount_Right <= -turncount2) right_motor(30, 1, 0);
        if (encoderCount_Left >= turncount2) left_motor(30, 0, 0);
    }
    stopMotors();
}

// Turn 180 degrees: both motors 300 encoder counts
void turn180(bool left) {
    encoderCount_Left = 0;
    encoderCount_Right = 0;
    if (left) {
        right_motor(SPEED, 1, 0); // right forward
        left_motor(SPEED, 0, 0); // left backward
        while (encoderCount_Left > -turncount3 || encoderCount_Right < turncount3) {
            if (encoderCount_Left <= -turncount3){
                left_motor(20, 1, 0);
                //stopMotors2(false, true);
            }
            if (encoderCount_Right >= turncount3) {
                right_motor(20, 0, 0);
                //stopMotors2(true, false);
            }
        }
    } else {
        right_motor(SPEED, 0, 0); // right backward
        left_motor(SPEED, 1, 0); // left forward
        while (encoderCount_Right > -turncount3 || encoderCount_Left < turncount3) {
            if (encoderCount_Right <= -turncount3){
                right_motor(20, 1, 0);
                //stopMotors2(true, false);
            }
            if (encoderCount_Left >= turncount3) {
                left_motor(20, 0, 0);
                //stopMotors2(false, true);
            }
        }
    }
    stopMotors();
}

//===================== Gyro ========================
/*
void turn90(bool clockwise, float currentHeading) {
    float startHeading = currentHeading;
    float targetHeading = startHeading + (clockwise ? 90 : -90);
    if (targetHeading >= 360) targetHeading -= 360;
    if (targetHeading < 0) targetHeading += 360;

    while (abs(targetHeading - currentHeading) > 2) { // 2° tolerance
        float correction = headingPID(targetHeading, currentHeading);
        // Motor control placeholder:
        // setMotorSpeed(correction, -correction);
    }
    // setMotorSpeed(0, 0); // stop motors
}

void turn180_gyro(float currentHeading) {
    float startHeading = currentHeading;
    float targetHeading = startHeading + 180;
    if (targetHeading >= 360) targetHeading -= 360;

    while (abs(targetHeading - currentHeading) > 2) {
        float correction = headingPID(targetHeading, currentHeading);
        // Motor control placeholder:
        // setMotorSpeed(correction, -correction);
    }
    // setMotorSpeed(0, 0); // stop motors
}
    */