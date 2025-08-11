// Functions to turn the robot using encoder readings
#include <Arduino.h>
#include "Motor_control.h"
#include "Get_readings.h"
#include "encoder.h"
#include "PID_controller.h"

// Define motor speed
#define SPEED 80
#define turncount1 150
#define turncount2 150
#define turncount3 370
#define turncount4 360


// Turn left: left motor 100, right motor 150 encoder counts
void turnLeft() {
    int L_speed = SPEED;
    int R_speed = SPEED;
    stopMotors();
    delay(500);
    noInterrupts();
    encoderCount_Left = 0;
    encoderCount_Right = 0;
    interrupts();
    right_motor(SPEED, 1, 0); // right forward
    //left_motor(SPEED, 0, 0); // left backward

    while (encoderCount_Left > -turncount1 || encoderCount_Right < turncount1) {
        Serial.print("encoderCount_Left: ");
        Serial.print(encoderCount_Left);
        Serial.print(" ");
        Serial.print("encoderCount_Right: ");
        Serial.print(encoderCount_Right);
        Serial.println();
        if (encoderCount_Left <= -turncount1) left_motor(20, 0, 0);
        else left_motor(L_speed, 0, 0);
        if (encoderCount_Right >= turncount1) right_motor(20, 1, 0);
        else right_motor(R_speed, 1, 0);
    }
    stopMotors();
}

// Turn right: right motor 100, left motor 150 encoder counts
void turnRight() {
    int L_speed = SPEED;
    int R_speed = SPEED;
    stopMotors();
    delay(500);
    noInterrupts();
    encoderCount_Left = 0;
    encoderCount_Right = 0;
    interrupts();

    right_motor(SPEED, 0, 0); // Right motor backward
    left_motor(SPEED, 1, 0); // Left motor forward

    while (encoderCount_Right > -turncount2 || encoderCount_Left < turncount2) {
         Serial.print("encoderCount_Right: ");
        Serial.print(encoderCount_Right);
        Serial.print(" ");
        Serial.print("encoderCount_Left: ");
        Serial.print(encoderCount_Left);
        Serial.println();
        if (encoderCount_Right <= -turncount2) right_motor(20, 0, 0);
        else right_motor(R_speed, 0, 0);
        if (encoderCount_Left >= turncount2) left_motor(20, 1, 0);
        else left_motor(L_speed, 1, 0);
    }
    stopMotors();
}

// Turn 180 degrees: both motors 300 encoder counts
void turn180(bool left) {
    stopMotors();
    delay(500);
    noInterrupts();
    encoderCount_Left = 0;
    encoderCount_Right = 0;
    interrupts();
    
    int L_speed = 100;
    int R_speed = 100;
    if (left) {
        right_motor(R_speed, 1, 0); // right forward
        left_motor(L_speed, 0, 0); // left backward
        while (encoderCount_Left > -turncount3 || encoderCount_Right < turncount3) {
        Serial.print("turn180  left->");
        Serial.print("encoderCount_Right: ");
        Serial.print(encoderCount_Right);
        Serial.print(" ");
        Serial.print("encoderCount_Left: ");
        Serial.print(encoderCount_Left);
        Serial.println();
            if (encoderCount_Left <= -turncount3){
                left_motor(20, 0, 0);
                //stopMotors2(false, true);
            }
            if (encoderCount_Right >= turncount3) {
                right_motor(20, 1, 0);
                //stopMotors2(true, false);
            }
        }
    } else {
        right_motor(R_speed, 0, 0); // right backward
        left_motor(L_speed, 1, 0); // left forward
        while (encoderCount_Right > -turncount4 || encoderCount_Left < turncount4) {
        Serial.print("turn180  right->");
        Serial.print("encoderCount_Right: ");
        Serial.print(encoderCount_Right);
        Serial.print(" ");
        Serial.print("encoderCount_Left: ");
        Serial.print(encoderCount_Left);
        Serial.println();
            if (encoderCount_Right <= -turncount4){
                right_motor(20, 0, 0);
            }
            if (encoderCount_Left >= turncount4) {
                left_motor(20, 1, 0);
            }
        }
    }
    stopMotors();
}


