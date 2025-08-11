#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// Define motor control pins
#define LM_EN 5
#define LM_EN_2 7
//#define LM_BW 25
//#define LM_FW 23
#define RM_EN 6
#define RM_EN_2 8
//#define RM_BW 27
//#define RM_FW 29


void motor_pin_configuration();
void right_motor(int speed, bool direction, int error);
void left_motor(int speed, bool direction, int error);
void forward(int leftSpeed, int rightSpeed, int error);
void backward(int leftSpeed, int rightSpeed, int error);
void stopMotors();
void stopMotors2(bool right_motor, bool left_motor);

#endif // MOTOR_CONTROL_H
