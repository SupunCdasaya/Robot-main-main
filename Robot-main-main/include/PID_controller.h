#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

#define KP  8.0 // Proportional gain 8
#define KD  4.0 // Derivative gain 0.5
#define kI  0.0 // Integral gain (not used in this example)

//=============== Encoder ================================
#define KPe 0.65//0.71 //0.7//15//20 // Proportional gain for encoder
#define KDe 0.52 //0.55 //0.5 //10 //6.8// 10 // Derivative gain for encoder
#define KIe 0.09 //0.085//5 //2.8// 5

//=============== Gyro ================================
#define Kpg  5
#define Kig  0.0
#define Kdg  0

int calculate_error(const int digital[9]);
int compute_pid(int error, int pre_error, float Kp, float Kd);

int calculate_error_encoder(long encoderCount_Left, long encoderCount_Right);
int compute_pid_encoder(int error, int previous_error, int &integral, float Kp, float Kd, float Ki);


// PID to move towards target heading
float gyro_error(int target, int current);
float gyro_pid(int error, int prev_error, float Kp, float Ki, float Kd, int &gyro_i);
//float headingPID(int targetHeading, int currentHeading, int previous_Error, float Kp, float Ki, float Kd);

#endif // PID_CONTROLLER_H
