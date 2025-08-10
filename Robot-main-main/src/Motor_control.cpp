
#include <Arduino.h>
#include "Motor_control.h"

void motor_pin_configuration() {
  pinMode(LM_EN, OUTPUT);
  pinMode(LM_FW, OUTPUT);
  pinMode(LM_BW, OUTPUT);
  pinMode(RM_EN, OUTPUT);
  pinMode(RM_FW, OUTPUT);
  pinMode(RM_BW, OUTPUT);
}

void right_motor(int speed, bool direction, int error) {
    if(direction == 1){ // Forward
        digitalWrite(RM_FW, HIGH);
        digitalWrite(RM_BW, LOW);
    }else{ // Backward
        digitalWrite(RM_FW, LOW);
        digitalWrite(RM_BW, HIGH);
    }
    analogWrite(RM_EN, constrain(speed - error, 0, 255));
}

void left_motor(int speed, bool direction, int error){
    if(direction == 1){ // Forward
        digitalWrite(LM_FW, HIGH);
        digitalWrite(LM_BW, LOW);
    }else{ // Backward
        digitalWrite(LM_FW, LOW);
        digitalWrite(LM_BW, HIGH);
    }
    analogWrite(LM_EN, constrain(speed + error, 0, 255));
}

void forward(int leftSpeed, int rightSpeed, int error) {
  right_motor(rightSpeed, 1, error);
  left_motor(leftSpeed, 1, error);
  
}

void backward(int leftSpeed, int rightSpeed, int error) {
  right_motor(rightSpeed, 0, error);
  left_motor(leftSpeed, 0, error);
}

void stopMotors() {
  digitalWrite(RM_FW, HIGH);
  digitalWrite(RM_BW, HIGH);
  digitalWrite(LM_FW, HIGH);
  digitalWrite(LM_BW, HIGH);
  analogWrite(RM_EN, 100);
  analogWrite(LM_EN, 100);
}

void stopMotors2(bool right_motor, bool left_motor) {
  if(right_motor) {
    digitalWrite(RM_FW, HIGH);
    digitalWrite(RM_BW, HIGH);
    analogWrite(RM_EN, 100);
  }
  if(left_motor) {
    digitalWrite(LM_FW, HIGH);
    digitalWrite(LM_BW, HIGH);
    analogWrite(LM_EN, 100);
  }
}

/*
void forward(int Speed, int motor, int error) {
  if (motor == 1) {
  digitalWrite(RM_FW, HIGH);
  digitalWrite(RM_BW, LOW);
  setMotorSpeed(Speed, motor, error);
  } else {
  digitalWrite(LM_FW, HIGH);
  digitalWrite(LM_BW, LOW);
  }
  
}

void backward(int leftSpeed, int rightSpeed, int error) {
  digitalWrite(LM_FW, LOW);
  digitalWrite(LM_BW, HIGH);
  digitalWrite(RM_FW, LOW);
  digitalWrite(RM_BW, HIGH);
  setMotorSpeed(leftSpeed, rightSpeed, error);
}*/