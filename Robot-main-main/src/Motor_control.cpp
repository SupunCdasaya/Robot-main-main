
#include <Arduino.h>
#include "Motor_control.h"

void motor_pin_configuration() {
  pinMode(LM_EN, OUTPUT);
  pinMode(LM_EN_2, OUTPUT);
  pinMode(RM_EN, OUTPUT);
  pinMode(RM_EN_2, OUTPUT);
}

void right_motor(int speed, bool direction, int error) {
    if(direction == 1){ // Forward
      analogWrite(LM_EN, constrain(speed + error, 0, 255));
    }else{ // Backward
      analogWrite(LM_EN_2, constrain(speed + error, 0, 255));
    }
}

void left_motor(int speed, bool direction, int error){
    if(direction == 1){ // Forward
      analogWrite(RM_EN, constrain(speed - error, 0, 255));
    }else{ // Backward
      analogWrite(RM_EN_2, constrain(speed - error, 0, 255));
    }
    
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
  analogWrite(RM_EN, 0);
  analogWrite(LM_EN, 0);
  analogWrite(RM_EN_2, 0);
  analogWrite(LM_EN_2, 0);
}

void stopMotors2(bool right_motor, bool left_motor) {
  if(right_motor) {
    analogWrite(RM_EN, 100);
    analogWrite(RM_EN_2, 100);
  }
  if(left_motor) {
    analogWrite(LM_EN, 100);
    analogWrite(LM_EN_2, 100);
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