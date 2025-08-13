#ifndef ENCODER_H
#define ENCODER_H

#define ENCODER_LEFT_A 2 //19// Pin for encoder A 3
#define ENCODER_LEFT_B 3 //18// Pin for encoder B
#define ENCODER_RIGHT_A  19 //2// Pin for encoder A
#define ENCODER_RIGHT_B 18 //3// Pin for encoder B

extern volatile long encoderCount_Left;
extern volatile long encoderCount_Right;

extern volatile bool left_ISR_hit;
extern volatile bool right_ISR_hit;

void countEncLeft();
void countEncRight();

#endif