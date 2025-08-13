#include <Arduino.h>
#include "encoder.h" 

volatile long encoderCount_Left = 0;
volatile long encoderCount_Right = 0;
volatile bool left_ISR_hit = 0;
volatile bool right_ISR_hit = 0;

 void countEncLeft() {
    if(digitalRead(ENCODER_LEFT_B) == LOW) {
        encoderCount_Left++;
    } else {
        encoderCount_Left--;
    }
    left_ISR_hit = 1;
}

void countEncRight() {
    if(digitalRead(ENCODER_RIGHT_B) == LOW) {
        encoderCount_Right++;
    } else {
        encoderCount_Right--;
    }
    right_ISR_hit = 1;
}