#ifndef TURNING_H
#define TURNING_H
#include <Arduino.h>


void turnLeft();
void turnRight();
void turn180(bool left);

void turn90(bool clockwise, float currentHeading);



#endif // TURNING_H