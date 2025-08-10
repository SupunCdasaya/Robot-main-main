#include <Arduino.h>
#include "Get_readings.h"
#include "Motor_control.h"
#include "PID_controller.h"
#include "buzzer.h"
#include "encoder.h"
#include "turning.h"
#include "action.h"

#include <Wire.h>
#include <QMC5883LCompass.h>
#include "L3G.h"

// Removed display initialization, now handled in oled.h/cpp

int thresholds[9];
int readings[9];
int digital[9];
int integral_error = 0;
int encoder_integral_error = 0;
int cross_type = 0;
int holding_speed = 20;
int right_speed = holding_speed;
int left_speed = holding_speed;

//======= IR PID =============
int error = 0;
int previous_error = 0;
int pid_output = 0;
int pid_sign = 0;

//========= Encorder =============
int encoder_error = 0;
int previous_encoder_error = 0;
int encoder_pid_output = 0;
int encoder_integral = 0;
int encoder_reading_zero = 0;

//====== Gyro Initialization ======
float pid_output_gyro = 0;
int integral_error_gyro = 0;
float previous_error_gyro = 0;
int initial_heading = 0;
int is_heading_set = 0;
int desired_heading = 0;
int error_gyro = 0;

float pid_output_to_motor = 0;
QMC5883LCompass compass;
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int soft_start = 0;

void setup() {
  pinMode(buzzer, OUTPUT);
  for(int i = 43;i<52;i++){
    pinMode(i, INPUT);
  }
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }

  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.setSmoothing(10,true); //call setSmoothing(STEPS, ADVANCED);
  compass.setCalibrationOffsets(-713.00, -1216.00, -1278.00);
  compass.setCalibrationScales(0.95, 1.06, 1.00);
  compass.setMode(0x01, 0x04, 0x10, 0x00);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("LANESRA");
  display.display();

  buzzer_on(1, 100);
  IR_pin_configuration();
  motor_pin_configuration();
  buzzer_on(3, 100);
  Serial.println("IR sensors calibration started.");
  calibrate_IR_sensors(thresholds, 100); // Calibrate IR sensors with 200 samples, store thresholds
  Serial.println("IR sensors calibration completed.");
  buzzer_on(3, 100);
  delay(1000);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), countEncLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), countEncRight, RISING);
  forward(20, 20, 0);
}

void loop() {

  //========= IR PID ==========
  read_IR_sensors(readings);
  digitalize_with_calibrated_threshold(readings, thresholds, digital);
  error = calculate_error(digital);
  pid_output = compute_pid(error, previous_error, KP, KD);
  //pid_output = map(pid_output, 0, 200, 0, 150);
  previous_error = error;

  //========== Encoder PID ==========
  if(right_speed<100 && left_speed<100){
    right_speed += 4;
    left_speed += 6;
  }
  encoder_error = calculate_error_encoder(encoderCount_Left, encoderCount_Right);
  encoder_pid_output = compute_pid_encoder(encoder_error, previous_encoder_error, encoder_integral, KPe, KDe, KIe);
  previous_encoder_error = encoder_error;

  Serial.print("Encoder PID Output: "); Serial.println(encoder_pid_output);
  // --------------------
  // Display data on OLED
  // --------------------
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Left Count: " + String(encoderCount_Left));
  display.println("Right Count: " + String(encoderCount_Right));
  display.println("Error: " + String(encoder_error));
  display.println("KPe: " + String(KPe));
  display.println("PID Output: " + String(encoder_pid_output));
  display.println("Speed:R L " + String(right_speed) +" "+ String(left_speed));
  display.display();

  /*compass.read();

    int x = compass.getX();
    int y = compass.getY();
    int z = compass.getZ();

    int heading = compass.getAzimuth(); // Heading in degrees
    

    Serial.print("X: "); Serial.print(x);
    Serial.print(" Y: "); Serial.print(y);
    Serial.print(" Z: "); Serial.print(z);
    Serial.print("  Heading: "); Serial.print(heading);
    if(is_heading_set == 5) {
      initial_heading = heading;
      Serial.print("Initial Heading: ");
      Serial.println(initial_heading);
      is_heading_set +=1;
      desired_heading = initial_heading; // Set desired heading after initial heading is set
    }else if(is_heading_set < 5) {
      is_heading_set += 1; // Set initial heading after 5 readings
      Serial.println("Setting initial heading...");
    }

    delay(300);
  
  // --------------------
  // Display data on OLED
  // --------------------
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Target: " + String(desired_heading));
  display.println("Current: " + String(heading));
  display.println("Error: " + String(error_gyro));
  display.println("PID Output: " + String(pid_output_gyro));
  display.println("Analog Read: " + String(analog_read));
  display.display();
  //pid_output_gyro = (headingPID(desired_heading, heading, previous_error_gyro, Kpg, Kig, Kdg))*1;
  
  //Serial.print("Leftcounter: " + String(encoderCount_Left));
  //Serial.println(" Rightcounter: " + String(encoderCount_Right));
  //Serial.print(" Encoder Error: " + String(encorder_error));
  //Serial.println(" Encoder PID Output: " + String(encorder_pid_output));

  //Serial.println("Forward");
  //left_motor(100, 1, 0);
  //right_motor(100, 1, 0);
  if(is_heading_set>5){
  //pid_output_gyro = (headingPID(desired_heading, heading, previous_error_gyro, Kpg, Kig, Kdg))*1;
  error_gyro = gyro_error(desired_heading, heading);
  pid_output_gyro = gyro_pid(error_gyro, previous_error_gyro, Kpg, Kig, Kdg, integral_error_gyro);
  Serial.println("PID Output Gyro: " + String(pid_output_gyro));
  previous_error_gyro = error_gyro;

  right_speed = 80;
  left_speed = 80;
  /*
  pid_output_to_motor = pid_output_gyro;
  if(right_speed - pid_output_to_motor < 0) {
    right_motor(200, 1, 0); 
  }else if(right_speed + pid_output_to_motor > 255){
    right_motor(50, 0, 0);  // Ensure right speed does not exceed 255
  }else if(right_speed - pid_output_to_motor > 0 || right_speed + pid_output_to_motor < 255){    
    right_motor(right_speed, 1, pid_output_to_motor);
  }

  if(left_speed - pid_output_to_motor <= 0) {
    left_motor(50, 1, 0);
  }else if(left_speed + pid_output_to_motor >= 255){
    left_motor(200, 0, 0); // Ensure left speed does not exceed 255
  }else if (left_speed - pid_output_to_motor > 0 || left_speed + pid_output_to_motor < 255){
    left_motor(left_speed, 1, pid_output_to_motor);
  } */

    Serial.print("left speed: " + String(left_speed));
    Serial.println(" right speed: " + String(right_speed));
    forward(left_speed, right_speed, encoder_pid_output);
   // Use PID output for correction forward( leftSpeed,  rightSpeed,  error);
  
  /*
    if(digital[0] == 1 && digital[1] == 1 && digital[2] == 1 && digital[3] == 1 && digital[4] == 1 && digital[5] == 1 && digital[6] == 1 && digital[7] == 1 && digital[8] == 1 ) { //cross
        Serial.println("Cross detected: Cross" );    
        cross_type = 1;
        backward(100,100,0);
        buzzer_on(10, 100);
        stopMotors();
        delay(5000);
        //handleSpecialCross("cross", readings, thresholds);
    }else if ((digital[0] == 1 && digital[1] == 1 && digital[2] == 1 && digital[3] == 1) && digital[4]==1){
      Serial.println("Cross detected: T-Left" );
      cross_type = 2;
      backward(100,100,0);
      stopMotors();
      buzzer_on(5, 100);
      delay(5000);
      //handleSpecialCross("t_left", readings, thresholds);
    }else if ((digital[5] == 1 && digital[6] == 1 && digital[7] == 1 && digital[8] == 1) && digital[4]==1){
      Serial.println("Cross detected: T-Right" );
      cross_type = 3;
      backward(100,100,0);
      stopMotors();
      buzzer_on(2, 100);
      delay(5000);
      //handleSpecialCross("t_right", readings, thresholds);
    }else if(digital[0] == 0 && digital[1] == 0 && digital[2] == 0 && digital[3] == 0 && digital[4] == 0 && digital[5] == 0 && digital[6] == 0 && digital[7] == 0 && digital[8] == 0){
      backward(100,100,0);
      stopMotors();
    }  

    */
    
/*
  delay(1000);
  Serial.println("Turning left");
  turnLeft();  
  delay(1000);
  forward(100, 100, 0);
  delay(1000);
  Serial.println("Turning right");
  turnRight();
  delay(1000);*/
  /*
delay(1000);
turn180(1);
delay(1000);
forward(105, 90, 0);
delay(1000);
turn180(0);
delay(1000);
*/

}
