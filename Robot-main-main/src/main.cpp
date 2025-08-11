#include <Arduino.h>
#include "Get_readings.h"
#include "Motor_control.h"
#include "PID_controller.h"
#include "buzzer.h"
#include "encoder.h"
#include "turning.h"
#include "action.h"

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
  calibrate_IR_sensors(thresholds, 200); // Calibrate IR sensors with 200 samples, store thresholds
  Serial.println("IR sensors calibration completed.");
  buzzer_on(3, 100);
  delay(1000);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), countEncLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), countEncRight, RISING);
}

void loop() {

  //========= IR PID ==========
  read_IR_sensors(readings);
  digitalize_with_calibrated_threshold(readings, thresholds, digital);
  bool cross = digital[0] && digital[1] && digital[2] && digital[3] && digital[4] && digital[5] && digital[6] && digital[7] && digital[8];
  bool t_left = (digital[0] && digital[1] && digital[2] && digital[3]) && digital[4];
  bool t_right = (digital[5] && digital[6] && digital[7] && digital[8]) && digital[4];
  bool white = (!digital[0]) && (!digital[1]) && (!digital[2]) && (!digital[3]) && (!digital[4]) && (!digital[5]) && (!digital[6]) && (!digital[7]) && (!digital[8]);

  if(cross == true || t_left == true || t_right == true || white == true) {
    if(cross) {
      Serial.println("Cross detected: Cross");
      buzzer_on(1, 50);
      handleSpecialCross("cross", digital, thresholds);
    } else if (t_left) {
      Serial.println("Cross detected: T-Left");
      buzzer_on(2, 20);
      handleSpecialCross("t_left", digital, thresholds);
    } else if (t_right) {
      Serial.println("Cross detected: T-Right");
      buzzer_on(3, 20);
      handleSpecialCross("t_right", digital, thresholds);
    }else if(white) {
      Serial.println("Cross detected: White");
      buzzer_on(1, 200);
      turn180(1);
      //bit forward
    }
  } else {
    error = calculate_error(digital);
    pid_output = compute_pid(error, previous_error, KP, KD);
    //pid_output = map(pid_output, 0, 200, 0, 150);
    previous_error = error;
    forward(80, 80, -pid_output);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Error: " + String(error));
    display.println("pid_output: " + String(pid_output));
    display.display();
  }

  //========== Encoder PID ==========
  // if(right_speed<100 && left_speed<100){
  //   right_speed += 5;
  //   left_speed += 5;
  // }
  // encoder_error = calculate_error_encoder(encoderCount_Left, encoderCount_Right);
  // encoder_pid_output = compute_pid_encoder(encoder_error, previous_encoder_error, encoder_integral, KPe, KDe, KIe);
  // previous_encoder_error = encoder_error;

  // Serial.print("Encoder PID Output: "); 
  // Serial.println(encoder_pid_output);
  // --------------------
  // Display data on OLED
  // --------------------
  // display.clearDisplay();
  // display.setCursor(0, 0);
  // display.println("Left Count: " + String(encoderCount_Left));
  // display.println("Right Count: " + String(encoderCount_Right));
  // display.println("Error: R-L " + String(encoder_error));
  // display.println("KPe: " + String(KPe));
  // display.println("PID Output: " + String(encoder_pid_output));
  // display.println("Speed:R L " + String(right_speed) +" "+ String(left_speed));
  // display.display();

  // Serial.print("left speed: " + String(left_speed));
  // Serial.println(" right speed: " + String(right_speed));
  // forward(left_speed, right_speed, -encoder_pid_output);
   
}
