#include "hardware.h"
#include <Arduino.h>
#include "motor_control.h"
#include "encoder.h"

float stair_target[2]={0,0};
int motor_output[3]={0,0,0};// stair1, stair2, pulley
int motor_dirs[3]={0,0,0};// stair1, stair2

void control_motors(float deg_mot1,float deg_mot2,int dir_motor3,int spd_motor3){
  reset_pid();
  stair_target[0]=deg_mot1;
  stair_target[1]=deg_mot2;
  motor_dirs[2]=dir_motor3;
  motor_output[2]=spd_motor3;
}

void rotate_motor(int motor_num, int dir, int duty){
  switch(motor_num){
    case MotorID::STAIR1:// stair1
      if(dir==0){//brake
        ledcWrite(0,duty);
        ledcWrite(1,duty);
      }
      else if(dir==1){//cw
        ledcWrite(0,duty);
        ledcWrite(1,0);
      }else if (dir == 2){//ccw
        ledcWrite(0,0);
        ledcWrite(1,duty);
      } else {//stop
        ledcWrite(0,0);
        ledcWrite(1,0);
      }
      break;
    case MotorID::STAIR2:// stair2
      if(dir==0){
        ledcWrite(2,duty);
        ledcWrite(3,duty);
      }
      else if(dir==1){
        ledcWrite(2,duty);
        ledcWrite(3,0);
      }else if (dir == 2){
        ledcWrite(2,0);
        ledcWrite(3,duty);
      } else {
        ledcWrite(2,0);
        ledcWrite(3,0);
      }
      break;
    case MotorID::PULLEY:// pulley
      if(dir==0){
        ledcWrite(4,duty);
        ledcWrite(5,duty);
      }
      else if(dir==1){
        ledcWrite(4,duty);
        ledcWrite(5,0);
      }else if (dir == 2){
        ledcWrite(4,0);
        ledcWrite(5,duty);
      } else {
        ledcWrite(4,0);
        ledcWrite(5,0);
      }
      break;
    default:
      break;
  }
}

void stop_motors(){
  rotate_motor(1,0,0);
  rotate_motor(2,0,0);
  rotate_motor(3,0,0);
}

// PID control parameters
const float Kp = 2.0;
const float Ki = 0.0;
const float Kd = 0.1;
float previous_errors[2] = {0,0};
float integrals[2] = {0,0};
float filtered_derivatives[2] = {0,0};
const int dt = 5; // in ms
const float tau = 1.0; // Time constant for low-pass filter
void PID_rotate_stair(int mot_num,bool success){
  if (!success) {
    //Serial.println("I2C Error during PID control");
    motor_output[mot_num] = 0;
    motor_dirs[mot_num] = 0;
    return;
  } 
  float current_angle = stair_angle[mot_num];
  float target_angle = stair_target[mot_num];
  while(target_angle >= 360) target_angle -= 360;
  while(target_angle < 0) target_angle += 360;
  stair_target[mot_num] = target_angle;
  float error = target_angle - current_angle;
  // Normalize error to the range [-180, 180]
  if (error > 180) error -= 360;
  if (error < -180) error += 360;
  if(abs(error)<1)error=0; // Dead zone for small error  
  integrals[mot_num] += error * (dt / 1000.0);
  float derivative = (error - previous_errors[mot_num]) / (dt / 1000.0);
  // Low-pass filter for derivative
  float alpha =  tau/(tau+dt);// Smoothing factor (0 < alpha < 1)
  filtered_derivatives[mot_num] = alpha * derivative + (1 - alpha) * filtered_derivatives[mot_num];
  derivative = filtered_derivatives[mot_num];
  float output = Kp * error + Ki * integrals[mot_num] + Kd * derivative;

  int dir; // 0: brake, 1: cw, 2: ccw
  if(output > 0){
    dir = 1; // Clockwise
  } else if (output < 0){
    dir = 2; // Coundtter-clockwise
    output = -output; // Make output positive for duty cycle
  } else {
    dir = 0; // Brake
    output = 0;
  }
  // Determine motor direction and duty cycle
  if ((motor_dirs[mot_num]== 1 && dir==2) || (motor_dirs[mot_num]==2 && dir==1)) {
    motor_output[mot_num] = 0; // Set duty cycle to 0 before changing direction
    motor_dirs[mot_num] = 3;
    Serial.println("Dead time applied");
    delayMicroseconds(DEAD_TIME);
  }
  if (output > 255) output = 255; // Cap the output to max duty cycle
  if(output < 2) output = 0; // Dead zone for low
  else if(output < EFFECTIVE_DUTY_MIN) output = EFFECTIVE_DUTY_MIN; // Minimum effective duty cycle
  // rotate_motor(mot_num, dir, duty);
  motor_dirs[mot_num] = dir;
  motor_output[mot_num] = output;
  previous_errors[mot_num] = error;
}

void reset_pid(){
  previous_errors[0] = 0;
  previous_errors[1] = 0;
  integrals[0] = 0;
  integrals[1] = 0;
  filtered_derivatives[0] = 0;
  filtered_derivatives[1] = 0;
}