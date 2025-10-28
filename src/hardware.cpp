#include "hardware.h"
#include <Wire.h>
#include <Arduino.h>
#include "imu.h"

void init_pins(){
  // Motor direction pinspinMode(STAIR1_DIR_PIN1,OUTPUT);
  pinMode(STAIR1_DIR_PIN2,OUTPUT);
  pinMode(STAIR2_DIR_PIN1,OUTPUT);
  pinMode(STAIR2_DIR_PIN2,OUTPUT);
  pinMode(PULLEY_DIR_PIN1,OUTPUT);
  pinMode(PULLEY_DIR_PIN2,OUTPUT);
  ledcSetup(0,PWM_FREQ,8);
  ledcAttachPin(STAIR1_DIR_PIN1,0);
  ledcSetup(1,PWM_FREQ,8);
  ledcAttachPin(STAIR1_DIR_PIN2,1);
  ledcSetup(2,PWM_FREQ,8);
  ledcAttachPin(STAIR2_DIR_PIN1,2);
  ledcSetup(3,PWM_FREQ,8);
  ledcAttachPin(STAIR2_DIR_PIN2,3);
  ledcSetup(4,PWM_FREQ,8);
  ledcAttachPin(PULLEY_DIR_PIN1,4);
  ledcSetup(5,PWM_FREQ,8);
  ledcAttachPin(PULLEY_DIR_PIN2,5);
}

void init_Serial(){
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Serial initialized");
}

void init_I2C(){
  Wire.begin();
  init_IMU();
  Serial.println("I2C initialized");
}

