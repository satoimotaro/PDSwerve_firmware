#include <Arduino.h>
#include "esp_timer.h"
#include "hardware.h"
#include "motor_control.h"
#include "encoder.h"
#include "imu.h"
#include "utils.h"

// global variables:
volatile bool i2c_flag = false;
volatile bool pid_flag = false;
volatile bool stop_flag = false;
int tmr_count=0;
const bool debug = true;
// Timer callback function
void onTimer(void* arg) {
  i2c_flag = true;  // フラグだけ立てる（通信はここでしない）
  tmr_count++;
}

// Timer callback function declaration
void init_timer(){
  // Timer initialization moved to main.cpp
  const esp_timer_create_args_t timer_args = {
    .callback = &onTimer,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "i2c_timer"
  };
  esp_timer_handle_t timer;
  esp_timer_create(&timer_args, &timer);
  esp_timer_start_periodic(timer, 5000); //5 ms
}

void setup() {
  // put your setup code here, to run once:
  init_pins();
  stop_motors();
  init_Serial();
  init_I2C();
  reset_encoder(true);
  Serial.println("Encoder calibrated");
  init_timer();
  // Create a periodic timer that triggers every 10 ms  
  Serial.println("Timer started");
}

void loop() {
  // put your main code here, to run repeatedly:
  if(i2c_flag){
    i2c_flag = false;
    i2c_success = (read_encoder() & read_IMU());
    PID_rotate_stair(0,i2c_success);
    PID_rotate_stair(1,i2c_success);
  }
  if(Serial.available() && !stop_flag){
    stop_flag = !controller_commands(Serial.read());
  }
  if(tmr_count>=10){ 
    tmr_count=0;
    if(debug)Serial_println(i2c_success);
  }
  if(!stop_flag){
    rotate_motor(1,motor_dirs[0],motor_output[0]);
    rotate_motor(2,motor_dirs[1],motor_output[1]);
    rotate_motor(3,motor_dirs[2],motor_output[2]);
  }
}

