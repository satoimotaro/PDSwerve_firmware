#include <Arduino.h>
#include <Wire.h>
#include "esp_timer.h"

#define STAIR1_DIR_PIN1 33
#define STAIR1_DIR_PIN2 27
#define STAIR2_DIR_PIN1 26
#define STAIR2_DIR_PIN2 25
#define PULLEY_DIR_PIN1 32
#define PULLEY_DIR_PIN2 23

#define TCA_ADDRESS 0x70

#define PWM_FREQ 500

#define DEAD_TIME 100 // in microseconds
#define EFFECTIVE_DUTY_MIN 40 // minimum effective duty cycle

// global variables:
int ZERO[4] = {1226, 1281,0,0}; //encoder zero position stair1, stair2, pulley1,pulley2
float Gear_Ratio[4]={8.0/60,8.0/60,1/1.2,1/1.2};//stair1,stair2,pulley1,pulley2
float stair_angle[2]={0,0};
float Tyre_angle[2]={0,0};
int encoder_rawdata[4] = {0,0,0,0};
int encoder_savedata[4] = {0,0,0,0};
int raw_rot_count[4]={0,0,0,0};
int angle_rot_count[4]={0,0,0,0};
float stair_target[2]={0,0};
int motor_output[3]={0,0,0};
int motor_dirs[3] = {0,0,0};
volatile bool i2c_flag = false;
volatile bool pid_flag = false;
volatile bool stop_flag = false;
int tmr_count=0;
int mot_err[2]={0,0};//debug用
const bool debug = true;
const int MPU_ADDR = 0x68; // MPU6050のI2Cアドレス
struct IMUData {
  float accel[3];  // [m/s^2]
  float gyro[3];   // [deg/s]
  float temp;      // [°C]
};
IMUData imu_data;
// put function declarations here:
void init_pins();
void rotate_motor(int motor_num, int dir, int duty);
float get_angle(int val,int mot_num,int zero_val);
float adjust_angle(int stair_num,float pulley_angle);
void tca_select(uint8_t i);
void init_Serial();
void init_I2C();
void Serial_println(bool i2c_success);
void reset_encoder(bool tyre_only);
bool read_encoder();
bool read_IMU();
bool i2c_success = false;
void reset_pid();

void PID_rotate_stair(int mot_num,bool success);
void controller_commands(char c);
void control_motors(float deg_mot1,float deg_mot2,int dir_motor3,int spd_motor3){
  reset_pid();
  stair_target[0]=deg_mot1;
  stair_target[1]=deg_mot2;
  motor_dirs[2]=dir_motor3;
  motor_output[2]=spd_motor3;
}

// Timer callback function
void onTimer(void* arg) {
  i2c_flag = true;  // フラグだけ立てる（通信はここでしない）
  tmr_count++;
}

void setup() {
  // put your setup code here, to run once:
  init_pins();
  init_Serial();
  init_I2C();
  reset_encoder(true);
  Serial.println("Encoder calibrated");
  // Create a periodic timer that triggers every 10 ms  
  const esp_timer_create_args_t timer_args = {
    .callback = &onTimer,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "i2c_timer"
  };
  esp_timer_handle_t timer;
  esp_timer_create(&timer_args, &timer);
  esp_timer_start_periodic(timer, 5000); //5 ms
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
    controller_commands(Serial.read());
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


void init_pins(){
  pinMode(STAIR1_DIR_PIN1,OUTPUT);
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
  rotate_motor(1,0,0);
  rotate_motor(2,0,0);
  rotate_motor(3,0,0);
}

void init_Serial(){
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Serial initialized");
}

void init_I2C(){
  Wire.begin();
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(1 << 4);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1レジスタ
  Wire.write(0);    // スリープ解除
  Wire.endTransmission();
  Serial.println("I2C initialized");
}

void rotate_motor(int motor_num, int dir, int duty){
  switch(motor_num){
    case 1:// stair1
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
    case 2:// stair2
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
    case 3:// pulley
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

float get_angle(int val,int mot_num,int zero_val){
  int v_save=encoder_savedata[mot_num];
  encoder_savedata[mot_num]=val;
  // Handle wrap-around
  bool handle = false;
  if(val - v_save > 3000) raw_rot_count[mot_num]--;
  else if(v_save - val > 3000) raw_rot_count[mot_num]++;
  val += raw_rot_count[mot_num]*4096;
  float angle = (val - zero_val) * 360 / 4095.0f * Gear_Ratio[mot_num];
  switch(mot_num){
    case 2:
      angle -= (angle_rot_count[1]*360.0f + stair_angle[1])*(5.0f/6.0f);
      break;
    case 3:
      angle -= (angle_rot_count[0]*360.0f + stair_angle[0])*(5.0f/6.0f);
      break;
    default:
      break;
  }
  angle_rot_count[mot_num]=0;
  while(angle > 360) {
    angle -= 360;
    angle_rot_count[mot_num]++;
  }
  while(angle < 0) {
    angle += 360;
    angle_rot_count[mot_num]--;
  }
  return angle;
}

void tca_select(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(1 << i | 1 << 4);
  Wire.endTransmission();
}

void reset_encoder(bool tyre_only = false){
  read_encoder();
  for(char ch = 0; ch < 4; ch++){
    if(!(tyre_only && ch < 2)){
      ZERO[ch]=encoder_savedata[ch];
    }
    encoder_savedata[ch]=encoder_rawdata[ch];
    raw_rot_count[ch]=0;
    angle_rot_count[ch]=0;
  }
  stair_target[0]=0;
  stair_target[1]=0;
  Serial.print("ZERO1: ");
  Serial.print(ZERO[0]);
  Serial.print(", ZERO2: ");
  Serial.println(ZERO[1]);
}

bool read_encoder(){  
  for (char ch = 0; ch < 4; ch++) {
    tca_select(ch);
    Wire.beginTransmission(0x36);
    Wire.write(0x0C);
    Wire.endTransmission(false);
    Wire.requestFrom(0x36, 2);
    if(Wire.available() >= 2){
      byte val_h = Wire.read();
      byte val_l = Wire.read();
      unsigned int val_tmp = (0x0F & val_h) << 8 | val_l;
      encoder_rawdata[ch]=val_tmp;
    } else {
      Serial.print("I2C read error on channel ");
      Serial.println((int)ch);
      return false;
    }
  }
  stair_angle[0]=get_angle(encoder_rawdata[0],0,ZERO[0]);
  stair_angle[1]=get_angle(encoder_rawdata[1],1,ZERO[1]);
  Tyre_angle[0]=360 - get_angle(encoder_rawdata[2],2,ZERO[2]);
  Tyre_angle[1]=360 - get_angle(encoder_rawdata[3],3,ZERO[3]);
  return true;
}

bool read_IMU(){
  int16_t rawData[7]; // accelX, accelY, accelZ, temp, gyroX, gyroY, gyroZ
  Wire.beginTransmission(MPU_ADDR); // MPU6050のアドレス
  Wire.write(0x3B); // データレジスタ先頭
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  for (int i = 0; i < 7; i++) {
    rawData[i] = (Wire.read() << 8) | Wire.read();
  }

  // --- スケーリング ---
  imu_data.accel[0] = (float)rawData[0] / 16384.0 * 9.80665; // m/s^2
  imu_data.accel[1] = (float)rawData[1] / 16384.0 * 9.80665;
  imu_data.accel[2] = (float)rawData[2] / 16384.0 * 9.80665;

  imu_data.temp = (float)rawData[3] / 340.0 + 36.53; // °C

  imu_data.gyro[0] = (float)rawData[4] / 131.0; // °/s
  imu_data.gyro[1] = (float)rawData[5] / 131.0;
  imu_data.gyro[2] = (float)rawData[6] / 131.0;
  return true; // Return true if successful, false otherwise
}

// PID parameters
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

void Serial_println(bool i2c_success){
  if(i2c_success){
    char buf[255];
    sprintf(buf, "Stair1[%.2f,%.2f,%d], Stair2[%.2f,%.2f,%d], Tyre_deg[%.2f,%.2f] | IMU Accel: [%.2f, %.2f, %.2f], Gyro: [%.2f, %.2f, %.2f],Tmp: %d",
            stair_angle[0], stair_target[0], angle_rot_count[0],
            stair_angle[1], stair_target[1], angle_rot_count[1],
            Tyre_angle[0], Tyre_angle[1],
            imu_data.accel[0], imu_data.accel[1], imu_data.accel[2],
            imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2],
            (int)imu_data.temp);
            
    Serial.println(buf);
  } else {
    Serial.println("Error reading encoder");
  }
}

void controller_commands(char c){
  switch(c){
    case 'r':
      reset_encoder();
      Serial.println("Encoder recalibrated"); 
      break;
    case 'q':
      stop_flag = true;
      stair_target[0]=stair_angle[0];
      stair_target[1]=stair_angle[1];
      motor_output[0] = 0;
      motor_output[1] = 0;
      motor_output[2] = 0;  
      motor_dirs[0] = 0;
      motor_dirs[1] = 0;
      motor_dirs[2] = 0;
      rotate_motor(1,0,0);
      rotate_motor(2,0,0);
      rotate_motor(3,0,0);
      Serial.println("emergency stop");
      break;
    case 'w':
      control_motors(0,0,1,150);
      break;
    case 'a':
      control_motors(90,90,1,150);
      break;
    case 's':
      control_motors(180,180,1,150);
      break;
    case 'd':
      control_motors(270,270,1,150);
      break;
    case 'z':
      control_motors(0,180,1,150);
      break;
    case 'x':
      control_motors(180,0,1,150);
      break;
    default:
      break;
  } 
}