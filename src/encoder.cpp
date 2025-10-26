#include "encoder.h"
#include <Wire.h>
#include "Arduino.h"
#include "hardware.h"
#include "motor_control.h"

// External encoder data
int ZERO[4] = {1226, 1281,0,0}; //encoder zero position stair1, stair2, pulley1,pulley2
int angle_rot_count[4]={0,0,0,0};
float stair_angle[2]={0,0};
float Tyre_angle[2]={0,0};
bool i2c_success = false;

// Internal encoder data
int encoder_rawdata[4] = {0,0,0,0};
int encoder_savedata[4] = {0,0,0,0};
int raw_rot_count[4] = {0,0,0,0};
float Gear_Ratio[4] = {8.0/60, 8.0/60, 1/1.2, 1/1.2};// stair1, stair2, pulley1, pulley2


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
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i | 1 << 4);
  Wire.endTransmission();
}

void reset_encoder(bool tyre_only){
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
    Wire.beginTransmission(ENC_ADDR);
    Wire.write(0x0C);
    Wire.endTransmission(false);
    Wire.requestFrom(ENC_ADDR, 2);
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
