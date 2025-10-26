#include "hardware.h"
#include "encoder.h"
#include "motor_control.h"
#include "imu.h"
#include <Arduino.h>

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

bool controller_commands(char c){
  switch(c){
    case 'r':
      reset_encoder();
      Serial.println("Encoder recalibrated"); 
      break;
    case 'q':
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
      return false;
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
  return true;
}