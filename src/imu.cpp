#include <Arduino.h>
#include <Wire.h>
#include "imu.h"
#include "hardware.h"

IMUData imu_data;
bool read_IMU(){
  int16_t rawData[7]; // accelX, accelY, accelZ, temp, gyroX, gyroY, gyroZ
  Wire.beginTransmission(IMU_ADDR); // MPU6050のアドレス
  Wire.write(0x3B); // データレジスタ先頭
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDR, 14, true);

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