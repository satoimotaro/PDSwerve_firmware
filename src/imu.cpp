#include <Arduino.h>
#include <Wire.h>
#include "imu.h"
#include "hardware.h"
#include <MadgwickAHRS.h>
#include <sensor_msgs/msg/imu.h>

IMUData imu_data;
Madgwick filter;
void init_IMU(){
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0);    // Set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    Serial.println("IMU initialized");
    filter.begin(100); // Set the filter update rate to 100 Hz
}

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
    imu_data.accel[0] = (float)rawData[0] / 16384.0;
    imu_data.accel[1] = (float)rawData[1] / 16384.0;
    imu_data.accel[2] = (float)rawData[2] / 16384.0;

    imu_data.temp = (float)rawData[3] / 340.0 + 36.53; // °C

    imu_data.gyro[0] = (float)rawData[4] / 131.0 * DEG_TO_RAD; // rad/s
    imu_data.gyro[1] = (float)rawData[5] / 131.0 * DEG_TO_RAD;
    imu_data.gyro[2] = (float)rawData[6] / 131.0 * DEG_TO_RAD;
    // --- センサフュージョンフィルタ更新 ---
    filter.updateIMU(imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2],
                     imu_data.accel[0], imu_data.accel[1], imu_data.accel[2]);
    return true; // Return true if successful, false otherwise
}

void imu_msg_fill(sensor_msgs::msg::Imu &imu_msg){
    // クォータニオンの設定
    imu_msg.orientation.x = filter.getQuaternionX();
    imu_msg.orientation.y = filter.getQuaternionY();
    imu_msg.orientation.z = filter.getQuaternionZ();
    imu_msg.orientation.w = filter.getQuaternionW();
    imu_msg.linear_acceleration.x = imu_data.accel[0] * 9.81; // Convert to m/s^2
    imu_msg.linear_acceleration.y = imu_data.accel[1] * 9.81;
    imu_msg.linear_acceleration.z = imu_data.accel[2] * 9.81;
    imu_msg.angular_velocity.x = imu_data.gyro[0];
    imu_msg.angular_velocity.y = imu_data.gyro[1];
    imu_msg.angular_velocity.z = imu_data.gyro[2];
}