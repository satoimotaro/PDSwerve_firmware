#ifndef IMU_H
#define IMU_H

struct IMUData {
  float accel[3];  // [m/s^2]
  float gyro[3];   // [deg/s]
  float temp;      // [°C]
};

extern IMUData imu_data;

bool read_IMU();

#endif // IMU_H
