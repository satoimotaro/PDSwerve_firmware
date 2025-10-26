#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

#define PWM_FREQ 500
#define DEAD_TIME 100
#define EFFECTIVE_DUTY_MIN 40

// モーター番号定義
enum MotorID { STAIR1 = 1, STAIR2 = 2, PULLEY = 3 };

// モーター制御関数
void rotate_motor(int motor_num, int dir, int duty);

// PID制御関数
void PID_rotate_stair(int mot_num, bool success);
void reset_pid();

// 外部からの指令制御
void control_motors(float deg_mot1, float deg_mot2, int dir_motor3, int spd_motor3);
void stop_motors();

// グローバルモータ状態
extern int motor_output[3];
extern int motor_dirs[3];
extern float stair_target[2];

#endif // MOTOR_CONTROL_H
