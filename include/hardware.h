#ifndef HARDWARE_INIT_H
#define HARDWARE_INIT_H

#define STAIR1_DIR_PIN1 33
#define STAIR1_DIR_PIN2 27
#define STAIR2_DIR_PIN1 26
#define STAIR2_DIR_PIN2 25
#define PULLEY_DIR_PIN1 32
#define PULLEY_DIR_PIN2 23
#define PWM_FREQ 500

#define TCA_ADDR 0x70
#define IMU_ADDR 0x68
#define ENC_ADDR 0x36

void init_pins();
void init_Serial();
void init_I2C();
void init_timer();

#endif // HARDWARE_INIT_H
