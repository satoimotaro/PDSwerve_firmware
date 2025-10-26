#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#define TCA_ADDRESS 0x70

extern int ZERO[4];
extern float stair_angle[2];
extern float Tyre_angle[2];
extern int angle_rot_count[4];

extern bool i2c_success;

void tca_select(uint8_t i);
bool read_encoder();
void reset_encoder(bool tyre_only = false);
float get_angle(int val, int mot_num, int zero_val);

#endif // ENCODER_H
