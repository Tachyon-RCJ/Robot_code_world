#ifndef ROBO_GYRO_H
#define ROBO_GYRO_H

#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

extern MPU6050 mpu;
extern float ypr[3], raw_ypr[3], offset_ypr[3];
extern float jairo;
extern bool dmpReady;
extern uint8_t mpuIntStatus;
extern uint8_t devStatus;
extern uint16_t packetSize;
extern uint16_t fifoCount;
extern uint8_t fifoBuffer[64];

void mpu_setup();
float getJairo();
void setJairo();

#endif