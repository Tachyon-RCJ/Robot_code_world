#include "robo_gyro.hpp"
#include <math.h>

MPU6050 mpu;
float ypr[3], raw_ypr[3], offset_ypr[3];
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

void mpu_setup() {
    Wire.begin();
    Wire.setClock(400000);
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setDMPEnabled(true);
    mpu.setXGyroOffset(27);
    mpu.setYGyroOffset(-40);
    mpu.setZGyroOffset(85);
    mpu.setZAccelOffset(1105);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
    setJairo();
}

float getJairo(){
  mpuIntStatus = mpu.getIntStatus();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    delay(10);
  }
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount(); 
  fifoBuffer[64];
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  fifoCount -= packetSize;
  Quaternion q;
  VectorFloat gravity;
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(raw_ypr, &q, &gravity);
  ypr[0] = raw_ypr[0] - offset_ypr[0];
  return ypr[0] * 180/M_PI;
}
void setJairo(){
  offset_ypr[0] = raw_ypr[0];
}