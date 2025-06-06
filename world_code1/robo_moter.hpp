#ifndef ROBO_MOTER_H
#define ROBO_MOTER_H

#include <Arduino.h>
#include <PID_v1.h>

extern String M1, M2, M3, M4;
extern String mchange[16];
extern float intoutput;
extern float Moterhelp[4];
extern int powermx;
extern int putPower;

extern double Setpoint, Input, Output;
extern double Kp, Ki, Kd;
extern PID pid;
extern int moutmax;
extern bool strongTurn;

int pidCalculate(float p);
void MoterSerial(int m1, int m2, int m3, int m4);
void MoterSerialPR(int p, int r);

#endif