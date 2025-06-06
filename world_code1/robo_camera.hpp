#ifndef ROBO_CAMERA_H
#define ROBO_CAMERA_H

#include <Arduino.h>
#include <SoftwareSerial.h>
// グローバル変数のextern宣言
extern int fCameraSize, rCameraSize, lCameraSize;
extern int rWorldSize, lWorldSize;
extern int fdistance, rdistance, ldistance;
extern int returnValue[2];
extern int rads, radsbr, radsbl;
extern String cameraLBuf, cameraRBuf;
extern float kaihi_muki_k;
extern int* ballRD;
extern String atack_goal_color;

extern int head;
extern int putPower;
extern int powermx;
extern int gbrads;
extern int gyrads;
extern bool kickMode;
extern SoftwareSerial mySerial1; // RX, TX
extern SoftwareSerial mySerial2; // RX, TX


// 関数プロトタイプ
int* cameraCheck();

#endif