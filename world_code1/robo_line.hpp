#ifndef ROBO_LINE_H
#define ROBO_LINE_H

#include <Arduino.h>

extern int preRads;
extern int lineFound[2];
extern int lineVal[4];
extern int lineOutVal[4];
extern int outTime;
extern int stopTime;
extern unsigned long startTime;
extern const unsigned long settingTime;

extern float kaihi_x;
extern float kaihi_y;
extern float kaihi_speed;
extern float kaihi_muki;
extern float kaihi_x_k;
extern float kaihi_y_k;
extern float kaihi_speed_k;
extern float kaihi_muki_k;
extern int surrounding[16];

void loodLineSet();
void lineSet(int i);
bool lineCheck();
void kaihi_check();

#endif