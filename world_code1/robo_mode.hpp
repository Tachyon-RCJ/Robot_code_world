#ifndef ROBO_MODE_H
#define ROBO_MODE_H

extern float kaihi_x_k;
extern float kaihi_y_k;
extern int lineVal[4];
extern int lineOutVal[4];
extern int goPosition[2];
extern int positionRange;

int backhome();
int k_backhome();
int a_roboGoRad(int r, int d);
int k_roboGoRad(int r, int d);

#endif