#ifndef ROBO_MODE_H
#define ROBO_MODE_H

extern float kaihi_x_k;
extern float kaihi_y_k;
extern int goPosition[2];
extern int positionRange;

int backhome();
int roboGoRad(int r, int d);

#endif