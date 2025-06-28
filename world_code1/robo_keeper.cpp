#include <Arduino.h>
#include "robo_mode.hpp"

int k_backhome(){
  int xposition = kaihi_x_k - goPosition[0];
  int yposition = kaihi_y_k - goPosition[1];
  if(abs(xposition) <= positionRange && abs(yposition) <= positionRange) return 185;
  if(xposition >= 0 && abs(yposition) <= positionRange) return 90;
  if(xposition < 0 && abs(yposition) <= positionRange) return -90;
  if(yposition >= 0 && abs(xposition) <= positionRange) return 0;
  if(yposition < 0 && abs(xposition) <= positionRange) return 180;
  if(xposition >= 0 && yposition >= 0) return 45;
  if(xposition >= 0 && yposition < 0) return 135;
  if(xposition < 0 && yposition >= 0) return -45;
  if(xposition < 0 && yposition < 0) return -135;
  return 185;
}

int k_roboGoRad(int r, int d){
  if(abs(r) > 181){
      return 185;
  }
  int reRad;
  
  if(d != -1){
    if(abs(r) > 20){
        return 185;
    } else if(r > 0){
        reRad = 90;
    } else if(r < 0){
        reRad = -90;
    }
  } else {
    reRad = r;
  }

  if(reRad < -180){//絶対値が180を超えないように
    reRad = 180 - (-180-reRad);
  } else if(reRad > 180){
    reRad = -180 + (reRad-180);
  }
  return reRad;
}