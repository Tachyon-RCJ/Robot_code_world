#include "robo_mode.hpp"

int backhome(){
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

int roboGoRad(int r, int d){
  /*
  Serial.print(" r:");
  Serial.print(String(r));
  Serial.print(" d:");
  Serial.println(String(d));
  */
  if(abs(r) > 181){
      return 185;
  }
  int reRad = r*1.5;
  
  if(d != -1){
    if((45 <= abs(r) <= 70) && (d < 90)){//機体の近くのボールは大きめに回り込む
      reRad = r*2.8;
    }
    if(abs(r) > 110){ //遠かったら近くまで行く
      if(d > 190){
        reRad = r;
      }
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