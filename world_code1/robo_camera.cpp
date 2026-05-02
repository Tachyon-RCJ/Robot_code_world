#include "robo_camera.hpp"

int fCameraSize = 0;
int rCameraSize = 0;
int lCameraSize = 0;
int rWorldSize = 0;
int lWorldSize = 0;
int fdistance = 0;
int rdistance = 0;
int ldistance = 0;
int returnValue[2] = {185, -1};
int surrounding[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int rads = 185; 
int gbrads = 185;
int gyrads = 185;
int head;
int goRad = 185;
int radsbr;
int radsbl;
int* ballRD;
String atack_goal_color = "blue";

String cameraLBuf = "";
String cameraRBuf = "";
String cameraFBuf = "";

unsigned long lastKickTime = 0;
int value_check;

int* cameraCheck(){
  if (Serial1.available() > 0) {
    cameraFBuf = Serial1.readStringUntil('e');

    int fIndex = cameraFBuf.indexOf('f');
    int bIndex = cameraFBuf.indexOf('b');
    int yIndex = cameraFBuf.indexOf('y');
    int lIndex = cameraFBuf.indexOf('l');
    int vIndex = cameraFBuf.indexOf('v');
    int aIndex = cameraFBuf.indexOf('a');
    int oIndex = cameraFBuf.indexOf('o');
    int pIndex = cameraFBuf.indexOf('p');
    int qIndex = cameraFBuf.indexOf('q');
    int rIndex = cameraFBuf.indexOf('r');
    int sIndex = cameraFBuf.indexOf('s');
    int tIndex = cameraFBuf.indexOf('t');
    int uIndex = cameraFBuf.indexOf('u');

    if (fIndex != -1 && bIndex != -1) {
      rads = cameraFBuf.substring(fIndex + 1, bIndex).toInt();
      rads = ((rads / 0.694) - 180);
      putPower = powermx;
    }

    if (bIndex != -1 && yIndex != -1 && atack_goal_color == "blue") {
      gbrads = cameraFBuf.substring(bIndex + 1, yIndex).toInt();
      gbrads = ((gbrads / 0.694) - 180);
    }

    if (yIndex != -1 && lIndex != -1 && atack_goal_color == "yellow") {
      gyrads = cameraFBuf.substring(yIndex + 1, lIndex).toInt();
      gyrads = ((gyrads / 0.694) - 180);
    }

    if (lIndex != -1 && vIndex != -1) {
      fdistance = cameraFBuf.substring(lIndex + 1, vIndex).toInt();
    }

    if (vIndex != -1 && aIndex != -1) {
      fCameraSize = cameraFBuf.substring(vIndex + 1, aIndex).toInt() * 3.3;
    }

    if (aIndex != -1 && oIndex != -1) {
      int kickcheck = cameraFBuf.substring(aIndex + 1, oIndex).toInt();
      if (digitalRead(22) == HIGH){
        if(kickcheck == 1) {
          unsigned long now = millis();
          if(now-lastKickTime > 500){
            kickMode = true;
            Serial.println("Kick Mode: ON");
            lastKickTime = now;
          }
        }
      }
    }

    if (oIndex != -1 && pIndex != -1) {
      value_check = cameraFBuf.substring(oIndex + 1, pIndex).toInt();
      if(value_check >= 0){
        surrounding[10] = value_check*2;
      }
    }

    if (pIndex != -1 && qIndex != -1) {
      value_check = cameraFBuf.substring(pIndex + 1, qIndex).toInt();
      if(value_check >= 0){
        surrounding[9] = value_check*2;
      }
    }

    if (qIndex != -1 && rIndex != -1) {
      value_check = cameraFBuf.substring(qIndex + 1, rIndex).toInt();
      if(value_check >= 0){
        surrounding[8] = value_check*2;
      }
    }

    if (rIndex != -1 && sIndex != -1) {
      value_check = cameraFBuf.substring(rIndex + 1, sIndex).toInt();
      if(value_check >= 0){
        surrounding[7] = value_check*2;
      }
    }

    if (sIndex != -1 && tIndex != -1) {
      value_check = cameraFBuf.substring(sIndex + 1, tIndex).toInt();
      if(value_check >= 0){
        surrounding[6] = value_check*2;
      }
    }

    if (tIndex != -1 && uIndex != -1) {
      value_check = cameraFBuf.substring(tIndex + 1, uIndex).toInt();
      if(value_check >= 0){
        surrounding[5] = value_check*2;
      }
    }

    if (uIndex != -1) {
      value_check = cameraFBuf.substring(uIndex + 1).toInt();
      if(value_check >= 0){
        surrounding[4] = value_check*2;
      }
    }
  }  

  //rightカメラ
  if (mySerial1.available() > 0){
    //radsbrStr = "";
    cameraRBuf = "";
    cameraRBuf = mySerial1.readStringUntil('e');
    int r1Index = cameraRBuf.indexOf('r');
    int s1Index = cameraRBuf.indexOf('s');
    int l1Index = cameraRBuf.indexOf('l');
    int w1Index = cameraRBuf.indexOf('w');
    //int b1Index = cameraRBuf.indexOf('b');
    int f1Index = cameraRBuf.indexOf('f');
    int g1Index = cameraRBuf.indexOf('g');
    int h1Index = cameraRBuf.indexOf('h');
    int i1Index = cameraRBuf.indexOf('i');
    int j1Index = cameraRBuf.indexOf('j');
    
    if (r1Index != -1 && s1Index != -1) {
      radsbr = cameraRBuf.substring(r1Index + 1, s1Index).toInt();
    }
    if (s1Index != -1 && l1Index != -1) {
      rCameraSize = cameraRBuf.substring(s1Index + 1, l1Index).toInt();
    }
    if (l1Index != -1 && w1Index != -1) {
      rdistance = cameraRBuf.substring(l1Index + 1, w1Index).toInt();
    }
    if (w1Index != -1 && f1Index != -1) {
      rWorldSize = cameraRBuf.substring(w1Index + 1, f1Index).toInt();
    }
    //if (b1Index != -1 && f1Index != -1) {
      //rWorldSize = cameraRBuf.substring(b1Index + 1, f1Index).toInt();
    //}
    
    if (f1Index != -1 && g1Index != -1) {
      value_check = cameraRBuf.substring(f1Index + 1, g1Index).toInt();
      if(value_check >= 0){
        surrounding[15] = value_check*2;
      }
    }
    if (g1Index != -1 && h1Index != -1) {
      value_check = cameraRBuf.substring(g1Index + 1, h1Index).toInt();
      if(value_check >= 0){
        surrounding[14] = value_check*2;
      }
    }
    if (h1Index != -1 && i1Index != -1) {
      value_check = cameraRBuf.substring(h1Index + 1, i1Index).toInt();
      if(value_check >= 0){
        surrounding[13] = value_check*2;
      }
    }
    if (i1Index != -1 && j1Index != -1) {
      value_check = cameraRBuf.substring(i1Index + 1, j1Index).toInt();
      if(value_check >= 0){
        surrounding[12] = value_check*2;
      }
    }
    if (j1Index != -1) {
      int nextIndex = j1Index + 1;
      while (nextIndex < cameraRBuf.length() && isDigit(cameraRBuf[nextIndex])) {
        nextIndex++;
      }
      value_check = cameraRBuf.substring(j1Index + 1, nextIndex).toInt();
      if(value_check >= 0){
        surrounding[11] = value_check*2;
      }
    }
      
  }
  
  //leftカメラ
  if (mySerial2.available() > 0){
    cameraLBuf = "";
    cameraLBuf = mySerial2.readStringUntil('e');
    int r2Index = cameraLBuf.indexOf('r');
    int s2Index = cameraLBuf.indexOf('s');
    int l2Index = cameraLBuf.indexOf('l');
    int w2Index = cameraLBuf.indexOf('w');
    //int b2Index = cameraLBuf.indexOf('b');
    int f2Index = cameraLBuf.indexOf('f');
    int g2Index = cameraLBuf.indexOf('g');
    int h2Index = cameraLBuf.indexOf('h');
    int i2Index = cameraLBuf.indexOf('i');
    int j2Index = cameraLBuf.indexOf('j');
    if (r2Index != -1 && s2Index != -1) {
      radsbl = cameraLBuf.substring(r2Index + 1, s2Index).toInt();
    }
    if (s2Index != -1 && l2Index != -1) {
      lCameraSize = cameraLBuf.substring(s2Index + 1, l2Index).toInt();
    }
    if (l2Index != -1 && w2Index != -1) {
      ldistance = cameraLBuf.substring(l2Index + 1, w2Index).toInt();
    }
    if (w2Index != -1 && f2Index != -1) {
      lWorldSize = cameraLBuf.substring(w2Index + 1, f2Index).toInt();
    }
    //if (b2Index != -1 && f2Index != -1) {
      //lWorldSize = cameraLBuf.substring(b2Index + 1, f2Index).toInt();
    //}
    if (f2Index != -1 && g2Index != -1) {
      value_check = cameraLBuf.substring(f2Index + 1, g2Index).toInt();
      if(value_check >= 0){
        surrounding[15] = max(surrounding[15],value_check*2);
      }
    }
    if (g2Index != -1 && h2Index != -1) {
      value_check = cameraLBuf.substring(g2Index + 1, h2Index).toInt();
      if(value_check >= 0){
        surrounding[0] = value_check*2;
      }
    }
    if (h2Index != -1 && i2Index != -1) {
      value_check = cameraLBuf.substring(h2Index + 1, i2Index).toInt();
      if(value_check >= 0){
        surrounding[1] = value_check*2;
      }
    }
    if (i2Index != -1 && j2Index != -1) {
      value_check = cameraLBuf.substring(i2Index + 1, j2Index).toInt();
      if(value_check >= 0){
        surrounding[2] = value_check*2;
      }
    }
    if (j2Index != -1) {
      int next2Index = j2Index + 1;
      while (next2Index < cameraLBuf.length() && isDigit(cameraLBuf[next2Index])) {
        next2Index++;
      }
      value_check = cameraLBuf.substring(j2Index + 1, next2Index).toInt();
      if(value_check >= 0){
        surrounding[3] = value_check*2;
      }
    }
      
    
  }
    
    
  /*
  Serial.print(String(fCameraSize));
  Serial.print(", ");
  Serial.print(rCameraSize);
  Serial.print(", ");
  Serial.print(lCameraSize);
  Serial.print(": ");
  */
  /*
  Serial.print("distance: ");
  Serial.print(String(fdistance));
  Serial.print(", ");
  Serial.print(rdistance);
  Serial.print(", ");
  Serial.print(ldistance);
  */
 
  int RrWorldSize = int(rWorldSize*1);//補正
  /*
  Serial.print("right");
  Serial.print(String(cameraRBuf));
  Serial.print(" | left");
  Serial.print(String(cameraLBuf));
  Serial.print(" | homeRad");
  Serial.println(String(backhome(rWorldSize, lWorldSize)));
  */
  /*
  Serial.print("right");
  Serial.print(String(rWorldSize));
  Serial.print(", ");
  Serial.print(String(RrWorldSize));
  Serial.print(" | left");
  Serial.println(String(lWorldSize));
  */
  /*
  if(abs(RrWorldSize-lWorldSize) <= 70){
    pixels.setPixelColor(0, pixels.Color(0,0,0));
    pixels.setPixelColor(1, pixels.Color(0,50,0));
    pixels.setPixelColor(2, pixels.Color(0,0,0));
    pixels.show();
    return 185;
  } else if(RrWorldSize >= lWorldSize){
    pixels.setPixelColor(0, pixels.Color(0,50,0));
    pixels.setPixelColor(1, pixels.Color(0,0,0));
    pixels.setPixelColor(2, pixels.Color(0,0,0));
    pixels.show();
    return 90;
  } else if(RrWorldSize < lWorldSize){
    pixels.setPixelColor(0, pixels.Color(0,0,0));
    pixels.setPixelColor(1, pixels.Color(0,0,0));
    pixels.setPixelColor(2, pixels.Color(0,50,0));
    pixels.show();
    return -90;
  }
  */  
  //return 185;

  //ゴール方向計算
  goalMarking();
  
  
  if(fCameraSize <= 1){
    rads = 185;
  }
  if(rCameraSize <= 4){
    radsbr = 185;
  }
  if(lCameraSize <= 4){
    radsbl = 185;
  }
  
  int maxBallSize = fCameraSize;
  if(rCameraSize > maxBallSize) maxBallSize = rCameraSize;
  if(lCameraSize > maxBallSize) maxBallSize = lCameraSize;

  /*
  Serial.print(",");
  Serial.print(rads);
  Serial.print(",");
  Serial.print(radsbr);
  Serial.print(",");
  Serial.print(radsbl);
  Serial.print(",");
  Serial.print(maxBallSize);
  */
  
  if(rads > 181 && radsbr > 181 && radsbl > 181){
    //Serial.print("no");
    //returnValue[0] = backhome(rWorldSize, lWorldSize);
    returnValue[0] = kaihi_muki_k;
    returnValue[1] = -1;
    return returnValue;
  }else if(fCameraSize == maxBallSize){
    //Serial.print("front");
    returnValue[0] = rads;
    returnValue[1] = fdistance;
    return returnValue;
  } else if(rCameraSize == maxBallSize){
    //Serial.print("right");
    returnValue[0] = radsbr;
    returnValue[1] = rdistance;
    return returnValue;
  } else if(lCameraSize == maxBallSize){
    //Serial.print("left");
    returnValue[0] = radsbl;
    returnValue[1] = ldistance;
    return returnValue;
  }
  returnValue[0] = kaihi_muki_k;
  returnValue[1] = -1;
  return returnValue;
}

void goalMarking(){
  float grad;
  int bGoalRads = 0;//LiDARから取得したゴール方向の総和
  int bGoalCount = 0;//ゴール方向の入力回数
  int yGoalRads = 0;//LiDARから取得したゴール方向の総和
  int yGoalCount = 0;//ゴール方向の入力回数
  for(int p = 0; p < 16; p++){
    if(p <= 3 || p >= 11){
      if(p >= 0 && p < 5){
        if(p == 0) grad = -22.5;
        else if(p == 1) grad = -45; 
        else if(p == 2) grad = -67.5; 
        else if(p == 3) grad = -90; 
      } else {
        if(p == 11) grad = 90;
        else if(p == 12) grad = 67.5; 
        else if(p == 13) grad = 45; 
        else if(p == 14) grad = 22.5; 
        else if(p == 15) grad = 0;
      }
      if(surrounding[p] % 10 == 2){
        bGoalRads += grad;
        bGoalCount++;
      } else if(surrounding[p] % 10 == 4){
        yGoalRads += grad;
        yGoalCount++;
      } else if(surrounding[p] % 10 == 6){
        bGoalRads += grad;
        bGoalCount++;
        yGoalRads += grad;
        yGoalCount++;
      }
    }
  }
  if(atack_goal_color == "blue"){
    if(yGoalCount > 0){
      gyrads = 180 - (yGoalRads / yGoalCount);
      if(gyrads < -180){//絶対値が180を超えないように
        gyrads = 180 - (-180-gyrads);
      } else if(gyrads > 180){
        gyrads = -180 + (gyrads-180);
      }
    } else {
      gyrads = 185;
    }
  } else if(atack_goal_color == "yellow"){
    if(bGoalCount > 0){
      gbrads = 180 - (bGoalRads / bGoalCount);
      if(gbrads < -180){//絶対値が180を超えないように
        gbrads = 180 - (-180-gbrads);
      } else if(gbrads > 180){
        gbrads = -180 + (gbrads-180);
      }
    } else {
      gbrads = 185;
    }
  }
}