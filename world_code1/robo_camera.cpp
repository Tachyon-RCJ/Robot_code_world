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

unsigned long lastKickTime = 0;
int value_check;

int* cameraCheck(){
  if (Serial1.available() > 0) {
    head = Serial1.read();
    if (head == 'f') {
      rads = Serial1.read();
      rads = ((rads/0.694) - 180);
      putPower = powermx;
    }
    head = Serial1.read();
    if (head == 'b') {
      gbrads = Serial1.read();
      gbrads = ((gbrads/0.694) - 180);
    }
    head = Serial1.read();
    if (head == 'y') {
      gyrads = Serial1.read();
      gyrads = ((gyrads/0.694) - 180);
    }
    head = Serial1.read();
    if (head == 'l') {
      fdistance = Serial1.read();
    }
    head = Serial1.read();
    if (head == 'v') {
      fCameraSize = Serial1.read()*3.3;
    }
    head = Serial1.read();
    if (head == 'a') {
      int kickcheck = Serial1.read();
      if(kickcheck == 1) {
        unsigned long now = millis();
        if(now-lastKickTime > 500){
          kickMode = true;
          Serial.println("Kick Mode: ON");
          lastKickTime = now;
        }
      }
    }
    head = Serial1.read();
    if (head == 'o') {
      value_check = Serial1.read();
      if(value_check >= 0){
        surrounding[10] = value_check*2;
      }
    }
    head = Serial1.read();
    if (head == 'p') {
      value_check = Serial1.read();
      if(value_check >= 0){
        surrounding[9] = value_check*2;
      }
    }
    head = Serial1.read();
    if (head == 'q') {
      value_check = Serial1.read();
      if(value_check >= 0){
        surrounding[8] = value_check*2;
      }
    }
    head = Serial1.read();
    if (head == 'r') {
      value_check = Serial1.read();
      if(value_check >= 0){
        surrounding[7] = value_check*2;
      }
    }
    head = Serial1.read();
    if (head == 's') {
      value_check = Serial1.read();
      if(value_check >= 0){
        surrounding[6] = value_check*2;
      }
    }
    head = Serial1.read();
    if (head == 't') {
      value_check = Serial1.read();
      if(value_check >= 0){
        surrounding[5] = value_check*2;
      }
    }
    head = Serial1.read();
    if (head == 'u') {
      value_check = Serial1.read();
      if(value_check >= 0){
        surrounding[4] = value_check*2;
      }
    /*
    } else if (head == 'c') {
      shootSubSend = false;
      shootSend = true;
      tone(PINNO,370,BEAT) ; // ファ#
      delay(BEAT) ;
      tone(PINNO,330,BEAT) ; // ミ
      delay(BEAT) ;
      tone(PINNO,220,BEAT) ; // ラ
      delay(BEAT) ;
      tone(PINNO,293,BEAT) ; // レ
    } else if (head == 's') {
      blshoot(0);
      mySerial1.print("ble");
      mySerial2.print("ble");
      Serial1.print("ble");
      drMode = false;
      drCatchMode = false;
      shootSubSend = false;
      shootSend = false;
      digitalWrite(19, LOW);
      */
    }
    //Serial.println(rads);
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
    /*
    if (cameraRBuf.charAt(cameraRBufLen - 1) == 'd'){
      cameraRBuf.remove(cameraRBufLen - 1);
      radsbrStr = cameraRBuf;
      drCatchMode = true;
    } else if (cameraRBuf.charAt(cameraRBufLen - 1) == 'r'){
      cameraRBuf.remove(cameraRBufLen - 1);
      radsbrStr = cameraRBuf;
      drCatchMode = false;
    } else if (cameraRBuf.charAt(cameraRBufLen - 1) == 's'){
      cameraRBuf.remove(cameraRBufLen - 1);
      rCameraSize = cameraRBuf.toInt();
    } else {
      drCatchMode = false;
      radsbrStr = cameraRBuf;
    }
    if (cameraRBuf == "blb"){
      radsbrStr = "185";
      drMode = true;
      digitalWrite(19, HIGH);
      mySerial2.print("blb");
      mySerial1.print("blb");
      Serial1.print("blb");
    } else if (cameraRBuf == "ble"){
      radsbrStr = "185";
      drMode = false;
      digitalWrite(19, LOW);
      mySerial2.print("ble");
      mySerial1.print("ble");
      Serial1.print("ble");
    } else if (cameraRBuf == "bls" && (!shootSubSend || millis() - sendTime >= timeout) && !shootSend){
      Serial1.print("bls");
      sendTime = millis();
      shootSubSend = true;
      tone(PINNO,293,BEAT) ; // レ
    }
    */
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
        surrounding[0] = value_check*2;
      }
    }
    /*
    int cameraLBufLen = cameraLBuf.length();
    if (cameraLBuf.charAt(cameraLBufLen - 1) == 'd'){
      cameraLBuf.remove(cameraLBufLen - 1);
      radsblStr = cameraLBuf;
      drCatchMode = true;
    } else if (cameraLBuf.charAt(cameraLBufLen - 1) == 'r'){
      cameraLBuf.remove(cameraLBufLen - 1);
      radsblStr = cameraLBuf;
      drCatchMode = false;
    } else if (cameraLBuf.charAt(cameraLBufLen - 1) == 's'){
      cameraLBuf.remove(cameraLBufLen - 1);
      lCameraSize = cameraLBuf.toInt();
    } else {
      drCatchMode = false;
      radsblStr = cameraLBuf;
    }
    if (radsblStr == "blb"){
      radsblStr = "185";
      drMode = true;
      digitalWrite(19, HIGH);
      mySerial1.print("blb");
      mySerial2.print("blb");
      Serial1.print("blb");
    } else if (radsblStr == "ble"){
      radsblStr = "185";
      drMode = false;
      digitalWrite(19, LOW);
      mySerial1.print("ble");
      mySerial2.print("ble");
      Serial1.print("ble");
    }
    */
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