#include "robo_line.hpp"
#include <LittleFS.h>
#define PINNO 13
#define BEAT 200

int preRads;
int lineFound[2] = {0, 0};
int outTime = 200;
int stopTime = 100;
unsigned long startTime;
const unsigned long settingTime = 7000;

float kaihi_x = 0;
float kaihi_y = 0;
float kaihi_speed = 0;
float kaihi_muki = 0;
float kaihi_x_k = 0;
float kaihi_y_k = 0;
float kaihi_speed_k = 0;
float kaihi_muki_k = 0;

void loodLineSet(){
  /*
  File file = LittleFS.open("/libo.txt", "r");
  String readLineBorder = file.readString();
  file.close();
  int loodBorderIndex = 0;
  int loodBorderStart = 0;
  int loodBorderEnd = readLineBorder.indexOf(',');
  while (loodBorderEnd >= 0) {
    lineOutVal[loodBorderIndex++] = readLineBorder.substring(loodBorderStart, loodBorderEnd).toInt();
    loodBorderStart = loodBorderEnd + 1;
    loodBorderEnd = readLineBorder.indexOf(',', loodBorderStart);
  }
  lineOutVal[loodBorderIndex++] = readLineBorder.substring(loodBorderStart).toInt();
  char BoderSendBuffer[100];
  sprintf(BoderSendBuffer,"LineBorder:26P:%d, 27P:%d, 28P:%d, 29P:%d", lineOutVal[0], lineOutVal[1], lineOutVal[2], lineOutVal[3]);
  mySerial.println(BoderSendBuffer);
  */
}

void lineSet(int i) {
  tone(PINNO,370,BEAT) ; // ファ#
  delay(BEAT) ;
  tone(PINNO,293,BEAT) ; // レ
  delay(BEAT) ;
  tone(PINNO,220,BEAT) ; // ラ
  delay(BEAT) ;
  int lineMaxVal = 0;
  int lineMinVal = 255;
  startTime = millis();
  const unsigned long sendInterVal = 10;
  int sendCount = 0;
  while (millis() - startTime < settingTime) {
    int value = analogRead(26+i);
    //最大値最小値確認
    if (value > lineMaxVal){
      lineMaxVal = value;
    }
    if (value < lineMinVal){
      lineMinVal = value;
    }
    if (value > (lineMaxVal-lineMinVal)/2.0){
      tone(PINNO,220,BEAT) ; // ラ
    }
    if((int)((millis() - startTime)/sendInterVal) > sendCount){
      sendCount++;
      mySerial.print("WDS/LSVal/");
      mySerial.println(String(value));
    }
  }
  lineOutVal[i] = (lineMaxVal + lineMinVal) / 2.0;
  tone(PINNO,330,BEAT) ; // ミ
  delay(BEAT) ;
  tone(PINNO,330,BEAT) ; // ミ
  delay(BEAT) ;
  File file = LittleFS.open("/libo.txt", "w");
  String writeLineBorder = String(lineOutVal[0])+","+String(lineOutVal[1])+","+String(lineOutVal[2])+","+String(lineOutVal[3]);
  file.print(writeLineBorder);
  file.close();
  mySerial.print("WDS/LBorV/");
  mySerial.println(String(lineOutVal[i]));
}

bool lineCheck(int *sensorVal) {
    preRads = 0; // 必要に応じて修正
    int found = 0;
    if (sensorVal[0] > lineOutVal[0]) found++;
    if (sensorVal[1] > lineOutVal[1]) found++;
    if (sensorVal[2] > lineOutVal[2]) found++;
    if (sensorVal[3] > lineOutVal[3]) found++;
    if (found != 0){
      tone(PINNO,330,BEAT) ; // ミ
      MoterSerialPR(powermx,preRads+180);
      delay(stopTime);
      MoterSerialPR(powermx,kaihi_muki_k);
      delay(outTime);
      return false;
    } else {
      return true;
    }
}

void kaihi_check(){
  kaihi_x = 0;
  kaihi_y = 0;
  
  kaihi_x += surrounding[0] * sin(-2.75);
  kaihi_x += surrounding[1] * sin(-2.36);
  kaihi_x += surrounding[2] * sin(-1.96);
  kaihi_x += surrounding[3] * sin(-1.57);
  kaihi_x += surrounding[4] * sin(-1.18);
  kaihi_x += surrounding[5] * sin(-0.79);
  kaihi_x += surrounding[6] * sin(-0.39);
  kaihi_x += surrounding[7] * sin(0);
  kaihi_x += surrounding[8] * sin(0.39);
  kaihi_x += surrounding[9] * sin(0.79);
  kaihi_x += surrounding[10] * sin(1.18);
  kaihi_x += surrounding[11] * sin(1.57);
  kaihi_x += surrounding[12] * sin(1.96);
  kaihi_x += surrounding[13] * sin(2.36);
  kaihi_x += surrounding[14] * sin(2.75);
  kaihi_x += surrounding[15] * sin(3.14);
  
  kaihi_y += surrounding[0] * cos(-2.75);
  kaihi_y += surrounding[1] * cos(-2.36);
  kaihi_y += surrounding[2] * cos(-1.96);
  kaihi_y += surrounding[3] * cos(-1.57);
  kaihi_y += surrounding[4] * cos(-1.18);
  kaihi_y += surrounding[5] * cos(-0.79);
  kaihi_y += surrounding[6] * cos(-0.39);
  kaihi_y += surrounding[7] * cos(0);
  kaihi_y += surrounding[8] * cos(0.39);
  kaihi_y += surrounding[9] * cos(0.79);
  kaihi_y += surrounding[10] * cos(1.18);
  kaihi_y += surrounding[11] * cos(1.57);
  kaihi_y += surrounding[12] * cos(1.96);
  kaihi_y += surrounding[13] * cos(2.36);
  kaihi_y += surrounding[14] * cos(2.75);
  kaihi_y += surrounding[15] * sin(3.14);

  kaihi_speed = sqrt(pow(kaihi_x, 2) + pow(kaihi_y, 2));

  if(kaihi_speed != 0){
    kaihi_x_k = kaihi_x;
    kaihi_y_k = kaihi_y;
    kaihi_speed_k = kaihi_speed;
    kaihi_muki_k = -(atan2(kaihi_y,kaihi_x)*(180/PI) - 90);
    if(kaihi_muki_k < -180){
      kaihi_muki_k += 360;
    } else if(kaihi_muki_k > 180){
      kaihi_muki_k -= 360;
    }
  }
}