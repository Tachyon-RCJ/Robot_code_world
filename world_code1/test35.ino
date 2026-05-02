//test21:新しい動作システムのテスト（面積が最大のカメラを選択）+ボールなかったら中心へ+RP2040で進行方向計算
#include <Arduino.h>
#include "I2Cdev.h"
#include "robo_gyro.hpp"
float jairo = 0;
bool searchflag = true;
bool backgoalfrontflag = false;

#include "robo_moter.hpp"

//===========LED_DEFINE========================================================
#include <Adafruit_NeoPixel.h>
const int DIN_PIN = 2; // D1
const int LED_COUNT = 12; // LEDの数
Adafruit_NeoPixel pixels(LED_COUNT, DIN_PIN, NEO_GRB + NEO_KHZ800); //LEDについての事前設定
int LED=0; //角度をLED位置に変換後の値
//=============================================================================

//==========SOUND_DEFINE=======================================================
#define BEAT 200 // 音の長さを指定
#define PINNO 13 // 圧電スピーカを接続したピン番号
//=============================================================================

//==========LINE_DEFINE========================================================
int lineVal[4] = {0, 0, 0, 0};
int lineOutVal[4] = {84, 54, 61, 40};//black
//int lineOutVal[4] = {47, 60, 34, 48};//white
#include "robo_line.hpp"
//=============================================================================

//==========ULTRASONIC=========================================================
int ultrasonicVal[3] = {0, 0, 0};//前,右,左
//=============================================================================

//==========WIRELESS_DEFINE====================================================
//無線
int commuMode[2] = {0,0};
#include "robo_wireless.hpp"
#include <LittleFS.h>
#include <SoftwareSerial.h>
//=============================================================================

//==========DRIBBLER_DEFINE====================================================
int radPower = 200; //ショットの時の回転速度
int shootTime = 190;//ショットの時の回転時間
int shootCool = 500;//ショットの後止まってる時間
unsigned long sendTime = 0;
const unsigned long timeout = 500;
//=============================================================================

//==========CAMERA_DEFINE======================================================
#include "robo_camera.hpp"
int homeTateBorder = 6400;
int homeYokoBorder = 80;
//=============================================================================

//==========POSITION_DEFINE====================================================
String posi = "None";
unsigned long last_sended_time = 0;
int a_position[2] = {0, 0};
int g_position[2] = {0, 0};
int goPosition[2] = {58, 3};
int positionRange = 10;
//=============================================================================

//==========KICKER_DEFINE======================================================
bool kickMode = false;
//=============================================================================

//==========DRIBLE_DEFINE======================================================
bool drMode = false;
bool drCatch = false;
//=============================================================================

#include "robo_serial.hpp"

SoftwareSerial mySerial1(19, 20); // RX, TX
SoftwareSerial mySerial2(17, 16); // RX, TX
#include <math.h>
//==========MAIN===============================================================
void setup() {
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
  pinMode(22, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);
  pinMode(24, INPUT_PULLUP);
  pinMode(25, OUTPUT);
  pinMode(19, OUTPUT);
  digitalWrite(19, false);
  pinMode(20, OUTPUT);
  digitalWrite(20, false);
  // put your setup code here, to run once:
  pixels.begin();
  mpu_setup();
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-moutmax, moutmax); // モーターのPWM制御範囲に合わせて設定
  pid.SetSampleTime(10); // PID制御の更新周期（ミリ秒）
  LittleFS.begin();
  posi = "attacker";
  //posi = "keeper";
  
  if(digitalRead(23) == HIGH){
    atack_goal_color = "blue";
    pixels.setPixelColor(5, pixels.Color(0,0,2));
    pixels.setPixelColor(6, pixels.Color(0,0,2));
    pixels.show();
  } else {
    atack_goal_color = "yellow";
    pixels.setPixelColor(5, pixels.Color(2,2,0));
    pixels.setPixelColor(6, pixels.Color(2,2,0));
    pixels.show();
  }
  
  mySerial.println("Se/PChange");
  tone(PINNO,370,BEAT) ; // ファ#
  delay(BEAT) ;
  tone(PINNO,293,BEAT) ; // レ
  delay(BEAT) ;
  //tone(PINNO,220,BEAT) ; // ラ
  //delay(BEAT) ;
  //tone(PINNO,293,BEAT) ; // レ
  //delay(BEAT) ;
  //tone(PINNO,330,BEAT) ; // ミ
  //delay(BEAT) ;
  //tone(PINNO,440,BEAT+100) ; // ラ
  //delay(400+BEAT) ;
  loodLineSet();
  pixels.setPixelColor(5, pixels.Color(0,0,0));
  pixels.setPixelColor(6, pixels.Color(0,0,0));
  pixels.show();
  delay(5000);
}

void loop() {
  /*
  if (Serial.available() > 0){
    tone(PINNO,293,BEAT) ; // レ
    wirelessText = "";
    wirelessText = Serial.readStringUntil('\n');
    commandRead(wirelessText);
  }*/
  
  if (mySerial.available() > 0){
    commandRead(mySerial.readString());
  }
  
  if(Serial2.available() > 0){
    commandRead2(Serial2.readStringUntil('\n'));
  }
  
  jairo = getJairo();
  float pitch = jairo;

  //gbrads = 0;
  //gyrads = 0;
  
  if (abs(pitch) > 130){
    strongTurn = true;
  } else if (abs(pitch) < 10 && strongTurn == true){
    strongTurn = false;
  }
  if(searchflag){
    if(abs(gbrads) < 130 && atack_goal_color == "blue"){
      if(abs(pitch) < 70){
        pitch = gbrads*(-1);
      } else if((pitch >= 0 && gbrads >= 0) || (pitch < 0 && gbrads < 0)){
        if(pitch >= 0){
          pitch = pitch-70;
        } else {
          pitch = pitch+70;
        }
      }
    } else if(abs(gyrads) < 130 && atack_goal_color == "yellow"){
      if(abs(pitch) < 70){
        pitch = gyrads*(-1);
      } else if((pitch >= 0 && gyrads >= 0) || (pitch < 0 && gyrads < 0)){
        if(pitch >= 0){
          pitch = pitch-70;
        } else {
          pitch = pitch+70;
        }
      }
    }
  }
  
  
  

  /*
  
  if(abs(gyrads) < 130 && atack_goal_color == "yellow"){
    pitch = gyrads*(-1);
    Serial.print("gyrads: ");
    Serial.println(gyrads);
  }
  */
  
  
  
  
  //float pitch = gbrads*(-1);
  //if (abs(gbrads) < 181){
    //Input = gbrads;
  //} else {
  //}
  intoutput = pidCalculate(pitch);
  if (strongTurn){
    if(pitch < 0){
      MoterSerial(170, 170, -170, -170);
    } else if(pitch < 0){
      MoterSerial(-170, -170, 170, 170);
    }
  }
  //Serial.println(String(pitch) + " " + String(gbrads) + " " + String(jairo));
  //Serial.println(String(intoutput));
  /*
  MoterSerial(intoutput, intoutput, -intoutput, -intoutput);
  */
  //Serial.println(intoutput);
  if(posi == "attacker"){
    pixels.setPixelColor(5, pixels.Color(0,0,2));
    pixels.setPixelColor(6, pixels.Color(0,0,2));
    pixels.show();
  } else {
    pixels.setPixelColor(5, pixels.Color(0,2,0));
    pixels.setPixelColor(6, pixels.Color(0,2,0));
    pixels.show();
  }
  //if(kickMode && !checkComm()){
  if(kickMode){
    digitalWrite(6, HIGH);
    delay(200);
    digitalWrite(6, LOW);
    delay(100);
    kickMode = false;
  }
  
}

void setup1(){
  Serial.begin(38400);
  //Serial.setTimeout(10);
  Serial1.begin(9600);
  Serial2.setTX(8);
  Serial2.setRX(9);
  Serial2.begin(115200);
  mySerial.begin(38400);
  mySerial.setTimeout(10);
  mySerial1.begin(115200);
  mySerial1.setTimeout(50);
  mySerial2.begin(115200);
  mySerial2.setTimeout(50);
}

void loop1() {
  //Serial.println("loop1");
  putPower = 200;
  ballRD = cameraCheck();
  //kaihi_check();

  lineVal[0] = analogRead(26);
  lineVal[1] = analogRead(27);
  lineVal[2] = analogRead(28);
  lineVal[3] = analogRead(29);

  if(posi == "attacker"){
    goRad = a_roboGoRad(ballRD[0], ballRD[1]);
  } else {
    goRad = k_roboGoRad(ballRD[0], ballRD[1]);
  }

  if (!strongTurn){
    if (true){
    //if (lineCheck(lineVal)){
      //delay(3000);
      //MoterSerialPR(0,0);
      //MoterSerialPR(255,0); 
      if (abs(goRad) < 181){
        MoterSerialPR(putPower,goRad);
      } else {
        MoterSerialPR(0,0);
      }
      
      //delay(3000);
      /*
      if (abs(rads) < 181){
        MoterSerialPR(putPower,);
      } else {
        MoterSerialPR(0,0);
      }
      */
      
      //blshoot(0);
      //delay(5000);
    //}
    }
  }
  //serial表示------------------------------------------------------
  //serial_surrounding();//カメラLiDAR
  //serial_goal();//ゴール方向確認
  serial_ultrasonic();//超音波センサ
  //serial_kaihi();//カメラLiDARから導き出される位置
  //serial_RDGO();//現在のボール情報(r,d)を表示
  //serial_line();//ラインセンサ
  //serial_camera_ball();
  //----------------------------------------------------------------
}
//=============================================================================

//==========LED================================================================
void setColor(int *ledArray, int *color) {
  for(int i = 0; i < LED_COUNT; i++) {
    if(ledArray[i] == 1) {
      pixels.setPixelColor(i, pixels.Color(color[0], color[1], color[2]));
    }
    else {
      pixels.setPixelColor(i, 0); // LEDをオフにする
    }
  }
}
//=============================================================================

//==========MONITOR============================================================
void monitorShow(String text){
  if(text == "dmnjyro"){
    mySerial.print("dmnjyro|"+String(ypr[0]));
    Serial.println("OK");
  }
}

void positionSet(){
  tone(PINNO,370,BEAT) ; // ファ#
  delay(BEAT) ;
  tone(PINNO,370,BEAT) ; // ファ#
  delay(BEAT) ;
  tone(PINNO,370,BEAT) ; // ファ#
  delay(BEAT) ;
  const unsigned long startTime = millis();
  int interval = 100;
  int count = 0;
  int arraySize = 50;
  int posiX[arraySize];
  int posiY[arraySize];
  while (count < arraySize) {
    posiX[count] = kaihi_x_k;
    posiY[count] = kaihi_y_k;
    count++;
    delay(interval);
  }
  for (int i = 0; i < arraySize - 1; i++) {
    for (int j = 0; j < arraySize - 1; j++) {
      if (posiX[j] > posiX[j + 1]) {
        int temp = posiX[j];
        posiX[j] = posiX[j + 1];
        posiX[j + 1] = temp;
      }
    }
  }
  for (int i = 0; i < arraySize - 1; i++) {
    for (int j = 0; j < arraySize - 1; j++) {
      if (posiY[j] > posiY[j + 1]) {
        int temp = posiY[j];
        posiY[j] = posiY[j + 1];
        posiY[j + 1] = temp;
      }
    }
  }
  mySerial.print("poistion Value: X: ");
  mySerial.print(String(posiX[33]));
  mySerial.print(" Y: ");
  mySerial.println(String(posiY[33]));
}
//=============================================================================

//==========DRIBBLER===========================================================
void blshoot(int a){
  if (a == 0){
    MoterSerial(radPower, radPower, -radPower, -radPower);
    delay(shootTime);
    MoterSerial(0, 0, 0, 0);
    delay(shootCool);
    MoterSerial(-radPower, -radPower, radPower, radPower);
    delay(shootTime);
    MoterSerial(0, 0, 0, 0);
  } else {
    MoterSerial(-radPower, -radPower, radPower, radPower);
    delay(shootTime);
    MoterSerial(0, 0, 0, 0);
    delay(shootCool);
    MoterSerial(radPower, radPower, -radPower, -radPower);
    delay(shootTime);
    MoterSerial(0, 0, 0, 0);
  }
}
//=============================================================================
//PID制御の調整
