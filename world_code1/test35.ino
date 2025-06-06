//test21:新しい動作システムのテスト（面積が最大のカメラを選択）+ボールなかったら中心へ+RP2040で進行方向計算
#include "I2Cdev.h"
#include "robo_gyro.hpp"
float jairo = 0;

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
#include "robo_line.hpp"
//=============================================================================

//==========WIRELESS_DEFINE====================================================
//無線
#include "robo_wireless.hpp"
#include <LittleFS.h>
#include <SoftwareSerial.h>
//=============================================================================

//==========DRIBBLER_DEFINE====================================================
int radPower = 200; //ショットの時の回転速度
int shootTime = 190;
int shootCool = 500;
int drBack = 100;
int drCatch = 50;
bool drMode = false;
bool drCatchMode = false;
bool shootSubSend = false;
bool shootSend = false;
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


SoftwareSerial mySerial1(11, 12); // RX, TX
SoftwareSerial mySerial2(17, 16); // RX, TX
#include <Arduino.h>
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
  tone(PINNO,220,BEAT) ; // ラ
  delay(BEAT) ;
  tone(PINNO,293,BEAT) ; // レ
  delay(BEAT) ;
  tone(PINNO,330,BEAT) ; // ミ
  delay(BEAT) ;
  tone(PINNO,440,BEAT+100) ; // ラ
  delay(400+BEAT) ;
  loodLineSet();
  pixels.setPixelColor(5, pixels.Color(0,0,0));
  pixels.setPixelColor(6, pixels.Color(0,0,0));
  pixels.show();
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
  // put your main code here, to run repeatedly:
  jairo = getJairo();
  float pitch = jairo;
  //Serial.println(abs(int(ypr[0] * 180/M_PI)));
  //int hyouzi[]={0,0,0,0,0,0,0,0,0,0,0,0,0};
  //LED = round(int(ypr[0] * 180/M_PI)/6);
  /*
  LED = round(int(rads));
  if(LED < 0){
    LED = 6 + abs(LED);
  } else {
    LED = 6 - LED;
  }
  if((0 <= LED )&&(LED <= 12)){
    hyouzi[LED] = 50;
  }
  if((0 <= LED )&&(LED <= 12)){
    hyouzi[LED-1] = 20;
  }
  if((0 <= LED )&&(LED <= 12)){
    hyouzi[LED+1] = 20;
  }
  pixels.setPixelColor(0, pixels.Color(0,0,hyouzi[0]));
  pixels.setPixelColor(1, pixels.Color(0,0,hyouzi[1]));
  pixels.setPixelColor(2, pixels.Color(0,0,hyouzi[2]));
  pixels.setPixelColor(3, pixels.Color(0,0,hyouzi[3]));
  pixels.setPixelColor(4, pixels.Color(0,0,hyouzi[4]));
  pixels.setPixelColor(5, pixels.Color(0,0,hyouzi[5]));
  pixels.setPixelColor(6, pixels.Color(0,0,hyouzi[7]));
  pixels.setPixelColor(7, pixels.Color(0,0,hyouzi[8]));
  pixels.setPixelColor(8, pixels.Color(0,0,hyouzi[9]));
  pixels.setPixelColor(9, pixels.Color(0,0,hyouzi[10]));
  pixels.setPixelColor(10, pixels.Color(0,0,hyouzi[11]));
  pixels.setPixelColor(11, pixels.Color(0,0,hyouzi[12]));
  pixels.show();
  */
  
  if (abs(pitch) > 130){
    strongTurn = true;
  } else if (abs(pitch) < 10 && strongTurn == true){
    strongTurn = false;
  }
  /*
  if(abs(gbrads) < 130 && atack_goal_color == "blue"){
    pitch = gbrads*(-1);
    Serial.print("gbrads: ");
    Serial.println(gbrads);
  }
  
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
    MoterSerial(170, 170, -170, -170);
  }
  Serial.println(String(pitch));
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
  Serial1.begin(115200);
  Serial2.setTX(8);
  Serial2.setRX(9);
  Serial2.begin(115200);
  mySerial.begin(38400);
  mySerial.setTimeout(10);
  mySerial1.begin(115200);
  mySerial1.setTimeout(10);
  mySerial2.begin(115200);
  mySerial2.setTimeout(10);
}

void loop1() {
  putPower = powermx;
  ballRD = cameraCheck();
  kaihi_check();
  //生存確認
  /*
  Serial.print(surrounding[0]);
  Serial.print(',');
  Serial.print(surrounding[1]);
  Serial.print(',');
  Serial.print(surrounding[2]);
  Serial.print(',');
  Serial.print(surrounding[3]);
  Serial.print(',');
  Serial.print(surrounding[4]);
  Serial.print(',');
  Serial.print(surrounding[5]);
  Serial.print(',');
  Serial.print(surrounding[6]);
  Serial.print(',');
  Serial.print(surrounding[7]);
  Serial.print(',');
  Serial.print(surrounding[8]);
  Serial.print(',');
  Serial.print(surrounding[9]);
  Serial.print(',');
  Serial.print(surrounding[10]);
  Serial.print(',');
  Serial.print(surrounding[11]);
  Serial.print(',');
  Serial.print(surrounding[12]);
  Serial.print(',');
  Serial.print(surrounding[13]);
  Serial.print(',');
  Serial.print(surrounding[14]);
  Serial.print(',');
  Serial.println(surrounding[15]);
  Serial.print(kaihi_x_k);
  Serial.print(',');
  Serial.print(kaihi_y_k);
  Serial.print(',');
  */
  //Serial.print(kaihi_muki_k);
  //Serial.print(',');
  /*
  Serial.println(kaihi_speed_k);
  */
  /*
  Serial.print(" r:");
  Serial.print(String(ballRD[0]));
  Serial.print(" d:");
  Serial.print(String(ballRD[1]));
  */
  
  goRad = roboGoRad(ballRD[0], ballRD[1]);
  /*
  Serial.print(" go:");
  Serial.print(String(goRad));
  Serial.print(" goal:");
  Serial.print(String(gbrads));
  Serial.print(" gColor:");
  Serial.println(atack_goal_color);
  */
  

  if (drMode){
    if (drCatchMode || shootSend){
      putPower = drCatch;
    } else {
      putPower = drBack;
    }
  }
  //mySerial.print(rads);
  //mySerial.print(",");
  //mySerial.print(radsbrStr);
  //mySerial.print(",");
  //mySerial.println(radsblStr);
  lineVal[0] = analogRead(26);
  lineVal[1] = analogRead(27);
  lineVal[2] = analogRead(28);
  lineVal[3] = analogRead(29);

  if (!strongTurn){
    if (true){
    //if (lineCheck()){
      //delay(3000);
      //MoterSerialPR(0,0);
      //MoterSerialPR(200,0);
      
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
  /*
  if (senValMode) {
    mySerial.print("Sy/SVMo/");
    mySerial.print("L");
    mySerial.print("/");
    mySerial.print(String(analogRead(26)));
    mySerial.print(",");
    mySerial.print(String(analogRead(27)));
    mySerial.print(",");
    mySerial.print(String(analogRead(28)));
    mySerial.print(",");
    mySerial.print(String(analogRead(29)));
    //mySerial.print("/");
    //mySerial.print(M1+","+M2+","+M3+","+M4);
    //mySerial.print("/");
    //mySerial.print(String(abs(int(ypr[0] * 180/M_PI))));
    mySerial.println("/");
  }
  */
  /*
  Serial.print(lineVal[0]);
  Serial.print(", ");
  Serial.print(lineVal[1]);
  Serial.print(", ");
  Serial.print(lineVal[2]);
  Serial.print(", ");
  Serial.println(lineVal[3]);
  */
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
