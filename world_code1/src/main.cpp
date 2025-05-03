//test21:新しい動作システムのテスト（面積が最大のカメラを選択）+ボールなかったら中心へ+RP2040で進行方向計算
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
float jairo = 0;

//===========プロトタイプ宣言========================================================
void MoterSerial(int m1, int m2, int m3, int m4);
void MoterSerialPR(int p, int r);
void commandRead(String comtxt);
void hexStringToByteArray(const char* hexString, byte* byteArray, int byteArrayLength);
void setColor(int *ledArray, int *color);
void monitorShow(String text);
int backhome();
int roboGoRad(int r, int d);
void positionSet();
bool lineCheck();
void kaihi_check();
void lineSet(int i);
void loodLineSet();
int* cameraCheck();
void blshoot(int a);
//=============================================================================

//===========LED_DEFINE========================================================
#include <Adafruit_NeoPixel.h>
const int DIN_PIN = 2; // D1
const int LED_COUNT = 12; // LEDの数
Adafruit_NeoPixel pixels(LED_COUNT, DIN_PIN, NEO_GRB + NEO_KHZ800); //LEDについての事前設定
int LED=0; //角度をLED位置に変換後の値
//=============================================================================

//==========PID_DEFINE=========================================================
#include <PID_v1.h>
double Setpoint, Input, Output;
double Kp = 2, Ki = 0, Kd = 0; //ゲイン設定
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); //PIDの事前設定
String mchange[16]={"0","1","2","3","4","5","6","7","8","9","a","b","c","d","e","f"}; //１６進数変換用
float intoutput; //モーター出力値を整数に変換
int moutmax = 40;
bool strongTurn = false;
//=============================================================================

//==========MOTER_DEFINE=======================================================
String M1,M2,M3,M4; //モーターマイコンへシリアル通信用値保存
float Moterhelp[4] = {1, 1, 1, 1}; //モーター個体差補正値定義{bl.fl,br,fr}
int powermx = 220; //モーターパワーの上限値を設定
int putPower = 220;//実際に出す力
int theta = 0; //ロボットの行きたい方向
bool goCup = false;
//============================================================================= 

//==========SOUND_DEFINE=======================================================
#define BEAT 200 // 音の長さを指定
#define PINNO 13 // 圧電スピーカを接続したピン番号
//=============================================================================

//==========LINE_DEFINE========================================================
int preRads;
int lineFound[2]={0,0};
int lineVal[4] = {0, 0, 0, 0};
int lineOutVal[4] = {35,35,35,35};
int outTime = 200;
int stopTime = 100;
float kaihi_x = 0;
float kaihi_y = 0;
float kaihi_speed = 0;
float kaihi_muki = 0;
float kaihi_x_k = 0;
float kaihi_y_k = 0;
float kaihi_speed_k = 0;
float kaihi_muki_k = 0;
//=============================================================================

//==========WIRELESS_DEFINE====================================================
//無線
#include <LittleFS_Mbed_RP2040.h>
//#include <AESLib.h>
#include <PicoSoftwareSerial.h>
SoftwareSerial mySerial(14, 15); // RX, TX
//AESLib aesLib;
const char* hexKey;
const char* hexIV;
const char* hexCiphertext;
uint16_t decrypted_length;
byte aes_key[16];
byte aes_iv[16];
byte ciphertext[64]; // 暗号文のバイト配列
byte cleartext[64]; // 復号化されたデータを格納するバッファ
char textRead;
String wirelessText = "";
bool senValMode = true;//センサの値をPCへ送るかどうか
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

//==========LINESET_DEFINE=====================================================
unsigned long startTime;
const unsigned long settingTime = 7000;
int lineBorder[4] = {0, 0, 0, 0};
//=============================================================================

//==========CAMERA_DEFINE======================================================
int fCameraSize = 0;
int rCameraSize = 0;
int lCameraSize = 0;
int rWorldSize = 0;
int lWorldSize = 0;
int homeTateBorder = 6400;
int homeYokoBorder = 80;
int fdistance = 0;
int rdistance = 0;
int ldistance = 0;
int returnValue[2] = {185, -1};
int surrounding[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
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

int rads = 185; 
int gbrads = 185;
int gyrads = 185;
int head;
int goRad = 185;
int radsbr;
int radsbl;
int* ballRD;
String atack_goal_color = "";

String cameraLBuf = "";
String cameraRBuf = "";
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
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(27);
  mpu.setYGyroOffset(-40);
  mpu.setZGyroOffset(85);
  mpu.setZAccelOffset(1105); // 1688 factory default for my test chip
  if (devStatus == 0) {
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      mpu.setDMPEnabled(true);
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
  }
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-moutmax, moutmax); // モーターのPWM制御範囲に合わせて設定
  pid.SetSampleTime(10); // PID制御の更新周期（ミリ秒）
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
  mpu.dmpGetCurrentFIFOPacket(fifoBuffer); 
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  if (ypr[0] * 180/M_PI >= 0){
    jairo=int(((ypr[0] * 180/M_PI)*0.7)*3.9);
  } else {
    jairo=(int(((180+(180-abs((ypr[0] * 180/M_PI))))*0.7)*3.9));
  }
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
  float pitch = ypr[0] * 180/M_PI;
  
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
  Input = pitch;
  //}
  Setpoint = 0.0; 
  pid.Compute();
  intoutput = (int)Output;
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

UART Serial2(8, 9);

void setup1(){
  Serial.begin(38400);
  //Serial.setTimeout(10);
  Serial1.begin(115200);
  //Serial2.setTX(8);
  //Serial2.setRX(9);
  //Serial2.setPins(8, 9);
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
      //MoterSerialPR(putPower,0);
      
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

//==========MOTER==============================================================
void MoterSerial(int m1, int m2, int m3, int m4){
  //Serial.println(String(m1)+","+String(m2)+","+String(m3)+","+String(m4));
  if (m1 < 0){
    M1="b"+String(mchange[int(abs(m1)/16)])+String(mchange[abs(m1)%16]);
  } else if (m1 >= 0){
    M1="f"+String(mchange[int(abs(m1)/16)])+String(mchange[abs(m1)%16]);
  }
  if (m2 < 0){
    M2="b"+String(mchange[int(abs(m2)/16)])+String(mchange[abs(m2)%16]);
  } else if (m2 >= 0){
    M2="f"+String(mchange[int(abs(m2)/16)])+String(mchange[abs(m2)%16]);
  }
  if (m3 < 0){
    M3="b"+String(mchange[int(abs(m3)/16)])+String(mchange[abs(m3)%16]);
  } else if (m3 >= 0){
    M3="f"+String(mchange[int(abs(m3)/16)])+String(mchange[abs(m3)%16]);
  }
  if (m4 < 0){
    M4="b"+String(mchange[int(abs(m4)/16)])+String(mchange[abs(m4)%16]);
  } else if (m4 >= 0){
    M4="f"+String(mchange[int(abs(m4)/16)])+String(mchange[abs(m4)%16]);
  }
  if (digitalRead(22) == HIGH){
    Serial2.print(M1 + M2 + M3 + M4 + "s");
  } else {
    Serial2.print("00000000s");
  }
  //Serial.println(M1 + M2 + M3 + M4);
}

void MoterSerialPR(int p, int r){
  float movehelp[4] = {1,1,1,1}; //移動中のロボット角度補正{bl.fl,br,fr}
  if (sin((PI/4)-radians(r)) >= 0){
    movehelp[3] = 1-(intoutput/100);
    movehelp[0] = 1+(intoutput/100);
  } else {
    movehelp[0] = 1-(intoutput/100);
    movehelp[3] = 1+(intoutput/100);
  }
  if (cos((PI/4)-radians(r)) >= 0){
    movehelp[2] = 1-(intoutput/100);
    movehelp[1] = 1+(intoutput/100);
  } else {
    movehelp[1] = 1-(intoutput/100);
    movehelp[2] = 1+(intoutput/100);
  }
  int mpower = p*10.0/max(max(abs(movehelp[0]*Moterhelp[0]*10*sin((PI/4)-radians(r))),abs(movehelp[1]*Moterhelp[1]*10*cos((PI/4)-radians(r)))),max(abs(movehelp[2]*Moterhelp[2]*10*cos((PI/4)-radians(r))),abs(movehelp[3]*Moterhelp[3]*10*sin((PI/4)-radians(r)))));
  MoterSerial(int(movehelp[0]*(Moterhelp[0]*mpower*sin((PI/4)-radians(r)))),int(movehelp[1]*(Moterhelp[1]*mpower*cos((PI/4)-radians(r)))),int(movehelp[2]*(Moterhelp[2]*mpower*cos((PI/4)-radians(r)))),int(movehelp[3]*(Moterhelp[3]*mpower*sin((PI/4)-radians(r)))));
}
//=============================================================================

//==========WIRELESS===========================================================
//R:アクチュエーターやLED、スピーカーに起動を促す処理
//S:変数値の上書きやぷろぐらむの実行を促す処理
void commandRead(String comtxt){
  if(comtxt.substring(0,10) == "Se/PChange"){
    if(posi == "attacker"){
      posi = "keeper";
    } else {
      posi = "attacker";
    }
    last_sended_time = millis();
  } else if(comtxt.substring(0,14) == "Se/SendMessage"){
    if(posi == "keeper"){
      mySerial.println("Se/SendOK");
      last_sended_time = millis();
    }
  } else if(comtxt.substring(0,7) == "WDS/SSA"){
    mySerial.print("Sy/SVMo/");
    mySerial.print("LMJYBRGD");
    mySerial.print("/");
    mySerial.print(String(analogRead(26)));
    mySerial.print(",");
    mySerial.print(String(analogRead(27)));
    mySerial.print(",");
    mySerial.print(String(analogRead(28)));
    mySerial.print(",");
    mySerial.print(String(analogRead(29)));
    mySerial.print("/");
    mySerial.print(M1+","+M2+","+M3+","+M4);
    mySerial.print("/");
    mySerial.print(String(int(ypr[0] * 180/M_PI)));
    mySerial.print("/");
    mySerial.print(String(gyrads));
    mySerial.print("/");
    mySerial.print(String(gbrads));
    mySerial.print("/");
    if(ballRD[1] != -1){
      mySerial.print(String(ballRD[0]));
    } else {
      mySerial.print(String(185));
    }
    mySerial.print("/");
    mySerial.print(String(goRad));
    mySerial.print("/");
    mySerial.print(String(ballRD[1]));
    mySerial.println("/");
  } else if(comtxt.substring(0,11) == "WDS/LSetfrr"){
    lineSet(2);
    loodLineSet();
  } else if(comtxt.substring(0,11) == "WDS/LSetbrb"){
    lineSet(3);
    loodLineSet();
  } else if(comtxt.substring(0,11) == "WDS/LSetbll"){
    lineSet(0);
    loodLineSet();
  } else if(comtxt.substring(0,11) == "WDS/LSetflf"){
    lineSet(1);
    loodLineSet();
  } else if(comtxt.substring(0,9) == "WDS/LBVal"){
    mySerial.print("WDS/LBVals/");
    mySerial.print(String(lineOutVal[0]));
    mySerial.print(",");
    mySerial.print(String(lineOutVal[1]));
    mySerial.print(",");
    mySerial.print(String(lineOutVal[2]));
    mySerial.print(",");
    mySerial.print(String(lineOutVal[3]));
  } else if(comtxt.substring(0,10) == "WDS/LiDVal"){
    mySerial.print("WDS/LiDVals/");
    mySerial.print(surrounding[0]);
    mySerial.print(',');
    mySerial.print(surrounding[1]);
    mySerial.print(',');
    mySerial.print(surrounding[2]);
    mySerial.print(',');
    mySerial.print(surrounding[3]);
    mySerial.print(',');
    mySerial.print(surrounding[4]);
    mySerial.print(',');
    mySerial.print(surrounding[5]);
    mySerial.print(',');
    mySerial.print(surrounding[6]);
    mySerial.print(',');
    mySerial.print(surrounding[7]);
    mySerial.print(',');
    mySerial.print(surrounding[8]);
    mySerial.print(',');
    mySerial.print(surrounding[9]);
    mySerial.print(',');
    mySerial.print(surrounding[10]);
    mySerial.print(',');
    mySerial.print(surrounding[11]);
    mySerial.print(',');
    mySerial.print(surrounding[12]);
    mySerial.print(',');
    mySerial.print(surrounding[13]);
    mySerial.print(',');
    mySerial.print(surrounding[14]);
    mySerial.print(',');
    mySerial.println(surrounding[15]);
  } else if(comtxt.substring(0,8) == "WDS/MyPo"){
    mySerial.print("WDS/MyPos/");
    mySerial.print(String(kaihi_x_k));
    mySerial.print(',');
    mySerial.print(String(kaihi_y_k));
    mySerial.print(',');
    mySerial.print(String(kaihi_speed_k));
    mySerial.print(',');
    mySerial.println(String(kaihi_muki_k));
  } else if(comtxt.substring(0,9) == "WDS/PoSet"){
    positionSet();
  }
}

void hexStringToByteArray(const char* hexString, byte* byteArray, int byteArrayLength) {
  for (int i = 0; i < byteArrayLength; i++) {
    sscanf(hexString + 2 * i, "%2hhx", &byteArray[i]);
  }
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

//==========LINE===============================================================
bool lineCheck(){
  preRads = rads;
  int found = 0;
  if (lineVal[0] > lineOutVal[0]) found++;
  if (lineVal[1] > lineOutVal[1]) found++;
  if (lineVal[2] > lineOutVal[2]) found++;
  if (lineVal[3] > lineOutVal[3]) found++;
  if (found != 0){
    //tone(PINNO,220,BEAT) ; // ラ
    MoterSerialPR(powermx,preRads+180);
    delay(stopTime);
    MoterSerialPR(powermx,kaihi_muki_k);
    delay(outTime);
    return false;
  } else {
    return true;
  }
  /*
  if (found == 4){
    return true;
  } else if (found == 3){
    tone(PINNO,220,BEAT) ; // ラ
    MoterSerialPR(250,preRads+180);
    delay(outTime);
    return false;
  } else if (found == 2){
    tone(PINNO,220,BEAT) ; // ラ
    if (analogRead(27) > lineOutVal && analogRead(28) > lineOutVal){
      MoterSerialPR(250,180);
      delay(outTime);
      return false;
    } else if (analogRead(28) > lineOutVal && analogRead(29) > lineOutVal){
      MoterSerialPR(250,-90);
      delay(outTime);
      return false;
    } else if (analogRead(29) > lineOutVal && analogRead(26) > lineOutVal){
      MoterSerialPR(250,0);
      delay(outTime);
      return false;
    } else if (analogRead(26) > lineOutVal && analogRead(27) > lineOutVal){
      MoterSerialPR(250,90);
      delay(outTime);
      return false;
    } else {
      MoterSerialPR(250,preRads+180);
      delay(outTime);
      return false;
    }
  } else if (found == 1){
    if (analogRead(26) > lineOutVal){
        if (lineFound[0] == 0){
          lineFound[0] = 26;
        } else {
          lineFound[1] = 26;
        }
    } else if (analogRead(27) > lineOutVal){
      if (lineFound[0] == 0){
        lineFound[0] = 27;
      } else {
        lineFound[1] = 27;
      }
    } else if (analogRead(28) > lineOutVal){
      if (lineFound[0] == 0){
        lineFound[0] = 28;
      } else {
        lineFound[1] = 28;
      }
    } else if (analogRead(29) > lineOutVal){
      if (lineFound[0] == 0){
        lineFound[0] = 29;
      } else {
        lineFound[1] = 29;
      }
    }
    if (lineFound[1] == 0){
      return true;
    } else {
      tone(PINNO,220,BEAT) ; // ラ
      if ((lineFound[0] == 27 && lineFound[1] == 28) || (lineFound[0] == 28 && lineFound[1] == 27)){
        MoterSerialPR(250,180);
        delay(outTime);
        return false;
      } else if ((lineFound[0] == 28 && lineFound[1] == 29) || (lineFound[0] == 29 && lineFound[1] == 28)){
        MoterSerialPR(250,-90);
        delay(outTime);
        return false;
      } else if ((lineFound[0] == 29 && lineFound[1] == 26) || (lineFound[0] == 26 && lineFound[1] == 29)){
        MoterSerialPR(250,0);
        delay(outTime);
        return false;
      } else if ((lineFound[0] == 26 && lineFound[1] == 27) || (lineFound[0] == 27 && lineFound[1] == 26)){
        MoterSerialPR(250,90);
        delay(outTime);
        return false;
      } else {
        MoterSerialPR(250,preRads+180);
        delay(outTime);
        return false;
      }
      lineFound[0] = 0;
      lineFound[1] = 0;
    }
  } else {
    return true;
  }
  */
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
  tone(PINNO, 330, BEAT); // ミ
  delay(BEAT);
  tone(PINNO, 330, BEAT); // ミ
  delay(BEAT);
  const char *filePath = MBED_LITTLEFS_FILE_PREFIX "/libo.txt";
  FILE *file = fopen(filePath, "w");
  if (file) {
    String writeLineBorder = String(lineOutVal[0]) + "," + String(lineOutVal[1]) + "," + String(lineOutVal[2]) + "," + String(lineOutVal[3]);
    if (fwrite(writeLineBorder.c_str(), 1, writeLineBorder.length(), file)) {
        Serial.println("File write successful");
    } else {
        Serial.println("File write failed");
    }
    fclose(file);
  } else {
    Serial.println("Failed to open file for writing");
  }
  mySerial.print("WDS/LBorV/");
  mySerial.println(String(lineOutVal[i]));
}

void loodLineSet(){
  const char *filePath = MBED_LITTLEFS_FILE_PREFIX "/libo.txt";
  FILE *file = fopen(filePath, "r");
  if (file) {
      char buffer[128]; // ファイルから読み込むためのバッファ
      if (fgets(buffer, sizeof(buffer), file)) {
          String readLineBorder = String(buffer);
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
          sprintf(BoderSendBuffer, "LineBorder:26P:%d, 27P:%d, 28P:%d, 29P:%d", 
                  lineOutVal[0], lineOutVal[1], lineOutVal[2], lineOutVal[3]);
          mySerial.println(BoderSendBuffer);
      } else {
          Serial.println("Failed to read from file");
      }
      fclose(file);
  } else {
      Serial.println("Failed to open file for reading");
  }
}
//=============================================================================

//==========CAMERA=============================================================
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
        kickMode = true;
      }
    }
    head = Serial1.read();
    if (head == 'o') {
      surrounding[10] = Serial1.read()*2;
    }
    head = Serial1.read();
    if (head == 'p') {
      surrounding[9] = Serial1.read()*2;
    }
    head = Serial1.read();
    if (head == 'q') {
      surrounding[8] = Serial1.read()*2;
    }
    head = Serial1.read();
    if (head == 'r') {
      surrounding[7] = Serial1.read()*2;
    }
    head = Serial1.read();
    if (head == 's') {
      surrounding[6] = Serial1.read()*2;
    }
    head = Serial1.read();
    if (head == 't') {
      surrounding[5] = Serial1.read()*2;
    }
    head = Serial1.read();
    if (head == 'u') {
      surrounding[4] = Serial1.read()*2;
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
      surrounding[15] = cameraRBuf.substring(f1Index + 1, g1Index).toInt();
    }
    if (g1Index != -1 && h1Index != -1) {
      surrounding[14] = cameraRBuf.substring(g1Index + 1, h1Index).toInt();
    }
    if (h1Index != -1 && i1Index != -1) {
      surrounding[13] = cameraRBuf.substring(h1Index + 1, i1Index).toInt();
    }
    if (i1Index != -1 && j1Index != -1) {
      surrounding[12] = cameraRBuf.substring(i1Index + 1, j1Index).toInt();
    }
    if (j1Index != -1) {
      int nextIndex = j1Index + 1;
      while (nextIndex < cameraRBuf.length() && isDigit(cameraRBuf[nextIndex])) {
        nextIndex++;
      }
      surrounding[11] = cameraRBuf.substring(j1Index + 1, nextIndex).toInt();
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
      surrounding[15] = max(surrounding[15], cameraLBuf.substring(f2Index + 1, g2Index).toInt());
    }
    if (g2Index != -1 && h2Index != -1) {
      surrounding[0] = cameraLBuf.substring(g2Index + 1, h2Index).toInt();
    }
    if (h2Index != -1 && i2Index != -1) {
      surrounding[1] = cameraLBuf.substring(h2Index + 1, i2Index).toInt();
    }
    if (i2Index != -1 && j2Index != -1) {
      surrounding[2] = cameraLBuf.substring(i2Index + 1, j2Index).toInt();
    }
    if (j2Index != -1) {
      int next2Index = j2Index + 1;
      while (next2Index < cameraLBuf.length() && isDigit(cameraLBuf[next2Index])) {
        next2Index++;
      }
      surrounding[3] = cameraLBuf.substring(j2Index + 1, next2Index).toInt();
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
