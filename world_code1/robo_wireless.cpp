#include "robo_wireless.hpp"

SoftwareSerial mySerial(14, 15); // RX, TX
AESLib aesLib;
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
    mySerial.print(String(int(jairo)));
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