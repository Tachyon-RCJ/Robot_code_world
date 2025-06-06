#ifndef ROBO_WIRELESS_H
#define ROBO_WIRELESS_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <AESLib.h>
#include <LittleFS.h>

extern SoftwareSerial mySerial;
extern AESLib aesLib;
extern const char* hexKey;
extern const char* hexIV;
extern const char* hexCiphertext;
extern uint16_t decrypted_length;
extern byte aes_key[16];
extern byte aes_iv[16];
extern byte ciphertext[64];
extern byte cleartext[64];
extern char textRead;
extern String wirelessText;
extern bool senValMode;

extern String posi;
extern unsigned long last_sended_time;
extern String M1, M2, M3, M4;
extern float jairo;
extern int gbrads;
extern int gyrads;
extern int *ballRD;
extern int goRad;
extern float kaihi_x_k;
extern float kaihi_y_k;
extern float kaihi_speed_k;
extern float kaihi_muki_k;
extern int lineOutVal[4];
extern int surrounding[16];

void loodLineSet();
void lineSet(int i);
void positionSet();

void commandRead(String comtxt);
void hexStringToByteArray(const char* hexString, byte* byteArray, int byteArrayLength);

#endif