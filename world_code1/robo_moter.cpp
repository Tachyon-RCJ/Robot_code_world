#include "robo_moter.hpp"

double Setpoint, Input, Output;
double Kp = 6, Ki = 0, Kd = 0; //ゲイン設定
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); //PIDの事前設定
String mchange[16]={"0","1","2","3","4","5","6","7","8","9","a","b","c","d","e","f"}; //１６進数変換用
float intoutput; //モーター出力値を整数に変換
int moutmax = 40;
bool strongTurn = false;//スタート時に回転する

String M1, M2, M3, M4;
float Moterhelp[4] = {1, 1, 1, 1};
int powermx = 150;
int putPower = 150;

int pidCalculate(float p){
    Input = p;
    Setpoint = 0.0; 
    pid.Compute();
    return (int)Output;
}

void MoterSerial(int m1, int m2, int m3, int m4){
    if (m1 < 0){
        M1="b"+String(mchange[int(abs(m1)/16)])+String(mchange[abs(m1)%16]);
    } else {
        M1="f"+String(mchange[int(abs(m1)/16)])+String(mchange[abs(m1)%16]);
    }
    if (m2 < 0){
        M2="b"+String(mchange[int(abs(m2)/16)])+String(mchange[abs(m2)%16]);
    } else {
        M2="f"+String(mchange[int(abs(m2)/16)])+String(mchange[abs(m2)%16]);
    }
    if (m3 < 0){
        M3="b"+String(mchange[int(abs(m3)/16)])+String(mchange[abs(m3)%16]);
    } else {
        M3="f"+String(mchange[int(abs(m3)/16)])+String(mchange[abs(m3)%16]);
    }
    if (m4 < 0){
        M4="b"+String(mchange[int(abs(m4)/16)])+String(mchange[abs(m4)%16]);
    } else {
        M4="f"+String(mchange[int(abs(m4)/16)])+String(mchange[abs(m4)%16]);
    }
    if (digitalRead(22) == HIGH){
        Serial2.print(M1 + M2 + M3 + M4 + "s");
    } else {
        Serial2.print("00000000s");
    }
}

void MoterSerialPR(int p, int r){
    // モーターごとのベクトル計算
    float base[4];
    base[0] = p * sin((PI/4) - radians(r));
    base[1] = p * cos((PI/4) - radians(r));
    base[2] = p * cos((PI/4) - radians(r));
    base[3] = p * sin((PI/4) - radians(r));

    // PID補正値を加算（例：回転方向補正）
    // intoutputはPID制御で得られる回転補正値（-moutmax～+moutmax程度）
    base[0] += intoutput;
    base[1] += intoutput;
    base[2] -= intoutput;
    base[3] -= intoutput;

    // 出力リミット
    for(int i=0; i<4; i++){
        if(base[i] > powermx) base[i] = powermx;
        if(base[i] < -powermx) base[i] = -powermx;
    }

    MoterSerial(
        (int)base[0],
        (int)base[1],
        (int)base[2],
        (int)base[3]
    );
}