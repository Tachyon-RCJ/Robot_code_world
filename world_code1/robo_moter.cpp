#include "robo_moter.hpp"

double Setpoint, Input, Output;
double Kp = 2, Ki = 0, Kd = 0; //ゲイン設定
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); //PIDの事前設定
String mchange[16]={"0","1","2","3","4","5","6","7","8","9","a","b","c","d","e","f"}; //１６進数変換用
float intoutput; //モーター出力値を整数に変換
int moutmax = 40;
bool strongTurn = false;//スタート時に回転する

String M1, M2, M3, M4;
float Moterhelp[4] = {1, 1, 1, 1};
int powermx = 160;
int putPower = 160;

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
    float movehelp[4] = {1,1,1,1};
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
    int mpower = p*10.0/max(
        max(abs(movehelp[0]*Moterhelp[0]*10*sin((PI/4)-radians(r))),abs(movehelp[1]*Moterhelp[1]*10*cos((PI/4)-radians(r)))),
        max(abs(movehelp[2]*Moterhelp[2]*10*cos((PI/4)-radians(r))),abs(movehelp[3]*Moterhelp[3]*10*sin((PI/4)-radians(r))))
    );
    MoterSerial(
        int(movehelp[0]*(Moterhelp[0]*mpower*sin((PI/4)-radians(r)))),
        int(movehelp[1]*(Moterhelp[1]*mpower*cos((PI/4)-radians(r)))),
        int(movehelp[2]*(Moterhelp[2]*mpower*cos((PI/4)-radians(r)))),
        int(movehelp[3]*(Moterhelp[3]*mpower*sin((PI/4)-radians(r))))
    );
}