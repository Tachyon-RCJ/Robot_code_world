#include <Arduino.h>
#include "robo_mode.hpp"

int k_backhome(){
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

int k_roboGoRad(int r, int d){
  if(abs(r) > 181){
      return 185;
  }

  int reRad = 0; // 修正後の角度を初期化
  if(d < 0){
    return 185;
  } else {
    // 各センサーグループの白線検出状態を判定
    bool sensor27 = (lineVal[0] > lineOutVal[0]);     // 赤色 (前方)
    bool sensor26 = (lineVal[1] > lineOutVal[1]);    // 青色 (左側面)
    bool sensor28 = (lineVal[2] > lineOutVal[2]);  // 黄色 (右側面)
    bool sensor29 = (lineVal[3] > lineOutVal[3]);   // 緑色 (後方)
    
    // ボールの角度 r に基づいて進む方向を決定
    if (r >= -180 && r < -15) { // ボールが左側にある場合（左へ進む）
        if (sensor26 && sensor27 && !sensor28) {
            // 左側と前方に白線がある（理想的な左カーブトレース）
            reRad = -90.0; // 左側面をキープするように進む
        } else if (sensor26 && sensor27 && sensor28) {
            // 前方と左右に白線がある（直進に近いが左に寄りたい）
            reRad = -45.0; // やや左へ修正
        } else if (sensor26 && !sensor27) {
            // 左側面には白線があるが、前方に見えない（カーブが急すぎるか、見失いそう）
            reRad = -135.0; // より強く左旋回
        } else if (!sensor26 && sensor27) {
            // 前方には白線があるが、左側面に見えない（右に逸脱気味）
            reRad = -45.0; // 左へ戻る
        } else if (sensor28) {
            // 右側に白線が検出された（大きく左に逸脱、または白線をまたぎそう）
            reRad = -180.0; // 最大の左旋回で白線を探す
        } else if (sensor29) {
            // 後方に白線が検出された（完全に白線をまたいだ、緊急事態）
            reRad = -180.0; // 急な左旋回で白線に戻る
        } else {
            // 全てのセンサーが白線を検出しない（左への移動中）
            reRad = -90.0; // 左方向へ進みながら白線を探す
        }
    } else if (r > 15 && r <= 180) { // ボールが右側にある場合（右へ進む）
        if (sensor28 && sensor27 && !sensor26) {
            // 右側と前方に白線がある（理想的な右カーブトレース）
            reRad = 90.0; // 右側面をキープするように進む
        } else if (sensor28 && sensor27 && sensor26) {
            // 前方と左右に白線がある（直進に近いが右に寄りたい）
            reRad = 45.0; // やや右へ修正
        } else if (sensor28 && !sensor27) {
            // 右側面には白線があるが、前方に見えない（カーブが急すぎるか、見失いそう）
            reRad = 135.0; // より強く右旋回
        } else if (!sensor28 && sensor27) {
            // 前方には白線があるが、右側面に見えない（左に逸脱気味）
            reRad = 45.0; // 右へ戻る
        } else if (sensor26) {
            // 左側に白線が検出された（大きく右に逸脱、または白線をまたぎそう）
            reRad = 180.0; // 最大の右旋回で白線を探す
        } else if (sensor29) {
            // 後方に白線が検出された（完全に白線をまたいだ、緊急事態）
            reRad = 180.0; // 急な右旋回で白線に戻る
        } else {
            // 全てのセンサーが白線を検出しない（右への移動中）
            reRad = 90.0; // 右方向へ進みながら白線を探す
        }
    } else { // ボールの角度が0度付近、または想定外の値（中央付近でのライン維持）
        reRad = 185;
    }
  }
  return reRad;
}