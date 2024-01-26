/*
    The MIT License (MIT)

    Copyright (c) 2024 FoopingTech.
    X(Twitter)    : @FoopingTech
    Create Time: 2024-01


    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

//サブコア誤書き込み防止
#ifdef SUBCORE
#error "Core selection is wrong!!"
#endif


#define RECORD_MODE 0  //0:推論モード,1:学習モード

#include <File.h>
#include <Flash.h>
//IMU
#include "Arduino_BMI270_BMM150.h"

//TFT
#define TFT_LED 8        //LED接続端子
#define LGFX_AUTODETECT  // 自動認識
#include <LovyanGFX.hpp>
#include "LGFX_SPRESENSE.hpp"
#include <LGFX_AUTODETECT.hpp>          // クラス"LGFX"を準備します
static LGFX_SPRESENSE_SPI_ILI9341 tft;  // LGFXのインスタンスを作成

//ToF
//#include <SPI.h>

//Canvas
#include "CANVAS.h"
//CANVAS *canvas1;
CANVAS *canvas2;
CANVAS *canvas3;
CANVAS *canvas4;
CANVAS *canvas5;
CANVAS *canvas6;

//AK09918
#include "AK09918.h"
#include <Wire.h>
AK09918_err_type_t err;
int32_t x, y, z;
AK09918 ak09918;
int mx, my, mz;
int mx0, my0, mz0;

//SW
#include "SW.h"
SW *switch1;

//MadgWick
#include <MadgwickAHRS.h>
#define INTERVAL 100000  //us
float roll = 0;
float pitch = 0;
float heading = 0;
Madgwick *filter;

//DNN
#include <DNNRT.h>
DNNRT dnnrt;
#define DNN_DATA_WIDTH 28
#define DNN_DATA_HEIGHT 28
DNNVariable input(DNN_DATA_WIDTH *DNN_DATA_HEIGHT);
static String const labels[4] = { "EIGHT", "CIRCLE", "MINUS", "NON" };
int command = 3;

//INTERFACE to M5ATOM
#define OUTPUT1_PIN 27
#define OUTPUT2_PIN 20

//mode
// モードの列挙型
enum MODE {
  MODE1,
  MODE2,
  MODE3,
  MODE4
};
MODE currentMode = MODE4;
MODE beforeMode = MODE4;

int TaskSpan;                   //タスク実行間隔
uint32_t startTime;             //開始時間
uint32_t cycleTime;             //サイクルタイム
uint32_t deltaTime;             //
uint32_t spentTime;             //経過時間
bool _InitCondition = false;    //初期化状態
bool _DeinitCondition = false;  //終了時状態


int CheckCommand() {
  float *dnnbuf = input.data();
  int count = 0;
  for (int i = 0; i < DNN_DATA_WIDTH; i++) {
    for (int j = 0; j < DNN_DATA_HEIGHT; j++) {
      dnnbuf[count] = canvas4->output[i + 28 * j];
      count++;
    }
  }

  dnnrt.inputVariable(input, 0);
  dnnrt.forward();

  DNNVariable output = dnnrt.outputVariable(0);
  int index = output.maxIndex();
  return index;
}

void InterfaceOutput(MODE m) {
  switch (m) {
    case MODE1:
      digitalWrite(OUTPUT1_PIN, LOW);
      digitalWrite(OUTPUT2_PIN, LOW);
      break;

    case MODE2:
      digitalWrite(OUTPUT1_PIN, LOW);
      digitalWrite(OUTPUT2_PIN, HIGH);
      break;

    case MODE3:
      digitalWrite(OUTPUT1_PIN, HIGH);
      digitalWrite(OUTPUT2_PIN, LOW);
      break;

    case MODE4:
      digitalWrite(OUTPUT1_PIN, HIGH);
      digitalWrite(OUTPUT2_PIN, HIGH);
      break;
  }
}


void setup() {
  Serial.begin(115200);

  //INTERFACE
  pinMode(OUTPUT1_PIN, OUTPUT);
  pinMode(OUTPUT2_PIN, OUTPUT);
  digitalWrite(OUTPUT1_PIN, HIGH);
  digitalWrite(OUTPUT2_PIN, HIGH);

  //TFT
  TFT_Init();

  switch1 = new SW(PIN_D21, INPUT_PULLUP);
  //タイマ割り込み
  attachTimerInterrupt(TimerInterruptFunction, INTERVAL);
  
  canvas4 = new CANVAS(&tft,240,240,0,0);    //杖軌跡

  //IMU
  IMU_Init();

  //AK09918
  AK09918_Init();

  //ジャイロセンサ
  GyroInit();

  //MadgWick
  MadgWick_Init();


  //SD->Flash
  Flash.begin();

  //DNN
  File nnbfile = Flash.open("model.nnb");
  int ret = dnnrt.begin(nnbfile);
  if (ret < 0) {
    Serial.println("dnnrt.begin failed" + String(ret));
  }

}

void CANVAS_main() {
  //杖軌跡描画
  canvas4->WandDraw28(heading, roll);
}

void mainloop(MODE m) {
  //前回実行時からモードが変わってたら終了処理
  if (beforeMode != m) DeinitActive();
  beforeMode = m;
  //前回実行時からの経過時間を計算
  deltaTime = millis() - cycleTime;
  //経過時間を計算
  spentTime = millis() - startTime;

  if (deltaTime >= TaskSpan) {
    //    Serial.println("mainloop");
    if (!_InitCondition) {  //未初期化時実行
                            //      Serial.println("Init");
      InitFunction(m);
    } else if (_DeinitCondition) {  //修了処理時実行
                                    //      Serial.println("Deinit");
      DeinitFunction();
    } else {
      //      Serial.println("main()");
      //モードごとの処理
      InterfaceOutput(m);
      Serial.println(m);
      switch (m) {
        case MODE1:
          break;

        case MODE2:
          //TOF_SetLED(255,0,0);
          break;

        case MODE3:
          //TOF_SetLED(0,255,0);
          currentMode = MODE4;

          break;

        case MODE4:
          //TOF_SetLED(0,0,0);
          break;
      }
    }
    //cycleTimeリセット
    cycleTime = millis();
  }
}

void DeinitActive() {  //モード終了時に呼ぶ
  startTime = millis();
  //モードごとの処理
  _DeinitCondition = true;
}
void DeinitFunction() {  //最後に呼ばれる

  _InitCondition = false;    //初期化状態
  _DeinitCondition = false;  //終了時状態
}
void InitFunction(MODE m) {  //初回呼ぶ
  //モードごとの処理
  switch (m) {
    case MODE1:
      //TOF_SetLED(255,255,255);
      //PlaySound(1);
      ledOn(LED0);
      ledOff(LED1);
      ledOff(LED2);
      ledOff(LED3);
      break;

    case MODE2:
      //TOF_SetLED(255,0,0);
      //PlaySound(2);
      ledOff(LED0);
      ledOn(LED1);
      ledOff(LED2);
      ledOff(LED3);
      break;

    case MODE3:
      //TOF_SetLED(0,255,0);
      //PlaySound(3);
      ledOff(LED0);
      ledOff(LED1);
      ledOn(LED2);
      ledOff(LED3);
      break;

    case MODE4:
      //TOF_SetLED(0,0,0);
      ledOff(LED0);
      ledOff(LED1);
      ledOff(LED2);
      ledOn(LED3);

      break;
  }
  startTime = millis();
  _InitCondition = true;  //初回フラグon
}

void loop() {

  IMU_main();  //IMUセンサ値更新
  //TOF_main();         //TOFセンサ値更新
  //AK09918_main();     //地磁気センサ更新
  CANVAS_main();                                   //描画更新
  if (RECORD_MODE == 0) command = CheckCommand();  //DNN
  SW_main();                                       //ボタンチェック(押下時Reset処理)
  //Audio_main();                                    //オーディオ

  Serial_main();  //Arduinoシリアル操作

  //モード起動時処理
  if (RECORD_MODE == 0) {
    if (IMU_CheckAccActive()) {
      if (command == 0) {
        currentMode = MODE1;
        ResetCanvas();
      }
      if (command == 1) {
        currentMode = MODE2;
        ResetCanvas();
      }
      if (command == 2) {
        currentMode = MODE3;
        ResetCanvas();
      }
    }
  }

  //所定の加速度より早い場合キャンバスを消す
  if (IMU_CalcAccVec(IMU_ReadAccX(), IMU_ReadAccY(), IMU_ReadAccZ()) > 1.5) {
    //Serial.println(IMU_CalcAccVec);
    currentMode = MODE4;
    //ResetCanvas();
  }

  mainloop(currentMode);
}
