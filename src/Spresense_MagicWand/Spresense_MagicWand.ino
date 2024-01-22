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

#define RECORD_MODE 0//0:推論モード,1:学習モード
//SD
#include <SDHCI.h>
SDClass  SD;
#include <File.h>

//IMU
#include "Arduino_BMI270_BMM150.h"

//TFT
#define TFT_LED 8       //LED接続端子
#define LGFX_AUTODETECT // 自動認識
#include <LovyanGFX.hpp>
#include "LGFX_SPRESENSE.hpp"
#include <LGFX_AUTODETECT.hpp>        // クラス"LGFX"を準備します
static LGFX_SPRESENSE_SPI_ILI9341 tft;// LGFXのインスタンスを作成

//ToF
#include <SPI.h>

//Canvas
#include "CANVAS.h"
CANVAS *canvas1;
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
int mx,my,mz;
int mx0,my0,mz0;

//SW
#include "SW.h"
SW *switch1;

//MadgWick
#include <MadgwickAHRS.h>
#define INTERVAL 100000 //us
float roll=0;
float pitch=0;
float heading=0;
Madgwick *filter;

//DNN
#include <DNNRT.h>
DNNRT dnnrt;
#define DNN_DATA_WIDTH 28
#define DNN_DATA_HEIGHT 28
DNNVariable input(DNN_DATA_WIDTH*DNN_DATA_HEIGHT);
static String const labels[4]= {"EIGHT", "CIRCLE", "MINUS", "NON"};
int command =3;

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

int TaskSpan;       //タスク実行間隔
uint32_t startTime; //開始時間
uint32_t cycleTime; //サイクルタイム
uint32_t deltaTime; //
uint32_t spentTime; //経過時間
bool _InitCondition=false;  //初期化状態
bool _DeinitCondition=false;  //終了時状態


//Audio
#include <Audio.h>
AudioClass *theAudio;
bool ErrEnd = false;

static void audio_attention_cb(const ErrorAttentionParam *atprm)
{
  puts("Attention!");
  
  if (atprm->error_code >= AS_ATTENTION_CODE_WARNING)
    {
      ErrEnd = true;
   }
}

File myFile;
bool audio=false;
void PlaySound(int i){
    /* Open file placed on SD card */
  if(i==1)myFile = SD.open("audio1.mp3");
  else if(i==2)myFile = SD.open("audio2.mp3");
  else if(i==3)myFile = SD.open("audio3.mp3");
  
  if (!myFile)
    {
      printf("File open error\n");
      exit(1);
    }
  Serial.println(myFile.name());

  /* Send first frames to be decoded */
  err_t err = theAudio->writeFrames(AudioClass::Player0, myFile);

  if (err == AUDIOLIB_ECODE_FILEEND)
  {
    theAudio->stopPlayer(AudioClass::Player0);
  }
  if ((err != AUDIOLIB_ECODE_OK) && (err != AUDIOLIB_ECODE_FILEEND))
    {
      printf("File Read Error! =%d\n",err);
      myFile.close();
      exit(1);
    }


  /* Main volume set -1020 to 120 */
  theAudio->setVolume(120);

  theAudio->startPlayer(AudioClass::Player0);
  audio = true;

  myFile.close();
  //usleep(40000);
}
int CheckCommand(){
  float *dnnbuf = input.data();
  int count=0;
  for (int i=0;i<DNN_DATA_WIDTH;i++) {
    for (int j=0;j<DNN_DATA_HEIGHT;j++) {
      dnnbuf[count] = canvas4->output[i + 28 * j];
      count++;
    }
  }

  dnnrt.inputVariable(input,0);
  dnnrt.forward();

  DNNVariable output = dnnrt.outputVariable(0);
  int index = output.maxIndex();
  return index;
}



void setup() {
  Serial.begin(9600);
  //TFT
  TFT_Init();
  
  switch1 = new SW(PIN_D21,INPUT_PULLUP);
  //タイマ割り込み
  attachTimerInterrupt(TimerInterruptFunction,INTERVAL);

  //CANVAS(tft,w,h,x,y)
  canvas1 = new CANVAS(&tft,80,40,20,240);      //ToFText
  canvas2 = new CANVAS(&tft,20,40,0,240);       //ToFドット
  //canvas3 = new CANVAS(&tft,140,40,100,240);    //グラフ
  canvas4 = new CANVAS(&tft,240,240,0,0);    //杖軌跡
  canvas5 = new CANVAS(&tft,240,280,0,280);   //テキスト
  canvas6 = new CANVAS(&tft,140,40,100,240);      //Text


  //SD
  SD.begin();
  //USB MSC
  if (SD.beginUsbMsc()) {
    Serial.println("USB MSC Failure!");
  } else {
    Serial.println("*** USB MSC Prepared! ***");
    Serial.println("Insert SD and Connect Extension Board USB to PC.");
  }
  //DNN
  File nnbfile = SD.open("model.nnb");
  int ret = dnnrt.begin(nnbfile);
  if (ret < 0) {
    Serial.println("dnnrt.begin failed" + String(ret));
  }
  
  //ToF
  SPI5.begin();
  SPI5.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  TOF_Init();

  //IMU
  IMU_Init();

  //AK09918
  AK09918_Init();

  //ジャイロセンサ
  GyroInit();

  //MadgWick
  MadgWick_Init();

  //Audio
  theAudio = AudioClass::getInstance();
  theAudio->begin(audio_attention_cb);
  puts("initialization Audio Library");
  /* Set clock mode to normal ハイレゾかノーマルを選択*/
  theAudio->setRenderingClockMode(AS_CLKMODE_NORMAL);
  /*select LINE OUT */
  theAudio->setPlayerMode(AS_SETPLAYER_OUTPUTDEVICE_SPHP, AS_SP_DRV_MODE_LINEOUT);
  /*init player DSPファイルはmicroSD カードの場合は "/mnt/sd0/BIN" を、SPI-Flash の場合は "/mnt/spif/BIN" を指定します。*/
  err_t err = theAudio->initPlayer(AudioClass::Player0, AS_CODECTYPE_MP3, "/mnt/sd0/BIN", AS_SAMPLINGRATE_AUTO, AS_CHANNEL_STEREO);
  /* Verify player initialize */
  if (err != AUDIOLIB_ECODE_OK)
    {
      printf("Player0 initialize error\n");
      exit(1);
    }
    
}


void CANVAS_main(){
  //ToFセンサテキスト描画
  int* range = TOF_ReadDistance3d();
  canvas1->StringDrawL(String(TOF_ReadDistance())+"mm",TFT_WHITE);
  //ToFセンサドット描画
  canvas2->DotDraw(range);
  //姿勢グラフ描画
  //float value1[] = {roll,pitch,heading};
  //canvas3->GraphDraw(value1);
  //杖軌跡描画
  canvas4->WandDraw28(heading,roll);
  //IMU,地磁気センサ値テキスト描画
  canvas5->StringDraw("aX="+ String(IMU_ReadAccX())+",aY="+String(IMU_ReadAccY())+",aZ="+String(IMU_ReadAccZ())+"\ngX="+ String(IMU_ReadGyroX())+",gY="+String(IMU_ReadGyroY())+",gZ="+String(IMU_ReadGyroZ())+"\nmX="+ String(mx)+",mY="+String(my)+",mZ="+String(mz), TFT_YELLOW);
  //
  canvas6->StringDrawL(String(labels[command])+"",TFT_WHITE);
}




void mainloop(MODE m){
  //前回実行時からモードが変わってたら終了処理
  if(beforeMode != m)DeinitActive();
  beforeMode = m;
   //前回実行時からの経過時間を計算
  deltaTime = millis() - cycleTime;
  //経過時間を計算
  spentTime = millis() - startTime;

  if(deltaTime >= TaskSpan){
//    Serial.println("mainloop");
    if(!_InitCondition){//未初期化時実行
//      Serial.println("Init");
      InitFunction(m);
    }else if(_DeinitCondition){//修了処理時実行
//      Serial.println("Deinit");
      DeinitFunction();
    }else{
//      Serial.println("main()");
        //モードごとの処理
    switch (m) {
      case MODE1:
        //TOF_SetLED(255,255,255);      
        break;

      case MODE2:
        //TOF_SetLED(255,0,0);
        break;

      case MODE3:
        //TOF_SetLED(0,255,0);
        currentMode = MODE4;
        
        break;

      case MODE4:
        TOF_SetLED(0,0,0);
        break;
      }
      
    }
    //cycleTimeリセット
    cycleTime = millis();
  }
}

void DeinitActive(){//モード終了時に呼ぶ
  startTime = millis();
  //モードごとの処理
  _DeinitCondition=true;
}
void DeinitFunction(){//最後に呼ばれる
  
  _InitCondition=false;  //初期化状態
  _DeinitCondition=false;  //終了時状態
}
void InitFunction(MODE m){//初回呼ぶ
  //モードごとの処理
  switch (m) {
  case MODE1:
    TOF_SetLED(255,255,255);
    PlaySound(1);
    break;

  case MODE2:
    TOF_SetLED(255,0,0);
    PlaySound(2);
    break;

  case MODE3:
    TOF_SetLED(0,255,0);
    PlaySound(3);
    break;

  case MODE4:
    TOF_SetLED(0,0,0);
    break;
}
  startTime = millis();
  _InitCondition=true;  //初回フラグon
}

void Audio_main(){
  int err = theAudio->writeFrames(AudioClass::Player0, myFile);
  /*  Tell when player file ends */
  if (err == AUDIOLIB_ECODE_FILEEND)
    {
      //printf("Main player File End!\n");
    }

  /* Show error code from player and stop */
  if (err)
    {
      //printf("Main player error code: %d\n", err);
      if(audio == true){
        theAudio->stopPlayer(AudioClass::Player0);
        myFile.close();
        audio = false;
      }
    }
}


void loop() {
  IMU_main();         //IMUセンサ値更新
  TOF_main();         //TOFセンサ値更新
  AK09918_main();     //地磁気センサ更新
  CANVAS_main();      //描画更新
  if(RECORD_MODE == 0)command = CheckCommand(); //DNN
  SW_main();         //ボタンチェック(押下時Reset処理)
  Audio_main();      //オーディオ
  Serial_main();      //Arduinoシリアル操作

  //モード起動時処理
  if(RECORD_MODE == 0){
    if(IMU_CheckAccActive()&& TOF_ReadDistance()>50){
      if(command==0){
        currentMode = MODE1;
        ResetCanvas();
      }
      if(command==1){
        currentMode = MODE2;
        ResetCanvas();
      }
      if(command==2){
        currentMode = MODE3;
        ResetCanvas();
      }
    }
  }
  mainloop(currentMode);

  //所定の加速度より早い場合キャンバスを消す
  if(IMU_CalcAccVec(IMU_ReadAccX(),IMU_ReadAccY(),IMU_ReadAccZ())>1.5){
    //Serial.println(IMU_CalcAccVec);
    //currentMode = MODE4;
    ResetCanvas();
  }


}

