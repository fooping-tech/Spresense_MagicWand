#include <IRremote.hpp>
#include <M5Atom.h>
#include <esp_now.h>
#include <WiFi.h>
#include "RINGLED.h"
#define LED_DATA_PIN 21
#define IR_RECEIVE_PIN 23
#define SEND_LED_PIN 19
#define INPUT1_PIN 25
#define INPUT2_PIN 22
#define ENABLE_LED_FEEDBACK true

RINGLED led = RINGLED();

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
        led.fire2(2,100);
        //TOF_SetLED(255,255,255);      
        break;

      case MODE2:
        //TOF_SetLED(255,0,0);
        //led.fire2(1,200);
        led.flash(100);
        break;

      case MODE3:
        //TOF_SetLED(0,255,0);
        //currentMode = MODE4;
        led.flash(200);
        break;

      case MODE4:
        led.pacifica();
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
    IrSender.sendNEC(0xEF00,0x3,1);//on
    //IrSender.sendNEC(0xEF00,0x4,1);//red

    //send_data(1,id,duty,hue,brightness);
    break;

  case MODE2:
    //IrSender.sendNEC(0xEF00,0x3,1);//on
    IrSender.sendNEC(0xEF00,0x7,1);//white
    //send_data(1,id,duty,hue,brightness);
    break;

  case MODE3:
    //IrSender.sendNEC(0xEF00,0x3,1);//on
    IrSender.sendNEC(0xEF00,0x5,1);//green
    //send_data(1,id,duty,hue,brightness);
    break;

  case MODE4:
    IrSender.sendNEC(0xEF00,0x2,1);//off
    break;
}
  startTime = millis();
  _InitCondition=true;  //初回フラグon
}


void setup() {
  M5.begin(true, false, true); 
  led.setup(LED_DATA_PIN);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  IrSender.begin(SEND_LED_PIN);

  pinMode(INPUT1_PIN,INPUT_PULLUP);
  pinMode(INPUT2_PIN,INPUT_PULLUP);

  //ESP-NOW INITIAL
  ESPNOW_setup();
}

void loop() {
  M5.update();

  int input1 = digitalRead(INPUT1_PIN);
  int input2 = digitalRead(INPUT2_PIN);
  //モード起動時処理
  if(input1==0 && input2==0){
    currentMode = MODE1;
  }
  if(input1==0 && input2==1){
    currentMode = MODE2;
  }
  if(input1==1 && input2==0){
    currentMode = MODE3;
  }
  if(input1==1 && input2==1){
    currentMode = MODE4;
  }

  mainloop(currentMode);
  
//  leds[0] = CRGB::Red;                      // LED[0]を赤に設定
//  FastLED.show();                           // LEDを表示
//  FastLED.addLeds<WS2812, LED_DATA_PIN, GRB>(leds, NUM_LEDS);   // RGB LEDを初期設定
//  FastLED.setBrightness(20);                               // 明るさを設定（20以上は熱で壊れる可能性あり。）


  if (IrReceiver.decode()) {
    
    IrReceiver.printIRResultShort(&Serial);
    Serial.println(IrReceiver.decodedIRData.decodedRawData);
    if (IrReceiver.decodedIRData.decodedRawData == 0x95) {
      Serial.println("1");
    }
    IrReceiver.resume();
  }
//   if(M5.Btn.wasPressed()){
// //  IrSender.sendSony(0x1,0x15,1);
//     //IrSender.sendPulseDistanceWidthFromArray(); //Protocol=PulseDistance Repeat gap=75250us Raw-Data=0x10809522C 40 bits LSB first
//     IrSender.sendNEC(0xEF00,0x3,1);
//   }


}