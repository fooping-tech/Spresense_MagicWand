//--------------------------------------
//  9軸センサによる姿勢推定
//--------------------------------------

//センサからの読み出し値
float accx=0; //加速度x
float accy=0; //加速度y
float accz=0; //加速度z
float magx=0; //地磁気x
float magy=0; //地磁気y
float magz=0; //地磁気z
float avelx=0; //ジャイロセンサx
float avely=0; //ジャイロセンサy
float avelz=0; //ジャイロセンサz

//ジャイロセンサ補正値(Initで算出)
float avelx0=0;
float avely0=0;
float avelz0=0;

//姿勢値オフセット変数
float head0=0;
float roll0=0;
float pitch0=0;

//ジャイロセンサドリフト補正  
void GyroInit(){
  Serial.print("Gyro Init");
  for(int i=0 ;i<100;i++){
    //加算
    avelx0 +=avelx;
    avely0 +=avely;
    avelz0 +=avelz;
    Serial.print(".");
  }
  
    avelx0 = avelx0/100;
    avely0 = avely0/100;
    avelz0 = avelz0/100;
    //初期値を表示
    Serial.println("Finished");
    Serial.print("GyroX:"+ String(avelx0));
    Serial.print(",GyroY:"+ String(avely0));
    Serial.println(",GyroZ:"+ String(avelz0));
}

//タイマー割り込み関数
unsigned int TimerInterruptFunction() {
  avelx = IMU_ReadAveGyroX();
  avely = IMU_ReadAveGyroY();
  avelz = IMU_ReadAveGyroZ();
  accx = IMU_ReadAccX();
  accy = IMU_ReadAccY();
  accz = IMU_ReadAccZ();
  magx = 0;//mx;//屋内では地磁気センサ誤動作するため0
  magy = 0;//my;//屋内では地磁気センサ誤動作するため0
  magz = 0;//mz;//屋内では地磁気センサ誤動作するため0
  filter->update(avelx,avely,avelz,accx,accy,accz,magx,magy,magz);
  // Serial.print("heading:");
  // Serial.print(filter->getYaw());
  // Serial.print(",roll:");
  // Serial.println(filter->getRoll());
  roll = filter->getRoll()-roll0;
  pitch = filter->getPitch()-pitch0;
  heading = filter->getYaw()-head0;
  return INTERVAL;
}

//AK09918
void AK09918_Init() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    err = ak09918.initialize();
    ak09918.switchMode(AK09918_POWER_DOWN);
    ak09918.switchMode(AK09918_CONTINUOUS_100HZ);

    err = ak09918.isDataReady();
    while (err != AK09918_ERR_OK) {
        Serial.println("Waiting Sensor");
        delay(100);
        err = ak09918.isDataReady();
    }
    err = ak09918.isDataReady();
    // err = AK09918_ERR_OK;
    if (err == AK09918_ERR_OK) {
        err = ak09918.isDataSkip();
        if (err == AK09918_ERR_DOR) {
            //Serial.println(ak09918.strError(err));
        }
        err = ak09918.getData(&x, &y, &z);
        if (err == AK09918_ERR_OK) {
            Serial.print("X:");
            Serial.print(x);
            Serial.print(",");
            Serial.print("Y:");
            Serial.print(y);
            Serial.print(",");
            Serial.print("Z:");
            Serial.print(z);
            Serial.println("");
            mx0 = x;
            my0 = y;
            mz0 = z;
        } else {
            Serial.println(ak09918.strError(err));
        }
    } else {
        Serial.println(ak09918.strError(err));
    }
}
void AK09918_main() {
  err = ak09918.isDataReady();
  // err = AK09918_ERR_OK;
  if (err == AK09918_ERR_OK) {
      err = ak09918.isDataSkip();
      if (err == AK09918_ERR_DOR) {
          //Serial.println(ak09918.strError(err));
      }
      err = ak09918.getData(&x, &y, &z);
      if (err == AK09918_ERR_OK) {

          mx = x - mx0;
          my = y - my0;
          mz = z - mz0;
      } else {
          Serial.println(ak09918.strError(err));
      }
  } else {
      Serial.println(ak09918.strError(err));
  }
  //delay(100);
}
//--------------------------------------
//  TFT
//--------------------------------------
void TFT_Init(){
  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, HIGH);
  tft.begin();
  tft.setRotation(2);
  tft.setSwapBytes(true);
}
//Canvasリセット
void ResetCanvas(){
    MadgWick_Init();     //フィルタおよび変数を初期化
    //canvas4->Reset();    //Canvasを初期化
    delete canvas4;
    canvas4 = new CANVAS(&tft,240,240,0,0);    //杖軌跡
}

//MadgWickフィルタ初期化
void MadgWick_Init(){
  filter = new Madgwick();
  filter->begin(1000000/INTERVAL);
  roll0=filter->getRoll();
  pitch0=filter->getPitch();
  head0=filter->getYaw();
  avelx0=0;
  avely0=0;
  avelz0=0;
  avelx=0;
  avely=0;
  avelz=0;
  accx=0;
  accy=0;
  accz=0;
}


//--------------------------------------
//  Switch
//--------------------------------------

void SW_main(){
  //SW状態を取得
  bool swflag = switch1->check_change();
  //スイッチの値に変化があった場合
  if(swflag){
    ResetCanvas();
  }
}

//--------------------------------------
//  Record Functions
//--------------------------------------
int number=0;
int label=0;
//Arudinoシリアル通信の受信値を処理
void Serial_main(){
  if (Serial.available() > 0) { // シリアルバッファにデータがあるか確認
    char receivedChar = Serial.read(); // 1バイト読み取り

    if (receivedChar == 's') { // もし受信したデータが's'なら
        SaveCSV();
        Serial.println("Done!");  
    }
    if (receivedChar == 'l') { // もし受信したデータが'l'ならラベル+1
        label++;
        if(label>5)label=0;
        Serial.print("label=");
        Serial.println(label);  
    }
    if (receivedChar == 'p') { // もし受信したデータが'p'なら
        canvas4->PrintSerial28();
    }
    if (receivedChar == 'r') { // もし受信したデータが'r'なら
        ResetCanvas();
    }


  }
}
//CSVにセーブする
void SaveCSV(){
  char filename[16];
  sprintf(filename, "%01d%03d.csv", label, number);
  SD.remove(filename);
  File myFile = SD.open(filename, FILE_WRITE);

  for (int i=0;i<28;i++) {
    for (int j=0;j<28;j++) {
      myFile.print(canvas4->output[i + 28 * j]);
      //Serial.print(canvas4->output[i + 28 * j]);
      if(j!=27)myFile.print(",");
      //Serial.print(",");
    }
    myFile.println("");
    //Serial.println("");
  }
  Serial.printf("%01d%03d.csv", label, number);
  myFile.close();
  number++;
}