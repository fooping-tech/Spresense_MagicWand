//--------------------------------------
//  9軸センサ値処理
//--------------------------------------

float IMU_x, IMU_y, IMU_z;
float IMU_mx,IMU_my,IMU_mz;
float IMU_gx,IMU_gy,IMU_gz;
float IMU_gxAve,IMU_gyAve,IMU_gzAve;
#define NUM 3 //移動平均回数
float IMU_gyroX[NUM];
float IMU_gyroY[NUM];
float IMU_gyroZ[NUM];
int IMU_index = 0;               // readings 配列のインデックス
float IMU_totalX = 0;               // 読み取り値の合計
float IMU_averageX = 0;             // 移動平均値
float IMU_totalY = 0;               // 読み取り値の合計
float IMU_averageY = 0;             // 移動平均値
float IMU_totalZ = 0;               // 読み取り値の合計
float IMU_averageZ = 0;             // 移動平均値
float IMU_offsetX = 0;
float IMU_offsetY = 0;
float IMU_offsetZ = 0;

//加速度センサ補正値
float acc_gainX = 1.01010101010101;
float acc_gainY = 1.01010101010101;
float acc_gainZ = 1;
float acc_offsetX = 0;
float acc_offsetY = -0.01;
float acc_offsetZ = 0;


void IMU_Init(){
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" uT");
  Serial.println();
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
  
  //配列初期化
  for(int i=0;i<NUM;i++){
    IMU_gyroX[i]=0;
    IMU_gyroY[i]=0;
    IMU_gyroZ[i]=0;
  }
  Serial.print("Gyro init.Put Device");
  for(int i=0;i<NUM;i++){
    IMU_main();
    Serial.print(".");
    delay(10);
  }

  IMU_Reset();
  Serial.println("Finish");
}
//ジャイロセンサの平均値を求める
void IMU_CalcAverage(float valueX,float valueY,float valueZ){
  
  // 過去の読み取り値の合計から古い値を引く
  IMU_totalX = IMU_totalX - IMU_gyroX[IMU_index];
  IMU_totalY = IMU_totalY - IMU_gyroY[IMU_index];
  IMU_totalZ = IMU_totalZ - IMU_gyroZ[IMU_index];

  // 最新のセンサの値を readings 配列に追加
  IMU_gyroX[IMU_index] = valueX;
  IMU_gyroY[IMU_index] = valueY;
  IMU_gyroZ[IMU_index] = valueZ;

  // 過去の読み取り値の合計に新しい値を加える
  IMU_totalX = IMU_totalX + IMU_gyroX[IMU_index];
  IMU_totalY = IMU_totalY + IMU_gyroY[IMU_index];
  IMU_totalZ = IMU_totalZ + IMU_gyroZ[IMU_index];

  // 次の readings 配列のインデックスを計算
  IMU_index = (IMU_index + 1) % NUM;

  // 移動平均を計算
  IMU_averageX = IMU_totalX / NUM;
  IMU_averageY = IMU_totalY / NUM;
  IMU_averageZ = IMU_totalZ / NUM;
}

float IMU_CalcAccVec(float x,float y,float z){
    return abs(sqrt(x*x+y*y+z*z) - 1);
}

float CorrectAccX(float a){
  return acc_gainX*(a+acc_offsetX);
}
float CorrectAccY(float a){
  return acc_gainY*(a+acc_offsetY);
}
float CorrectAccZ(float a){
  return acc_gainZ*(a+acc_offsetZ);
}

float IMU_ReadAccX(){
  return CorrectAccX(IMU_x);
}
float IMU_ReadAccY(){
  return CorrectAccY(IMU_y);
}
float IMU_ReadAccZ(){
  return CorrectAccZ(IMU_z);
}

float IMU_ReadMagX(){
  return IMU_mx;
}
float IMU_ReadMagY(){
  return IMU_my;
}
float IMU_ReadMagZ(){
  return IMU_mz;
}
float IMU_ReadGyroX(){
  return IMU_gx;
}
float IMU_ReadGyroY(){
  return IMU_gy;
}
float IMU_ReadGyroZ(){
  return IMU_gz;
}
float IMU_ReadAveGyroX(){
  return IMU_averageX - IMU_offsetX;
}
float IMU_ReadAveGyroY(){
  return IMU_averageY - IMU_offsetY;
}
float IMU_ReadAveGyroZ(){
  return IMU_averageZ - IMU_offsetZ;
}
void IMU_Reset(){
  IMU_offsetX = IMU_averageX;
  IMU_offsetY = IMU_averageY;
  IMU_offsetZ = IMU_averageZ;
}

//IMU Reset Check
float IMU_startTime = 0;
bool countflag = false;
bool gflag =false;
const float AccValue = 0.04; //閾値
const int SpentTime = 150;

bool IMU_CheckAccActive(){
  return gflag;
}
void IMU_main(){

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(IMU_x, IMU_y, IMU_z);
  }
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(IMU_mx, IMU_my, IMU_mz);
    Serial.println("MagneticField_OK");
  }
    if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(IMU_gx, IMU_gy, IMU_gz);

  }

  //ジャイロセンサ値移動平均処理
  IMU_CalcAverage(IMU_gx, IMU_gy, IMU_gz);
  
  //静止判定
  if(IMU_CalcAccVec(IMU_ReadAccX(),IMU_ReadAccY(),IMU_ReadAccZ()) < AccValue){
    if(!countflag){
      IMU_startTime = millis(); // カウンター開始前であればStartTimeを記録
      countflag = true;
    }
    if(millis() - IMU_startTime > SpentTime){
      gflag=true;
    }
    else{
      gflag=false;
    }
  }else{
    countflag = false; //閾値の以上なら
    gflag = false;     // IMUの値が閾値以上ならgflagをfalseに設定
  }

}