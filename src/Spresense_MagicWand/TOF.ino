/*
//--------------------------------------
//  ToFセンサ値処理
//--------------------------------------
int TOF_dis = 0;
int TOF_range3d[8][4] = {{0}};

int ledr = 0;
int ledg = 0;
int ledb = 0;

static  int oldseq = -1;

const float alpha = 0.3;  //LED LPF
int r_filtered = 0;
int g_filtered = 0;
int b_filtered = 0;
void TOF_SetLED(int r,int g ,int b){

// フィルタ処理
  r_filtered = alpha * r + (1 - alpha) * r_filtered;
  g_filtered = alpha * g + (1 - alpha) * g_filtered;
  b_filtered = alpha * b + (1 - alpha) * b_filtered;
  
  ledr = r_filtered;
  ledg = g_filtered;
  ledb = b_filtered;
}
void TOF_SetLedNow(int r,int g ,int b){
  ledr = r;
  ledg = g;
  ledb = b;
}
int TOF_ReadDistance(){
  return TOF_dis;
}

int* TOF_ReadDistance3d(){
  return &TOF_range3d[0][0];
}
// SPIから1バイトデータを受信する関数
static  inline  int TOF_spigetb()
{
  return SPI5.transfer(0) & 0xff;
}

// SPIから2バイトデータを受信する関数
static  int TOF_spigeth()
{
  int i;
  
  i = TOF_spigetb() << 8;
  i |= TOF_spigetb();
  return i;
}

// SPIから4バイトデータを受信する関数
static  int TOF_spigetw()
{
  int i;
  
  i = TOF_spigetb() << 24;
  i |= TOF_spigetb() << 16;
  i |= TOF_spigetb() << 8;
  i |= TOF_spigetb();
  return i;
}

// 指定されたバイト数だけSPIからデータを受信し、読み捨てる関数
static  void  TOF_spiskip(int len)
{
  while (len > 0) {
    TOF_spigetb();
    len--;
  }
}
// モジュールにコマンドを送信する関数
static  void  TOF_spicommandinner(int cmd, int val)
{
  SPI5.transfer(0xeb);
  SPI5.transfer(cmd);
  SPI5.transfer(1);
  SPI5.transfer(val);
  SPI5.transfer(0xed);
}

// モジュールにコマンドを送信し、読み捨てる関数
static  void  TOF_spicommand(int cmd, int val)
{
  TOF_spicommandinner(cmd, val);
  TOF_spiskip(256 - 5);
}

void TOF_ledIndicateMain(int dis){
  //Spresense LED表示処理
    char  c;
    switch (dis / 200) {
      case    0:
        c = 0;
        break;
      case    1:
        c = 1;
        break;
      case    2:
        c = 3;
        break;
      case    3:
        c = 7;
        break;
      case    4:
        c = 0xf;
        break;
      case    5:
        c = 0xe;
        break;
      case    6:
        c = 0xc;
        break;
      case    7:
      default:
        c = 8;
        break;
    }
    digitalWrite(LED0, (c & 1)? HIGH : LOW);
    digitalWrite(LED1, (c & 2)? HIGH : LOW);
    digitalWrite(LED2, (c & 4)? HIGH : LOW);
    digitalWrite(LED3, (c & 8)? HIGH : LOW);
}
void TOF_Init(){
  Serial.println("TOF_Init");
  TOF_spiskip(256);
  delay(500);
  TOF_spicommand(0, 0xff);    // sync mode.
  TOF_spicommand(0x12,0);    // 6m mode.
  delay(500);
  TOF_spiskip(256);
  TOF_spiskip(TOF_spigetb());     // sync.
  TOF_spicommand(0, 0);       // normal mode.
  delay(500);

//  TOF_spicommand(0x10, 0x40); // 64frames/s
//  TOF_spicommand(0x10, 0x80); // 128frames/s
  TOF_spicommand(0x10, 0); // 256frames/s
//  TOF_spicommand(0x11, 0x40); // 320frames/s
//  TOF_spicommand(0x11, 1); // 5frames/s
  delay(500);
  TOF_spiskip(256);
  Serial.println("TOF_Init_OK");
}

void TOF_main(){

    static  int count = 0;

    //static  int ledr = 0;
    //static  int ledg = 0;
    //static  int ledb = 0;
  
    //SPI通信仕様に従う
    int magic0 = TOF_spigetb();//1byte
    int seq0 = TOF_spigetb();   //1byte
    TOF_spiskip(2);   //2byte
    int range = TOF_spigetw();//1Dの距離 //4byte(32bit)
    int light = TOF_spigeth();//1Dの光量 //2byte(16bit)

    for(int i=0;i<8;i++){
      //Serial.print("["+String(i)+"]");
      for(int j=0;j<4;j++){
        int range3d = TOF_spigetw();  //4byte*32(32bit*32)
        TOF_range3d[i][j] = range3d / (0x400000 / 1000);
        //Serial.print(TOF_range3d[i][j]);
        //Serial.print(" ");
      }
      //Serial.println("");
    }
    
//    TOF_spiskip(256 - 9); // 256 - 1 -8byte
    TOF_spicommandinner(0xc0, ledr);
    TOF_spicommandinner(0xc1, ledg);
    TOF_spicommandinner(0xc2, ledb);
    TOF_spiskip(256 - 9 - 2 - 4 * 32 - 5 * 3);
    
    int seq1 = TOF_spigetb();
    
    //Serial.println("magic=" + String(magic0) + ",seq0=" + String(seq0) + ",range=" + String(range) + ",seq1=" + String(seq1));
    //Check Value
    bool ResetFlag=false;
    if (magic0 != 0xe9)//データ同期失敗時
      ResetFlag=true;
    if (seq0 != seq1)//データブロック無効時
      ;//ResetFlag=true;
    if (oldseq < 0)
      ;
    else if (((oldseq - seq0) & 0x80) == 0)
      ;//ResetFlag=true;
    oldseq = seq0;

    if(ResetFlag){
      TOF_Init();
    }

    int dis = range / (0x400000 / 1000);
    TOF_dis = dis;
    if ((0)) {
      Serial.print(count++);
      Serial.print("(");
      Serial.print(seq0, HEX);
      Serial.print("): ");
      
      Serial.print(dis);
      Serial.print("mm\n");
    }
    TOF_ledIndicateMain(dis);
    
  
}
*/