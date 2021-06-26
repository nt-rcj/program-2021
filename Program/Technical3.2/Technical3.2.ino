//
// Robot main program
//           Ver 1.0 Dec.19.2019
//           By Hiroya Hashimoto
//           Ver 1.1 Jan.10.2021
//             Based on 20201221 ver.
//

//Motor control program
#include "NT_Robot202011.h" // Header file for Teensy 3.5
#include "motorDRV4.h" //  モーター制御のプログラムを読み込む

#include <Wire.h>
#include <PCF8574.h>
#include <VL6180X.h>
#include <VL53L0X.h>

PCF8574 pcf8574(I2C_PCF8574);
VL6180X ToF_front;  // create front ToF object
VL6180X ToF_back;   // create back ToF object
VL53L0X ToF_Long1;

//const int angle = 0;
const int Vlow = 13.5;  // Low limit voltage 1.1*12 = 13.2
//Mi-NHなら13.0,Li-poなら13.5(Li-poは過放電するので注意！)
const float Vstep = 0.01811;  // Voltage step 15.3V/845 = 0.01811

int blob_count, i;
static int openMV[39];
static int x_data_ball, y_data_ball, w_data_ball, h_data_ball;
static int x_data_yellowgoal, y_data_yellowgoal, w_data_yellowgoal, h_data_yellowgoal;
static int x_data_bluegoal, y_data_bluegoal, w_data_bluegoal, h_data_bluegoal;

static int8_t  gyro;
static int robot_dir, power;

static int emergency;
static int  outofbounds;   // "out of bounds" flag


static int lineflag;//line
static int line[4];

int sig, w, h, area;
int bg_w, bg_h, bg_area;
int yg_w, yg_h, yg_area;
float m , z;
float x, y;
float bg_x, bg_y;
float yg_x, yg_y;
float goal_x, goal_y;
float y_sig, b_sig, goal_sig;
int ball_front, ball_back, ball_Long1;

int level, data;
uint8_t role;
int xbee_date, xbee_x, xbee_y;
int blocks;
int ball_x, ball_y;
char buf[64];

int progress;
uint8_t finish;
int pixel;
uint32_t color;

void setup() {
  int pin;

  pinMode(StartSW, INPUT_PULLUP);

  // IOピンのモード設定

  pinMode(ledPin, OUTPUT);

  //pinMode(angle, INPUT);

  pinMode(LINE_LED, OUTPUT);
  pinMode(LINE1D, INPUT_PULLUP);
  pinMode(LINE2D, INPUT_PULLUP);
  pinMode(LINE3D, INPUT_PULLUP);
  pinMode(LINE4D, INPUT_PULLUP);
  pinMode(LINE5D, INPUT_PULLUP);
  pinMode(LINE1A, INPUT);
  pinMode(LINE2A, INPUT);
  pinMode(LINE3A, INPUT);
  pinMode(LINE4A, INPUT);
  pinMode(LINE5A, INPUT);

  pinMode(Kick1, OUTPUT);
  pinMode(Kick_Dir, OUTPUT);

  pinMode(SWR, OUTPUT);
  pinMode(SWG, OUTPUT);

  pinMode(Aux1, INPUT);
  pinMode(Aux2, INPUT);

  pinMode(GoalSW, INPUT_PULLUP);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  digitalWrite(Kick1, LOW);
  digitalWrite(Kick_Dir, LOW);
  digitalWrite(SWR, LOW);
  digitalWrite(SWG, LOW);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(INT_29, INPUT_PULLUP);          // interrupt port set

  Serial.begin(9600);   //  シリアル出力を初期化
  Serial.println("Starting...");

  //*****************************************************************************
  // Set initial value to variable

  emergency = false;
  outofbounds = false;
  power = 70;   //  set initial motor power

  //*****************************************************************************

  Wire.begin();
  // initialize ToF sensers
  // Set PCF8574pinMode to OUTPUT
  for ( i = 0; i < 8; i++)
    pcf8574.pinMode(i, OUTPUT);
  pcf8574.begin();
  for ( pin = 0; pin < 8; pin++)
    pcf8574.digitalWrite(pin, LOW);   //deactivate(reset) ToF_front,ToF_back

  /*
    CE端子をLOWにするとデバイスがリセットされアドレスが初期値に
    　戻るので注意
  */

  delay(10);
  pcf8574.digitalWrite(1 - 1, HIGH); //Activate ToF_front VL6180X
  delay(10);
  ToF_front.init();
  ToF_front.configureDefault();
  ToF_front.setAddress(TOF_1);  //好きなアドレスに設定
  ToF_front.setTimeout(100);
  delay(10);

  pcf8574.digitalWrite(2 - 1, HIGH); //Activate ToF_back VL6180X
  delay(10);
  ToF_back.init();
  ToF_back.configureDefault();
  ToF_back.setAddress(TOF_2);  //好きなアドレスに設定
  ToF_back.setTimeout(10);
  delay(10);
  
  pcf8574.digitalWrite(3 - 1, HIGH); //Activate ToF_Long1 
  delay(10);
  ToF_Long1.init();
  ToF_Long1.setAddress(TOF_3);  //好きなアドレスに設定
  ToF_Long1.setTimeout(10);
  delay(10);
  

  Serial.println("Initialize 1 ...");

  motorInit();  //  モーター制御の初期化
  dribInit();   //  ドリブラモーターの初期化

  Serial.println("Initialize 2 ...");

  delay(1000);  //  ドリブラ・キッカーの動作チェック
  dribbler1(100);
  dribbler2(100);
  delay(1000);
  dribbler1(0);
  dribbler2(0);
  delay(100);
  digitalWrite(Kick_Dir, LOW);
  delay(100);
  digitalWrite(Kicker, HIGH);
  delay(100);
  digitalWrite(Kicker, LOW);
  delay(100);
  digitalWrite(Kick_Dir, HIGH);
  delay(100);
  digitalWrite(Kicker, HIGH);
  delay(100);
  digitalWrite(Kicker, LOW);
  delay(100);

  Serial.println("Initialize 3 ...");

  Serial3.begin(19200);  // initialize serialport for openMV
  Serial2.begin(115200);   // WT901 IMU Sener
  Serial1.begin(9600); //xbee

  lineflag = false;   //reset outofbounds　flag
  for ( i = 0; i < 4; i++ )
    line[i] = false;
  robot_dir = 0;  //reset robot direction


  // LEDを初期化する
  LED_Init();
  digitalWrite(LED_R, LOW);  // LED_R消灯
  digitalWrite(LED_Y, LOW);  // LED_Y消灯
  digitalWrite(LED_G, LOW);  // LED_G消灯
  digitalWrite(LED_B, LOW);  // LED_B消灯

  digitalWrite(SWR, LOW);
  digitalWrite(SWG, HIGH);
  Serial.println("Initialize end");
}

void loop() {
  blob_count = get_openMV_coordinate();
  x_data_ball = (openMV[5] & 0b0000000000111111) + ((openMV[6] & 0b0000000000111111) << 6);
  y_data_ball = (openMV[7] & 0b0000000000111111) + ((openMV[8] & 0b0000000000111111) << 6);
  w_data_ball = (openMV[9] & 0b0000000000111111) + ((openMV[10] & 0b0000000000111111) << 6);
  h_data_ball = (openMV[11] & 0b0000000000111111) + ((openMV[12] & 0b0000000000111111) << 6);

  x_data_yellowgoal = (openMV[18] & 0b0000000000111111) + ((openMV[19] & 0b0000000000111111) << 6);
  y_data_yellowgoal = (openMV[20] & 0b0000000000111111) + ((openMV[21] & 0b0000000000111111) << 6);
  w_data_yellowgoal = (openMV[22] & 0b0000000000111111) + ((openMV[23] & 0b0000000000111111) << 6);
  h_data_yellowgoal = (openMV[24] & 0b0000000000111111) + ((openMV[25] & 0b0000000000111111) << 6);

  x_data_bluegoal = (openMV[31] & 0b0000000000111111) + ((openMV[32] & 0b0000000000111111) << 6);
  y_data_bluegoal = (openMV[33] & 0b0000000000111111) + ((openMV[34] & 0b0000000000111111) << 6);
  w_data_bluegoal = (openMV[35] & 0b0000000000111111) + ((openMV[36] & 0b0000000000111111) << 6);
  h_data_bluegoal = (openMV[37] & 0b0000000000111111) + ((openMV[38] & 0b0000000000111111) << 6);


  if (lineflag == true) {
    lineflag = false;
  }

  // get gyro data
  if (Serial2.available() > 0)
    while (Serial2.available() != 0) { //  Gyroの方位データをgyroに取り込む
      gyro = Serial2.read();
    }
  //Xbeeからの信号を読む
  if (Serial1.available() > 0)
    while (Serial1.available() > 0) {
      finish = Serial1.read();
    }
  // openMVのデーターを変換

  sig = openMV[1]; //  openMVのデータをsig,x,y,w,hに取り込む
  x = x_data_ball;
  y = y_data_ball;
  w = w_data_ball;
  h = h_data_ball;
  area = w * h;      // 認識したブロックの面積
  b_sig = openMV[27];
  bg_x = x_data_bluegoal;
  bg_y = y_data_bluegoal;
  bg_w = w_data_bluegoal;
  bg_h = h_data_bluegoal;
  bg_area = bg_w * bg_h;      // 認識したブロックの面積
  y_sig = openMV[14];
  yg_x = x_data_yellowgoal;
  yg_y = y_data_yellowgoal;
  yg_w = w_data_yellowgoal;
  yg_h = h_data_yellowgoal;
  yg_area = yg_w * yg_h;      // 認識したブロックの面

  if (sig != 0) { //中心補正
    x = 163 - x;
    y = 120 - y;
  }
  if (y_sig != 0) {
    yg_x = 160 - yg_x;
    yg_y = yg_y - 172;
  }
  if (b_sig != 0) {
    bg_x = 168 - bg_x;
    bg_y = bg_y - 68;
  }

  if (sig != 0) { //補正
    x = (x * 211800) / (140200 + 708 * sqrt(x * x + 28900));
    y = (y * 194600) / (140200 + 708 * sqrt(y * y + 28900));
  }
  if (y_sig != 0) {
    yg_x = -(yg_x * 211800) / (140200 + 708 * sqrt(yg_x * yg_x + 28900));
    yg_y = (yg_y * 194600) / (140200 + 708 * sqrt(yg_y * yg_y + 28900));
  }
  if (b_sig != 0) {
    bg_x = -(bg_x * 211800) / (140200 + 708 * sqrt(bg_x * bg_x + 28900));
    bg_y = (bg_y * 194600) / (140200 + 708 * sqrt(bg_y * bg_y + 28900));
  }

  //Serial1.write(xbee_date);// xbeeへ出力

  Serial.print(" ,x:");
  Serial.print(x);
  Serial.print(" ,y:");
  Serial.print(y);
  Serial.print(" ,ygoal_x=");//青ゴールのx座標
  Serial.print(yg_x);
  Serial.print(" ,ygoal_y="); //青ゴールのy座標
  Serial.print(yg_y);
  Serial.print(" ,bgoal_x=");//青ゴールのx座標
  Serial.print(bg_x);
  Serial.print(" ,bgoal_y="); //青ゴールのy座標
  Serial.print(bg_y);
  Serial.print(" ,progress=");
  Serial.print(progress);
  Serial.println();


  ball_back = ToF_back.readRangeSingleMillimeters();
  ball_front = ToF_front.readRangeSingleMillimeters();
  ball_Long1 = ToF_Long1.readRangeSingleMillimeters();

  Serial.print(" ToF_Long1=");
  Serial.print(ball_Long1);
  if (ToF_Long1.timeoutOccurred()){
    Serial.print(" ,ToF_Long1 TIMEOUT");
    digitalWrite(LED_Y, HIGH);
    ball_Long1 = 1000;
  } else {
    digitalWrite(LED_Y, LOW);
  }


  if (digitalRead(StartSW) == LOW) { // STartSW == Lowでスタート
    digitalWrite(SWR, HIGH);
    digitalWrite(SWG, LOW);

    //checkvoltage(Vlow);  //  電池の電圧をチェック
    if ( emergency == true ) {
      Serial.println("");
      Serial.println("  Battery Low!");
      doOutofbound();    //  故障なのでコートの外へ
    }

    checkvoltage(Vlow);
    if (emergency == true) {       // 電池の電圧が下がっていたら
      digitalWrite(LINE_LED, LOW); // ラインセンサのLEDを消灯
      motorFree();                 //　モーターを停止
      while (1) {                  //　無限ループ
        digitalWrite(SWR, LOW);
        digitalWrite(SWG, LOW);
        delay(300);
        digitalWrite(SWR, HIGH);
        digitalWrite(SWG, HIGH);
        delay(300);
      }
    }

    digitalWrite(LINE_LED, HIGH);

    if (digitalRead(GoalSW)) {  // 青色の場合
      goal_sig = y_sig;
      goal_x = yg_x;
      goal_y = yg_y;
    } else {                     // 黄色の場合
      goal_sig = b_sig;
      goal_x = bg_x;
      goal_y = bg_y;
    }

    //----------------main-------------------

    if(finish == 128){
      progress = 1;
    }

    if(abs(gyro) < 6){
      if(progress == 0){
        motorfunction(0, 0, 0);
      }else if(progress == 1){//
          if(yg_x > -35){//ペットボトルのとこ
            if(yg_x > 70){//初期位置
              motorfunction(-PI/3, 20, -gyro);
            } else {//横移動
              motorfunction(-PI/2, 30, -gyro);
            }
          } else {//ペットボトルをでる
            progress = 2;
            finish = 0;
            motorfunction(0, 0, 0);
          }
      }else{//とってゴールに入れる
        if(sig == 0){
          motorfunction(0, 20, -gyro);//ボールを見つけるために前に行く
        }else{
          if(0 < y && y < 50){//一定距離(y)内でドリブラーを回す
            dribbler1(100);
            if(abs(x) < 4){//ボールとx座標が大体同じ
              if(y < 33){//持っている
                if(abs(bg_x) < 10){//キックする
                  dribbler1(100);
                  motorfunction(0, 0, 0);
                  delay(600);
                  dribbler1(0);
                  digitalWrite(Kick_Dir, LOW);
                  delay(500);
                  digitalWrite(Kicker, HIGH);
                  delay(1500);
                  digitalWrite(Kicker, LOW);
                  progress = 0;
                  finish = 0;
                }else{
                  m = PI/3;
                  motorfunction(m, abs(bg_x)*2, -gyro);
                }
              }else{
                motorfunction(0, 30, -gyro);//直進(ドリブラーあり)
              }
            }else{
              m = atan2(x, y - 25);
              motorfunction(m ,abs(x) + 10 , -gyro);//ボールの前へ(ドリブラーあり)
            }
          }else if(y < 0){
            motorfunction(0, 0, 0);//後ろにボールがあるなら停止
          }else{
            m = atan2(x, y - 25);
            motorfunction(m, abs(y) + 10, -gyro);//前ならボールの前へ(ドリブラーなし)
          }
        }
      }
  //
  // **** end of main loop ******************************************************
  //
    }else{
      digitalWrite(LED_BUILTIN, HIGH);
      power = abs(gyro);   //  モーターの速度をgyroにする
      if (gyro > 0) {         // Ball is 1st quadrant
        turnCCW(power);
      } else if (gyro < 0) {  // 2nd quadrant
        turnCCW(-power);
      } else {
      }
    }
  }else{
    motorFree();
    dribbler1(0);
    dribbler2(0);
    digitalWrite(LINE_LED, LOW); // ラインセンサのLEDを消灯
    digitalWrite(SWR, LOW);
    digitalWrite(SWG, HIGH);
    digitalWrite(LED_Y, LOW);
    progress = 0;
    finish = 0;
  }
}

  int get_openMV_coordinate() { // get the coordinate data of orange ball
    int i;
    while (Serial3.available() != 0) { // buffer flush
      Serial3.read();
    }
    while ((openMV[0] = getOpenMV()) != 254); // wait for "254"
    for (i = 1; i < 39; i++) {
      openMV[i] = getOpenMV();
    }
    return openMV[0];
  }

  int getOpenMV() { // get serial data from openMV
    while (Serial3.available() == 0); // wait for serial data
    return Serial3.read();
  }


  //*****************************************************************************
  // interrupt handler
  // 割り込みの処理プログラム
  // Lineを踏んだらバックする

  void intHandle() {  // Lineを踏んだらlineflagをセットして止まる。
    int power;

    if ( digitalRead(StartSW) == HIGH)  // スイッチがOFFなら何もしない。
      return;
    power = 30;

    while (digitalRead(INT_29) == HIGH) {     // Lineセンサが反応している間は繰り返す
      if (digitalRead(LINE1D) == HIGH) {     // lineを踏んだセンサーを調べる
        back_Line1(power);        // Lineセンサと反対方向へ移動する
        lineflag = true;          // set lineflag
      } else if (digitalRead(LINE2D) == HIGH) {
        back_Line2(power);
        lineflag = true;          // set lineflag
      } else if (digitalRead(LINE3D) == HIGH) {
        back_Line3(power);
        lineflag = true;          // set lineflag
      } else if (digitalRead(LINE4D) == HIGH) {
        back_Line4(power);
        lineflag = true;          // set lineflag
      }
    }

    if (lineflag == false)     // センサーの反応がない場合は何もしない
      return;
    lineflag = true;          // set lineflag
    motorStop();                  // ラインから外れたらモーターstop
    return;
  }

  void back_Line1(int power) {             // Lineセンサ1が反応しなくなるまで後ろに進む
    float azimuth;
    digitalWrite(LED_R, HIGH);  // LED_R点灯
    while ((digitalRead(LINE1D) == HIGH) || (digitalRead(LINE5D) == HIGH) || (digitalRead(LINE3D) == HIGH)) {
      if (digitalRead(LINE4D) == HIGH) {
        azimuth = 3.14159 * 3.0 / 4.0;   // 後ろ方向(1+4)をradianに変換
      } else if (digitalRead(LINE2D) == HIGH) {
        azimuth = 3.14159 * 5.0 / 4.0;   // 後ろ方向(1+2)をradianに変換
      } else {
        azimuth = 3.14159 * 4.0 / 4.0;   // 後ろ方向(3)をradianに変換
      }
      motorfunction(azimuth, power, 0);  // azimuthの方向に進ませる
    }
    digitalWrite(LED_R, LOW);  // LED_R消灯
    motorStop();
  }

  void back_Line2(int power) {             // Lineセンサ2が反応しなくなるまで左に進む
    float azimuth;
    digitalWrite(LED_Y, HIGH);  // LED_Y点灯
    while ((digitalRead(LINE2D) == HIGH) || (digitalRead(LINE5D) == HIGH) || (digitalRead(LINE4D) == HIGH)) {
      if (digitalRead(LINE1D) == HIGH) {
        azimuth = 3.14159 * 5.0 / 4.0;   // 後ろ方向(2+1)をradianに変換
      } else if (digitalRead(LINE3D) == HIGH) {
        azimuth = 3.14159 * 7.0 / 4.0;   // 後ろ方向(2+3)をradianに変換
      } else {
        azimuth = 3.14159 * 6.0 / 4.0;   // 後ろ方向(4)をradianに変換
      }
      motorfunction(azimuth, power, 0);  // azimuthの方向に進ませる
    }
    digitalWrite(LED_Y, LOW);  // LED_Y消灯
    motorStop();
  }

  void back_Line3(int power) {             // Lineセンサ3が反応しなくなるまで前に進む
    float azimuth;
    digitalWrite(LED_G, HIGH);  // LED_G点灯
    while ((digitalRead(LINE3D) == HIGH) || (digitalRead(LINE5D) == HIGH) || (digitalRead(LINE1D) == HIGH)) {
      if (digitalRead(LINE4D) == HIGH) {
        azimuth = 3.14159 * 1.0 / 4.0;   // 後ろ方向(3+4)をradianに変換
      } else if (digitalRead(LINE2D) == HIGH) {
        azimuth = 3.14159 * 7.0 / 4.0;   // 後ろ方向(3+2)をradianに変換
      } else {
        azimuth = 3.14159 * 0.0 / 4.0;   // 後ろ方向(1)をradianに変換
      }
      motorfunction(azimuth, power, 0);  // azimuthの方向に進ませる
    }
    digitalWrite(LED_G, LOW);  // LED_G消灯
    motorStop();
  }

  void back_Line4(int power) {             // Lineセンサ4が反応しなくなるまで右に進む
    float azimuth;
    digitalWrite(LED_B, HIGH);  // LED_B点灯
    while ((digitalRead(LINE4D) == HIGH) || (digitalRead(LINE5D) == HIGH) || (digitalRead(LINE2D) == HIGH)) {
      if (digitalRead(LINE3D) == HIGH) {
        azimuth = 3.14159 * 1.0 / 4.0;   // 後ろ方向(4+3)をradianに変換
      } else if (digitalRead(LINE1D) == HIGH) {
        azimuth = 3.14159 * 3.0 / 4.0;   // 後ろ方向(4+1)をradianに変換
      } else {
        azimuth = 3.14159 * 2.0 / 4.0;   // 後ろ方向(2)をradianに変換
      }
      motorfunction(azimuth, power, 0);  // azimuthの方向に進ませる
    }
    digitalWrite(LED_B, LOW);  // LED_B消灯
    motorStop();
  }


  //
  //割り込みの処理プログラム終わり
  //*****************************************************************************
  //

  //
  //電池電圧を監視して電圧が下がったらOutOfBounceさせる処理**********************
  //

  float checkvoltage(float Vlow) {  // 電池電圧を監視する。
    int voltage, limit;
    limit = Vlow / 0.01811;
    voltage = analogRead(Vbatt); // Get Volatge
    if ( voltage < limit ) {      // 電圧が　Vlow以下であればemergencyをセットする。
      emergency = true;
      digitalWrite(SWG, LOW);
      digitalWrite(SWR, LOW);

    }
    return voltage * 0.01811;
  }

  void doOutofbound() { // 強制的にOut of bounds させる。

    detachInterrupt(5);   // Out of bounds するために割込みを禁止する
    digitalWrite(LINE_LED, LOW); // ラインセンサのLEDを消灯

    while (true) {  // 無限ループ
      if ( digitalRead(StartSW) == LOW)
        motorfunction(3.14159 / 2.0, 30, 0);
      else          // スタートスイッチが切られたら止まる
        motorfunction(3.14159 / 2.0, 0, 0);
      digitalWrite(SWG, LOW);
      digitalWrite(SWR, LOW);
      delay(25);
      digitalWrite(SWG, HIGH);
      digitalWrite(SWR, HIGH);
      delay(25);

    }
  }
