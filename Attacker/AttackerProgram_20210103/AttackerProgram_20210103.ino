//
// Robot main program
//           Ver 1.0 Aug.24.2019
//           Ver.2.0 Jan.13.2020
//           By Saeki Akitoshi
//
//           Ver 3.0 Nov.13.2020
//           Ver 3.1 Dec.06.2020
//           Ver 3.2 Jan.03.2021
//           modefied by H.Saeki
//


#include "NT_Robot202011.h" // Header file for Teensy 3.5
#include "motorDRV4.h" //  モーター制御のプログラムを読み込む

#include <Wire.h>
#include <PCF8574.h>
#include <VL6180X.h>

PCF8574 pcf8574(I2C_PCF8574);
VL6180X ToF_front;  // create front ToF object
VL6180X ToF_back;   // create back ToF object

const int angle = 0;
const int Vlow = 13.0;  // Low limit voltage 1.1*12 = 13.2
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

void setup() {

  pinMode(StartSW, INPUT_PULLUP);

  // IOピンのモード設定

  pinMode(ledPin, OUTPUT);

  pinMode(angle, INPUT);

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

  pinMode(SWR, OUTPUT);
  pinMode(SWG, OUTPUT);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  digitalWrite(Kick1, LOW);
  digitalWrite(SWR, LOW);
  digitalWrite(SWG, LOW);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(INT_29, INPUT_PULLUP);          // interrupt port set

//*****************************************************************************
// Set initial value to variable

  emergency = false;
  outofbounds = false;
  power = 70;   //  set initial motor power

//*****************************************************************************
  // initialize ToF sensers
  delay(10);
  pcf8574.digitalWrite(5 - 1, HIGH); //Activate ToF_front VL6180X
  delay(10);
  ToF_front.init();
  ToF_front.configureDefault();
  ToF_front.setAddress(TOF_5);  //好きなアドレスに設定
  ToF_front.setTimeout(100);
  delay(10);

  pcf8574.digitalWrite(6 - 1, HIGH); //Activate ToF_back VL6180X
  delay(10);
  ToF_back.init();
  ToF_back.configureDefault();
  ToF_back.setAddress(TOF_6);  //好きなアドレスに設定
  ToF_back.setTimeout(100);
  delay(10);

  motorInit();  //  モーター制御の初期化
  dribInit();   //  ドリブラモーターの初期化

  delay(1000);  //  ドリブラ・キッカーの動作チェック
  dribbler1(50);
  dribbler2(50);
  delay(1500);
  dribbler1(0);
  dribbler2(0);
  delay(100);
  digitalWrite(Kick1, HIGH);
  delay(100);
  digitalWrite(Kick1, LOW);

  Serial3.begin(19200);  // initialize serialport for openMV
  Serial2.begin(115200);   // WT901 IMU Sener

  lineflag = false;   //reset outofbounds　flag
  for ( i = 0; i < 4; i++ )
    line[i] = false;
  robot_dir = 0;  //reset robot direction

  // Caution D29 -> Interrupt5

  attachInterrupt(INT_29, intHandle, RISING);

  Serial.begin(9600);   //  シリアル出力を初期化
  Serial.println("Starting...");

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
  int sig, w, h, area;
  int bg_w, bg_h, bg_area;
  int yg_w, yg_h, yg_area;
  float m , z;
  float x, y;
  float bg_x, bg_y;
  float yg_x, yg_y;
  float goal_x, goal_y;
  float y_sig, b_sig, goal_sig;
  int ball_tof;

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

  // get gyro data
  if (Serial2.available() > 0)
    while (Serial2.available() != 0) { //  Gyroの方位データをgyroに取り込む
      gyro = Serial2.read();
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
  yg_area = yg_w * yg_h;      // 認識したブロックの面積

  if (digitalRead(goal_setup)) { // goal_setupはHighなら「青ゴール」を認識する。
    goal_sig = b_sig;
    goal_x = bg_x;
    goal_y = bg_y;

    //for ( pixel = 0; pixel < strip.numPixels(); pixel++) {
    //  strip.setPixelColor(pixel, strip.Color(0, 0, 255));
    //  strip.show();
    //}

  } else {
    goal_sig = y_sig;
    goal_x = yg_x;
    goal_y = yg_y;
  }

  // Convert coordinates data
  if (sig != 0) {
    x = 170 - x;
    y = 117 - y;
  }
  if (goal_sig != 0) {
    goal_x = goal_x - 170 ;
    goal_y = 110 - goal_y;
  }

  ball_tof = ToF_front.readRangeSingleMillimeters();
  Serial.print(" Sig=");  //  openMVのデータを出力する
  Serial.print(sig);
  Serial.print(" X=");
  Serial.print(x);
  Serial.print(" Y=");
  Serial.print(y);
  Serial.print(" goal_x=");
  Serial.print(goal_x);
  Serial.print(" goal_y=");
  Serial.print(goal_y);
  Serial.print(" ball_tof=");
  Serial.print(ball_tof);
  Serial.print(" gyro=");
  Serial.print(gyro);
  Serial.println();


  if (digitalRead(StartSW) == LOW) { // STartSW == Lowでスタート
    digitalWrite(SWR, HIGH);
    digitalWrite(SWG, LOW);

    checkvoltage(Vlow);  //  電池の電圧をチェック
    if ( emergency == true ) {
      Serial.println("");
      Serial.println("  Battery Low!");
      doOutofbound();    //  故障なのでコートの外へ
    }

    digitalWrite(LINE_LED, HIGH); // ラインセンサのLEDを点灯
    if (lineflag == true) {
      lineflag = false;
    }
    if (abs(gyro) < 20) {
      digitalWrite(LED_BUILTIN, LOW);
      if (ToF_front.readRangeSingleMillimeters() <= 50) {  //　ドリブラの直近にボールがあればドリブラを回す
        dribbler(50);
        if ( goal_sig == 0) {
          //dribbler(100);
          motorfunction(0, power, -gyro);
        } else {
          if ((goal_y >= 50) && (goal_y <= 80)) {
            dribbler(50);
            digitalWrite(Kick1, HIGH);
            delay(1000);
            dribbler(0);
            digitalWrite(Kick1, LOW);
          } else {
            dribbler(50);
            if (abs(goal_x) < 2) {
              motorfunction(0, power, -gyro);
            } else {
              if (goal_y <= 40) {
                m = goal_x / goal_y;
                z = atan(m) + PI; // arc tangent of m
                motorfunction(z, 10 + 20 * abs(goal_x), -gyro);
                delay(500);
              }
            }
          }
        }
      } else {    //　ドリブラの直近にボールがなければドリブラを止める
        dribbler(0);
        if (sig == 0) {      // No Ball found
          motorfunction(0, 0, 0);
        } else {                // Ball find
          if (y >= 70) {
            motorfunction(0 , power, -gyro);
          } else {
            if ((abs(x) < 2) && (y >= 0)) {
              motorfunction(0 , 30, -gyro);
              dribbler(50);
              delay(100);
            } else {
              if (y >= 40) {
                dribbler(0);
                m = y / x;
                z = atan(m); // arc tangent of m
                motorfunction(z, (abs(x) + abs(y)) / 2 , -gyro);
              } else {
                if (y < 0) {
                  if ( y <= 40) {
                    dribbler(0);
                    m = y / -(5 * x);
                    z = atan(m) + PI; // arc tangent of m
                    motorfunction(z, abs(y) + 40, -gyro);
                  } else {
                    dribbler(0);
                    m = y / (2.5 * x);
                    z = atan(m) + PI; // arc tangent of m
                    motorfunction(z, abs(y) + 40, -gyro);
                  }
                } else {
                  dribbler(0);
                  m = 0.5 * y / -x;
                  z = atan(m) + PI; // arc tangent of m
                  motorfunction(z, abs(y) + 40, -gyro);
                }
              }
            }
          }
        }
      }
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
      power = abs(gyro);   //  モーターの速度をgyroにする
      if (gyro > 0) {         // Ball is 1st quadrant
        turnCCW(power);
      } else if (gyro < 0) {  // 2nd quadrant
        turnCCW(-power);
      } else {

      }
    }
  } else {  // ロボット停止
    motorFree();
    dribbler(0);
    digitalWrite(LINE_LED, LOW); // ラインセンサのLEDを消灯
    digitalWrite(SWR, LOW);
    digitalWrite(SWG, HIGH);
  }
}


//
// **** end of main loop ******************************************************
//


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
