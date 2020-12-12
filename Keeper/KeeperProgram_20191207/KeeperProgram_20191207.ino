/*
  DSR1603 Magnetic Gyro test
  It's simply rotate the robot or direct one way.
  for Arduino mega
  Vcc=5V
  I2C to DSR1603Senser

  01.Dec.2019 by saeki

*/

#include "motorDRV.h" //  モーター制御のプログラムを読み込む
#include  "LED_TEST.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS 100

/* Set Senser Address to 0x28 */
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

const int LINE_LED = 10;  //  LINEセンサのLED制御
const int LINE1D = 22;  //  センサ1のデジタル入力
const int LINE1A = A0;  //  センサ1のアナログ入力
const int LINE2D = 23;  //  センサ2のデジタル入力
const int LINE2A = A1;  //  センサ2のアナログ入力
const int LINE3D = 24;  //  センサ3のデジタル入力
const int LINE3A = A2;  //  センサ3のアナログ入力
const int LINE4D = 25;  //  センサ4のデジタル入力
const int LINE4A = A3;  //  センサ4のアナログ入力

const int INT_5 = 18;   // Interrupt5

const int StartSW = A9;

static int  power = 50;   // 移動速度を保持する
static int  robot_dir;    // 移動方向を保持する

int blob_count, i;
static int openMV[39];
static int x_data_ball, y_data_ball, w_data_ball, h_data_ball;
static int x_data_yellowgoal, y_data_yellowgoal, w_data_yellowgoal, h_data_yellowgoal;
static int x_data_bluegoal, y_data_bluegoal, w_data_bluegoal, h_data_bluegoal;

static int8_t  gyro;
static int ball_dir;

static int  outofbounds;   // "out of bounds" flag

static int  lineflag;     // check out of bounds flag
static int  line[4];      // ラインセンサの状態を保持する配列

//int power;

void setup() {
  int i;
  // put your setup code here, to run once:

  // Line Senserの使用ピンを初期化する
  pinMode(LINE_LED, OUTPUT);
  pinMode(LINE1D, INPUT_PULLUP);
  pinMode(LINE2D, INPUT_PULLUP);
  pinMode(LINE3D, INPUT_PULLUP);
  pinMode(LINE4D, INPUT_PULLUP);
  pinMode(LINE1A, INPUT);
  pinMode(LINE2A, INPUT);
  pinMode(LINE3A, INPUT);
  pinMode(LINE4A, INPUT);

  // 割り込みピンを初期化する
  pinMode(INT_5, INPUT);          // interrupt set

  // モーターを初期化する
  motorInit();

  digitalWrite(LINE_LED, HIGH); // ラインセンサのLEDを点灯
  LED_Init();                   // LEDを初期化する
  lineflag = false;             // reset lineflag
  for ( i = 0; i < 4; i++)      // ラインセンサの状態フラグをクリア
    line[i] == false;
  robot_dir = 0;      //reset robot direction
  randomSeed(0);      // 乱数を初期化する

  // 最初は０の方向（正面）へpowerで進む
  robot_dir = 0;

  // 割り込みを初期化する。これはSetupの最後で実行
  // pin5の立下りで割り込む
  // Caution D18 -> Interrupt5
  attachInterrupt(5, intHandle, RISING);

  DSR1603_init();  // DSR1603の初期化

  Serial.begin(9600);
  Serial2.begin(19200);  // initialize serialport foe openMV
  Serial3.begin(115200);
  Serial.println("Start");
  power = 30;   //  モーターの速度を30にする
}

void loop() {
  int gyro;
  int i;
  unsigned int d_time;
  int level, data;
  int j;
  int sig, w, h, area;
  int bg_w, bg_h, bg_area;
  int yg_w, yg_h, yg_area;
  int blocks;
  char buf[64];
  float m , z;
  float x, y;
  float bg_x, bg_y;
  float yg_x, yg_y;
  float goal_x, goal_y;

  if (digitalRead(StartSW)) {
    LED01(HIGH);
    digitalWrite(LINE_LED, HIGH); // ラインセンサのLEDを点灯

    blob_count = get_openMV_coordinate();
    LED03(HIGH);
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

    if (lineflag == true) {         // lineを踏んでいたら
      robot_dir = random(8);    // 乱数で新しい進行方向をセットする。
      lineflag = false;             // lineflagをクリアする。
    }

    // openMVのデーターを変換

    sig = openMV[1]; //  openMVのデータをsig,x,y,w,hに取り込む
    x = x_data_ball;
    y = y_data_ball;
    w = w_data_ball;
    h = h_data_ball;
    area = w * h;      // 認識したブロックの面積
    bg_x = x_data_bluegoal;
    bg_y = y_data_bluegoal;
    bg_w = w_data_bluegoal;
    bg_h = h_data_bluegoal;
    bg_area = bg_w * bg_h;      // 認識したブロックの面積
    yg_x = x_data_yellowgoal;
    yg_y = y_data_yellowgoal;
    yg_w = w_data_yellowgoal;
    yg_h = h_data_yellowgoal;
    yg_area = yg_w * yg_h;      // 認識したブロックの面積

    goal_x = yg_x;
    goal_y = yg_y;

    // Convert coordinates data
    if (sig != 0) {
      x = 169 - x;
      y = 110 - y;
      goal_x = 180 - goal_x;
      goal_y = 110 - goal_y;
    }

    Serial.print(" Sig=");  //  pixyのデータを出力する
    Serial.print(sig);
    Serial.print(" X=");
    Serial.print(x);
    Serial.print(" Y=");
    Serial.print(y);
    Serial.print(" goal_x=");
    Serial.print(goal_x);
    Serial.print(" goal_y=");
    Serial.print(goal_y);
    Serial.println();

    Serial.print(gyro);
    Serial.println();

    gyro = get_DSR1603();
    Serial.print("I received: "); // 受信データを送りかえす
    Serial.println(gyro, DEC);

    if (abs(gyro) < 5) {
          //motorFoward(0, 0);
          if (sig == 0) {      // No Ball found
            motorFoward(0, 0);
          } else {                // Ball find
            if (x >= 0) {
              motorFoward(2, 3 * abs(x));
            } else {
              motorFoward(6, 3 * abs(x));
            }
          }
    } else {
      LED01(HIGH);
      digitalWrite(LED_BUILTIN, HIGH);
      power = gyro;   //  モーターの速度をgyroにする

      if (gyro > 0) {  // Ball is 1st quadrant
        turnCCW(20);
      } else if (gyro < 0) {  // 2nd quadrant
        turnCCW(-20);
      } else {
        // motorFoward(0, 0);  // Stop
      }
    }
  } else {
    LED01(LOW);
    digitalWrite(LINE_LED, LOW); // ラインセンサのLEDを消灯
    motorFowardRotate(0, 0, 0); // Stop
    LED01(LOW);
    LED04(LOW);
  }
}

int get_openMV_coordinate() { // get the coordinate data of orange ball
  int i;
  while (Serial2.available() != 0) { // buffer flush
    Serial2.read();
  }
  while ((openMV[0] = getOpenMV()) != 254); // wait for "254"
  for (i = 1; i < 39; i++) {
    openMV[i] = getOpenMV();
  }
  return openMV[0];
}

int getOpenMV() { // get serial data from openMV
  while (Serial2.available() == 0); // wait for serial data
  return Serial2.read();
}

void DSR1603_init() {
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

int8_t get_DSR1603() {
  float azimuth;
  int8_t gyro;

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  /* Display the floating point data */
  azimuth = euler.x() + 80.0;
  if ( azimuth > 360 )
    azimuth = azimuth - 360.0;
  if (azimuth <= 180.0)
    gyro = azimuth * (127.0 / 180.0);
  else
    gyro = (azimuth - 360.0) * (127.0 / 180.0);
  return gyro;
}

// interrupt handler
// 割り込みの処理プログラム

void intHandle() {  // Lineを踏んだらlineflagをセットして止まる。
  int i, old_dir;
  LED03(HIGH);              // LEDを点ける
  old_dir = robot_dir;      // 今の進行方向を記憶する
  for (i = 0; i < 4; i++)
    line[i] = false;
  if (digitalRead(LINE1D) == HIGH) {     // lineを踏んだセンサーを調べる
    line[0] = true;
    robot_dir = 4;
    lineflag = true;          // set lineflag
  }
  if (digitalRead(LINE2D) == HIGH) {
    line[1] = true;
    robot_dir = 6;
    lineflag = true;          // set lineflag
  }
  if (digitalRead(LINE3D) == HIGH) {
    line[2] = true;
    robot_dir = 0;
    lineflag = true;          // set lineflag
  }
  if (digitalRead(LINE4D) == HIGH) {
    line[3] = true;
    robot_dir = 2;
    lineflag = true;          // set lineflag
  }

  if (lineflag = false) {     // センサーの反応がない場合は
    robot_dir = old_dir;      // 方向を元に戻す
    LED02(HIGH);
    LED02(LOW);
    LED03(LOW);
    return;
  }
  while (digitalRead(INT_5) == HIGH) { // lineを踏んでいる間は
    motorFowardRotate(robot_dir, 50, 0);// robot_dirの方向に進ませる
  }
  motorFoward(0, 0);                  // ラインから外れたらモーターstop
  LED03(LOW);
  lineflag = true;
  return;
}
