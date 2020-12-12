//
// Robot main program
//           Ver 1.0 Dece.19.2019
//           By Hiroya Hashimoto and Higaki Shuta
//

//Motor control program
#include "motorDRV3.h" //  モーター制御のプログラムを読み込む
#include <Adafruit_NeoPixel.h>
#include <math.h>
#define PIN            40
#define NUMPIXELS      16
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

const int StartSW = A14;

//  ラインセンサのピンを設定する
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
const int Angel1 = 19;
const int Goal_SW = A6;

int blob_count, i;
static int openMV[39];
static int x_data_ball, y_data_ball, w_data_ball, h_data_ball;
static int x_data_yellowgoal, y_data_yellowgoal, w_data_yellowgoal, h_data_yellowgoal;
static int x_data_bluegoal, y_data_bluegoal, w_data_bluegoal, h_data_bluegoal;

static int8_t  gyro;
static int robot_dir, ball_dir, power;

static int  outofbounds;   // "out of bounds" flag

static int lineflag;//line
static int line[4];

void setup() {

  strip.begin();
  strip.show();
  strip.setBrightness(100);

  pinMode(StartSW, INPUT_PULLUP);

  // IOピンのモード設定


  pinMode(LINE_LED, OUTPUT);
  pinMode(LINE1D, INPUT_PULLUP);
  pinMode(LINE2D, INPUT_PULLUP);
  pinMode(LINE3D, INPUT_PULLUP);
  pinMode(LINE4D, INPUT_PULLUP);
  pinMode(LINE1A, INPUT);
  pinMode(LINE2A, INPUT);
  pinMode(LINE3A, INPUT);
  pinMode(LINE4A, INPUT);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(INT_5, INPUT);          // interrupt port set

  motorInit();  //  モーター制御の初期化
  power = 10;   //  set initial motor power

  Serial2.begin(19200);  // initialize serialport foe openMV
  Serial3.begin(115200); // KemarinTech Gyro senser

  lineflag = false;   //reset outofbounds　flag
  for ( i = 0; i < 4; i++ )
    line[i] = false;
  robot_dir = 0;  //reset robot direction

  // Caution D18 -> Interrupt5

  // attachInterrupt(5, intHandle, RISING);

  Serial.begin(9600);   //  シリアル出力を初期化
  Serial.print("Starting...\n");

  //digitalWrite(Kick1, HIGH);
  //delay(100);
  //digitalWrite(Kick1, LOW);
    for (int i = 0; i < NUMPIXELS; i++) {
      strip.setPixelColor(i, strip.Color(100, 100, 100) );
      strip.show();
      delay(100); // Delay for a period of time (in milliseconds).
    }  
}

void loop() {
  int level, data;
  int j;
  int sig, w, h, area;
  int bg_w, bg_h, bg_area;
  int yg_w, yg_h, yg_area;
  int blocks;
  int i;
  int ball_y;
  float pointP;
  char buf[64];
  float m , z;
  float x, y;
  float bg_x, bg_y;
  float yg_x, yg_y;
  float goal_x, goal_y;
  float y_sig,b_sig,goal_sig;
  float az, AZ, d, k;
  float circleX, circleY;
  //int i;

  int pixel;
  uint32_t color;


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
    if (Serial3.available() > 0)
      while (Serial3.available() != 0) { //  Gyroの方位データをgyroに取り込む
        gyro = Serial3.read();
      }

    // openMVのデーターを変換

    sig = openMV[1]; //  openMVのデータをsig,x,y,w,hに取り込む
    x = x_data_ball;
    y = y_data_ball;
    w = w_data_ball;
    h = h_data_ball;
    area = w * h;      // 認識したブロックの面積
    b_sig=openMV[27];
    bg_x = x_data_bluegoal;
    bg_y = y_data_bluegoal;
    bg_w = w_data_bluegoal;
    bg_h = h_data_bluegoal;
    bg_area = bg_w * bg_h;      // 認識したブロックの面積
    y_sig=openMV[14];
    yg_x = x_data_yellowgoal;
    yg_y = y_data_yellowgoal;
    yg_w = w_data_yellowgoal;
    yg_h = h_data_yellowgoal;
    yg_area = yg_w * yg_h;      // 認識したブロックの面積

    if(digitalRead(Goal_SW)) {
    	goal_sig=b_sig;
    	goal_x = bg_x;
    	goal_y = bg_y;
    }else{
    	goal_sig=y_sig;
    	goal_x = yg_x;
    	goal_y = yg_y;
    }

    goal_x = goal_x - 145 ;
    goal_y = goal_y - 162 ;
    AZ = PI/2.0;

    pointP = 45.0;
    k = 0.1;

float goal_dist = goal_x * goal_x + goal_y * goal_y;
    goal_dist = sqrt(goal_dist)

    d = k*(pointP - goal_dist)
    if(d < -1.5){
      d = -1.5;
    }else if(d > 1.5){
      d = 1.5;
    }

    // Convert coordinates data
    if (sig != 0) {
      x = 164 - x;
      y = 88 - y;
      ball_y = goal_y + y;
    }

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
    Serial.println();

    Serial.print(gyro);
    Serial.println();


  // check line and reverse
  if (digitalRead(StartSW) && (digitalRead(Angel1) == LOW)) {
    digitalWrite(LINE_LED, HIGH); // ラインセンサのLEDを点灯

    if (abs(gyro) <= 7) {
      digitalWrite(LED_BUILTIN, LOW);
      for(circleX = 0; circleX < 1; circleX+=0.01){
        circleY = sqrt(circleX * circleX) + 1;
        d = k*(circleY - goal_y);
        motorfunction(d, 20, -gyro);
      }
      } else {
      digitalWrite(LED_BUILTIN, HIGH);
      power = abs(gyro);   //  モーターの速度をgyroにする

      if (gyro > 0) {         // Ball is 1st quadrant
        turnCCW(power);
      } else if (gyro < 0) {  // 2nd quadrant
        turnCW(power);
      } else {

      }
    }
  } else {
    motorFree();
    dribbler(0);
    digitalWrite(LINE_LED, LOW); // ラインセンサのLEDを消灯

  for ( pixel = 0; pixel < strip.numPixels(); pixel++) {
    strip.setPixelColor(pixel, strip.Color(100, 100, 100));
    strip.show();
  }    
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

// interrupt handler

void intHandle() {
  int i;
  if (!digitalRead(StartSW))
  	return;
  
  for ( i = 0; i < 4; i++ )
    line[i] = false;
  if (digitalRead(LINE1D) == HIGH ) {
    line[0] = true;
    robot_dir = 3.14;
    lineflag = true;
  }
  if (digitalRead(LINE2D) == HIGH ) {
    line[1] = true;
    robot_dir = 3 * 3.14 / 2;
    lineflag = true;
  }
  if (digitalRead(LINE3D) == HIGH ) {
    line[2] = true;
    robot_dir = 0;
    lineflag = true;
  }
  if (digitalRead(LINE4D) == HIGH ) {
    line[3] = true;
    robot_dir = 3.14 / 2;
    lineflag = true;
  }
  if (lineflag = false) {
    return;
  }
  while (digitalRead(INT_5) == HIGH) {
    motorfunction(robot_dir, 30, -gyro);
  }
  lineflag = true;
  return;
}
