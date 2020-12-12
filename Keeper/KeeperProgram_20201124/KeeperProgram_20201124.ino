//
// Robot main program
//           Ver 1.0 Dece.19.2019
//           By Hiroya Hashimoto
//

//Motor control program
#include "motorDRV4.h" //  モーター制御のプログラムを読み込む
#include "NT_Robot202011.h" //Header file for Teensy 3.5

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
  pinMode(INT_29, INPUT);          // interrupt port set

  motorInit();  //  モーター制御の初期化
  power = 10;   //  set initial motor power

  Serial3.begin(19200);  // initialize serialport foe openMV
  Serial2.begin(9600); // KemarinTech Gyro senser

  lineflag = false;   //reset outofbounds　flag
  for ( i = 0; i < 4; i++ )
    line[i] = false;
  robot_dir = 0;  //reset robot direction

  // Caution D18 -> Interrupt5

  attachInterrupt(INT_29, intHandle, RISING);

  Serial.begin(9600);   //  シリアル出力を初期化
  Serial.print("Starting...\n");

  //digitalWrite(Kick1, HIGH);
  //delay(100);
  //digitalWrite(Kick1, LOW);
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
  char buf[64];
  float m , z;
  float x, y;
  float bg_x, bg_y;
  float yg_x, yg_y;
  float goal_x, goal_y;
  float y_sig,b_sig,goal_sig;
  float az, AZ, d, k;
  float targetP, distance, pointP;
  float goal_dist;
  float angle, inroot;
  float divergence, RtoBdist;
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

    if(digitalRead(GoalSW)) {
    	goal_sig=b_sig;
    	goal_x = bg_x;
    	goal_y = bg_y;
    }else{
    	goal_sig=y_sig;
    	goal_x = yg_x;
    	goal_y = yg_y;
    }

    goal_x = goal_x - 155 ;
    goal_y = goal_y - 162 ;    

    k = 0.01;
    angle = atan(goal_x / goal_y);
    goal_dist = sqrt(goal_x * goal_x + goal_y * goal_y);

    // Convert coordinates data
    if (sig != 0) {
      x = 164 - x;
      y = 61 - y;
      ball_y = goal_y + y;
    }

    RtoBdist = y * 150/42;

        if(RtoBdist > 150){
      RtoBdist = 150;
    }else if(RtoBdist < 0){
      RtoBdist = 0;
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
    Serial.print(" x=");
    Serial.print(x);
    Serial.print(" RtoBdist=");
    Serial.print(RtoBdist);
    Serial.println();

    Serial.print(gyro);
    Serial.println();


  // check line and reverse
  if (digitalRead(StartSW)) {
    digitalWrite(LINE_LED, HIGH); // ラインセンサのLEDを点灯

    if (abs(gyro) <= 7) {
     	digitalWrite(LED_BUILTIN, LOW);
     	if(sig == 0){
     		divergence = 0;
     	}else{
    		if(abs(x) <= 7){
    			divergence = 0;
      		}else if(x > 0){
      			distance = 300 + RtoBdist;
      			AZ = PI/2.0;
      			divergence = 1;
      		}else{
      			distance = 300 + RtoBdist;
      			AZ = -PI/2.0;
      			divergence = 1;
      		}
      	}
      	if(divergence == 1){
			targetP = distance / cosf(angle);
			inroot = sqrt(targetP * targetP + 33310);
			pointP = targetP * 6866 / (15850 + 72.01 * inroot);

		    d = k*(pointP - goal_dist);
    		if(d < -1.3){
    		  d = -1.3;
    		}else if(d > 1.3){
	    	  d = 1.3;
    		}

    		if(AZ > 0){
       		 	az = AZ - d;
        		motorfunction(az, 30, 0);
    		}else{
        		az = AZ + d;
        		motorfunction(az, 30, 0);
      		}
		}else{
			motorfunction(0, 0, 0);
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

// interrupt handler

void intHandle() {  // Lineを踏んだらlineflagをセットして止まる。
  int i, power, back_dir;
  float azimuth;

  if ( !digitalRead(StartSW))  // スイッチがOFFなら何もしない。
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

  if (lineflag = false)     // センサーの反応がない場合は何もしない
    return;
  lineflag = true;          // set lineflag
  motorStop();                  // ラインから外れたらモーターstop
  return;
}

void back_Line1(int power) {             // Lineセンサ1が反応しなくなるまで後ろに進む
  float azimuth;
  digitalWrite(LED_R, HIGH);  // LED_R点灯
  while ((digitalRead(LINE1D) == HIGH) || (digitalRead(LINE5D) == HIGH) || (digitalRead(LINE3D) == HIGH)) {
    azimuth = 4 * 3.14159 / 4.0;   // 後ろ方向(4)をradianに変換
    motorfunction(azimuth, power, 0);// back_dirの方向に進ませる
  }
  digitalWrite(LED_R, LOW);  // LED_R消灯
  motorStop();
}

void back_Line2(int power) {             // Lineセンサ2が反応しなくなるまで左に進む
  float azimuth;
  digitalWrite(LED_Y, HIGH);  // LED_Y点灯
  while ((digitalRead(LINE2D) == HIGH) || (digitalRead(LINE5D) == HIGH) || (digitalRead(LINE4D) == HIGH)) {
    azimuth = 6 * 3.14159 / 4.0;   // 左方向(6)をradianに変換
    motorfunction(azimuth, power, 0);// azimuthの方向に進ませる
  }
  digitalWrite(LED_Y, LOW);  // LED_Y消灯
  motorStop();
}

void back_Line3(int power) {             // Lineセンサ3が反応しなくなるまで前に進む
  float azimuth;
  digitalWrite(LED_G, HIGH);  // LED_G点灯
  while ((digitalRead(LINE3D) == HIGH) || (digitalRead(LINE5D) == HIGH) || (digitalRead(LINE1D) == HIGH)) {
    azimuth = 0 * 3.14159 / 4.0;   // 前方向(0)をradianに変換
    motorfunction(azimuth, power, 0);// azimuthの方向に進ませる
  }
  digitalWrite(LED_G, LOW);  // LED_G消灯
  motorStop();
}

void back_Line4(int power) {             // Lineセンサ4が反応しなくなるまで右に進む
  float azimuth;
  digitalWrite(LED_B, HIGH);  // LED_B点灯
  while ((digitalRead(LINE4D) == HIGH) || (digitalRead(LINE5D) == HIGH) || (digitalRead(LINE2D) == HIGH)) {
    azimuth = 2 * 3.14159 / 4.0;   // 右方向()をradianに変換
    motorfunction(azimuth, power, 0);// azimuthの方向に進ませる
  }
  digitalWrite(LED_B, LOW);  // LED_B消灯
  motorStop();
}
