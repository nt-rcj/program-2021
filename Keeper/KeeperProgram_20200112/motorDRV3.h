//
// FourWheel Motor control module
//           Ver 3.1 Jan.02.2020
//           By Saeki Akitoshi
//           By Saeki Hidekazu
// for Arduino MEGA2560
//モーターを動かすサブプログラム
//

//  IOピンを設定する

const int CH1DIR1   = 30;
const int CH1DIR2   = 31;
const int CH2DIR1   = 32;
const int CH2DIR2   = 33;
const int CH3DIR1   = 34;
const int CH3DIR2   = 35;
const int CH4DIR1   = 36;
const int CH4DIR2   = 37;
const int CHADIR1   = 26;
const int CHADIR2   = 27;
const int CHBDIR1   = 28;
const int CHBDIR2   = 29;

const int CH1PWM    =  2;
const int CH2PWM    =  3;
const int CH3PWM    =  5;
const int CH4PWM    =  6;
const int CHAPWM    =  7;
const int CHBPWM    =  8;


void motorCh1(int data) {   //CH1のモーターを動かすプログラム
  int power;
  if ( data == 0 ) {         // data == 0　なら停止
    analogWrite(CH1PWM, 0);  // power = 0
    digitalWrite(CH1DIR1, HIGH); // Brake
    digitalWrite(CH1DIR2, HIGH);
    return;
  }
  if (data > 0) {                //  data > 0 なら正転、data < 0 なら逆転
    digitalWrite(CH1DIR1, HIGH); // Forward
    digitalWrite(CH1DIR2, LOW);
  } else {
    digitalWrite(CH1DIR1, LOW);  // Reverse
    digitalWrite(CH1DIR2, HIGH);
  }
  power = abs(data);            //  -100～+100のデータを0～250に変換する
  if (power > 100)              //  data > 100 の場合は100にする
    power = 100;
  power = (power << 1) + (power >> 1); // power=power*2.5
  analogWrite(CH1PWM, power); // 0 < power < 250
}

void motorCh2(int data) {   //CH2のモーターを動かすプログラム
  int power;
  if ( data == 0 ) {     // if Brake
    analogWrite(CH2PWM, 0);  // power = 0
    digitalWrite(CH2DIR1, HIGH); // Brake
    digitalWrite(CH2DIR2, HIGH);
    return;
  }
  if (data > 0) {
    digitalWrite(CH2DIR1, HIGH); // Forward
    digitalWrite(CH2DIR2, LOW);
  } else {
    digitalWrite(CH2DIR1, LOW); // Reverse
    digitalWrite(CH2DIR2, HIGH);
  }
  power = abs(data);
  if (power > 100)
    power = 100;
  power = (power << 1) + (power >> 1); // power=power*2.5
  analogWrite(CH2PWM, power); // 0 < power < 250
}

void motorCh3(int data) {   //CH3のモーターを動かすプログラム
  int power;
  if ( data == 0 ) {     // if Brake
    analogWrite(CH3PWM, 0);  // power = 0
    digitalWrite(CH3DIR1, HIGH); // Brake
    digitalWrite(CH3DIR2, HIGH);
    return;
  }
  if (data > 0) {
    digitalWrite(CH3DIR1, HIGH); // Forward
    digitalWrite(CH3DIR2, LOW);
  } else {
    digitalWrite(CH3DIR1, LOW); // Reverse
    digitalWrite(CH3DIR2, HIGH);
  }
  power = abs(data);
  if (power > 100)
    power = 100;
  power = (power << 1) + (power >> 1); // power=power*2.5
  analogWrite(CH3PWM, power); // 0 < power < 250
}

void motorCh4(int data) {   //CH4のモーターを動かすプログラム
  int power;
  if ( data == 0 ) {     // if Brake
    analogWrite(CH4PWM, 0);  // power = 0
    digitalWrite(CH4DIR1, HIGH); // Brake
    digitalWrite(CH4DIR2, HIGH);
    return;
  }
  if (data > 0) {
    digitalWrite(CH4DIR1, HIGH); // Forward
    digitalWrite(CH4DIR2, LOW);
  } else {
    digitalWrite(CH4DIR1, LOW); // Reverse
    digitalWrite(CH4DIR2, HIGH);
  }
  power = abs(data);
  if (power > 100)
    power = 100;
  power = (power << 1) + (power >> 1); // power=power*2.5
  analogWrite(CH4PWM, power); // 0 < power < 250
}

void motorChA(int data) {   //CHA(5)のモーターを動かすプログラム
  int power;
  if ( data == 0 ) {     // if Brake
    analogWrite(CHAPWM, 0);  // power = 0
    digitalWrite(CHADIR1, HIGH); // Brake
    digitalWrite(CHADIR2, HIGH);
    return;
  }
  if (data > 0) {
    digitalWrite(CHADIR1, HIGH); // Forward
    digitalWrite(CHADIR2, LOW);
  } else {
    digitalWrite(CHADIR1, LOW); // Reverse
    digitalWrite(CHADIR2, HIGH);
  }
  power = abs(data);
  if (power > 100)
    power = 100;
  power = (power << 1) + (power >> 1); // power=power*2.5
  analogWrite(CHAPWM, power); // 0 < power < 250
}

void motorChB(int data) {   //CHB(6)のモーターを動かすプログラム
  int power;
  if ( data == 0 ) {     // if Brake
    analogWrite(CHBPWM, 0);  // power = 0
    digitalWrite(CHBDIR1, HIGH); // Brake
    digitalWrite(CHBDIR2, HIGH);
    return;
  }
  if (data > 0) {
    digitalWrite(CHBDIR1, HIGH); // Forward
    digitalWrite(CHBDIR2, LOW);
  } else {
    digitalWrite(CHBDIR1, LOW); // Reverse
    digitalWrite(CHBDIR2, HIGH);
  }
  power = abs(data);
  if (power > 100)
    power = 100;
  power = (power << 1) + (power >> 1); // power=power*2.5
  analogWrite(CHBPWM, power); // 0 < power < 250
}

void motorInit() {    // Arduinoのモーター制御を初期化する
  pinMode(CH1DIR1, OUTPUT);
  pinMode(CH1DIR2, OUTPUT);
  pinMode(CH2DIR1, OUTPUT);
  pinMode(CH2DIR2, OUTPUT);
  pinMode(CH3DIR1, OUTPUT);
  pinMode(CH3DIR2, OUTPUT);
  pinMode(CH4DIR1, OUTPUT);
  pinMode(CH4DIR2, OUTPUT);
  pinMode(CHADIR1, OUTPUT);
  pinMode(CHADIR2, OUTPUT);
  pinMode(CHBDIR1, OUTPUT);
  pinMode(CHBDIR2, OUTPUT);

  pinMode(CH1PWM, OUTPUT);
  pinMode(CH2PWM, OUTPUT);
  pinMode(CH3PWM, OUTPUT);
  pinMode(CH4PWM, OUTPUT);
  pinMode(CHAPWM, OUTPUT);
  pinMode(CHBPWM, OUTPUT);

  TCCR3B &= B11111000;
  TCCR3B |= B00000001; //set PWM 31kHz
  TCCR4B &= B11111000;
  TCCR4B |= B00000001; //set PWM 31kHz

  motorCh1(0);  // Set All motors STOP
  motorCh2(0);
  motorCh3(0);
  motorCh4(0);
  motorChA(0);
  motorChB(0);
}

//  ドリブラモーターを回すプログラム

void dribbler(int power) {
  motorChA(power);
  return;
}

// motor control function
// z:azimuth(radian)
// p:power(-100:100)
// rotation :rotation(-100:100)
//

void motorfunction(float z, int power, int rotation) {
  float x[4], x_max, w;
  int i;

  if (power > 100)    // powerを±100以下にする
    power = 100;
  else if (power < -100)
    power = -100;
  if (rotation > 100)    // rotationを±100以下にする
    rotation = 100;
  else if (rotation < -100)
    rotation = -100;

  x[0] = -(sin(z - 3.14159 / 4.0));
  x[1] = -(sin(z - 3 * 3.14159 / 4.0));
  x[2] = -(sin(z - 5 * 3.14159 / 4.0));
  x[3] = -(sin(z - 7 * 3.14159 / 4.0));

  x_max = 0.0;
  for ( i = 0; i < 4; i++) {
    if ( x_max < abs(x[i]))
      x_max = abs(x[i]);
  };
  for ( i = 0; i < 4; i++)
    x[i] = x[i] / x_max;
  w = -(rotation / 100.0);
  for ( i = 0; i < 4; i++)
    x[i] = x[i] + w;
  x_max = 0.0;
  for ( i = 0; i < 4; i++) {
    if ( x_max < abs(x[i]))
      x_max = abs(x[i]);
  };
  for ( i = 0; i < 4; i++)
    x[i] = x[i] / x_max;
  for ( i = 0; i < 4; i++)
    x[i] = x[i] * power;
  /*
    Serial.print(" Z=");
    Serial.print(z);
    Serial.print(" x[0]=");
    Serial.print(int(x[0]));
    Serial.print(" x[1]=");
    Serial.print(int(x[1]));
    Serial.print(" x[2]=");
    Serial.print(int(x[2]));
    Serial.print(" x[3]=");
    Serial.print(int(x[3]));
  */
  motorCh1(x[0]);
  motorCh2(x[1]);
  motorCh3(x[2]);
  motorCh4(x[3]);
}

//  時計回りに回転する

void turnCW(int power) {
  motorCh1(-power); motorCh2(-power); motorCh3(-power); motorCh4(-power);
}

//  反時計回りに回転する

void turnCCW(int power) {
  motorCh1(power); motorCh2(power); motorCh3(power); motorCh4(power);
}

//  モーターを止める(Brake)

void motorStop() {
  motorCh1(0); motorCh2(0); motorCh3(0); motorCh4(0);
}

//  モーターを止める(Free)

void motorFree() {

  digitalWrite(CH1PWM, HIGH);  // power = 0
  digitalWrite(CH1DIR1, LOW);  // Off
  digitalWrite(CH1DIR2, LOW);

  digitalWrite(CH2PWM, HIGH);  // power = 0
  digitalWrite(CH2DIR1, LOW);  // Off
  digitalWrite(CH2DIR2, LOW);

  digitalWrite(CH3PWM, HIGH);  // power = 0
  digitalWrite(CH3DIR1, LOW);  // Off
  digitalWrite(CH3DIR2, LOW);

  digitalWrite(CH4PWM, HIGH);  // power = 0
  digitalWrite(CH4DIR1, LOW);  // Off
  digitalWrite(CH4DIR2, LOW);

}
