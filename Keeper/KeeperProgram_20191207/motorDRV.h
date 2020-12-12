//
// FourWheel Motor control module
//           Ver 2.4 Dec.05.2019
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

//  変数directの内容(0-7)に従って8方向に移動するプログラム

void motorFoward(int direct, int power) {
  int Negpower;
  int m_power[4];

  Negpower = -power;
  if ((direct > 7) | (direct < 0))
    return;
  switch (direct) {
    case 0:
      m_power[0] = power; m_power[1] = power; m_power[2] = Negpower; m_power[3] = Negpower;
      break;
    case 1:
      m_power[0] = 0;  m_power[1] = power; m_power[2] = 0; m_power[3] = Negpower;
      break;
    case 2:
      m_power[0] = Negpower; m_power[1] = power; m_power[2] = power; m_power[3] = Negpower;
      break;
    case 3:
      m_power[0] = Negpower; m_power[1] = 0; m_power[2] = power; m_power[3] = 0;
      break;
    case 4:
      m_power[0] = Negpower; m_power[1] = Negpower; m_power[2] = power; m_power[3] = power;
      break;
    case 5:
      m_power[0] = 0; m_power[1] = Negpower; m_power[2] = 0; m_power[3] = power;
      break;
    case 6:
      m_power[0] = power; m_power[1] = Negpower; m_power[2] = Negpower; m_power[3] = power;
      break;
    case 7:
      m_power[0] = power; m_power[1] = 0; m_power[2] = Negpower;  m_power[3] = 0;
    default:
      break;
  };
  motorCh1(m_power[0]); motorCh2(m_power[1]); motorCh3(m_power[2]); motorCh4(m_power[3]);
  return;
}

void motorFowardRotate(int direct, int power, int rotation) {
  int m_power[4];
  float fm_power;
  int i, Negpower, max_power, abs_power;

  if (power > 100)    // powerを±100以下にする
    power = 100;
  else if (power < -100)
    power = -100;

  rotation = -rotation;  // CW(時計回り)を正方向とする
  Negpower = -power;

  abs_power = abs(power);   // abs(rotation) < abs(power) にする。
  if ( abs(rotation) > abs_power )
    if ( rotation >= 0 )
      rotation = abs_power;
    else
      rotation = -abs_power;

  if ( direct != -1 ) {     // if direct == -1 -> Simply Rotate!
    if ((direct > 7) | (direct < 0))
      return;
    switch (direct) {
      case 0:
        m_power[0] = power; m_power[1] = power; m_power[2] = Negpower; m_power[3] = Negpower;
        break;
      case 1:
        m_power[0] = 0;	m_power[1] = power; m_power[2] = 0; m_power[3] = Negpower;
        break;
      case 2:
        m_power[0] = Negpower; m_power[1] = power; m_power[2] = power; m_power[3] = Negpower;
        break;
      case 3:
        m_power[0] = Negpower; m_power[1] = 0; m_power[2] = power; m_power[3] = 0;
        break;
      case 4:
        m_power[0] = Negpower; m_power[1] = Negpower; m_power[2] = power; m_power[3] = power;
        break;
      case 5:
        m_power[0] = 0; m_power[1] = Negpower; m_power[2] = 0; m_power[3] = power;
        break;
      case 6:
        m_power[0] = power; m_power[1] = Negpower; m_power[2] = Negpower; m_power[3] = power;
        break;
      case 7:
        m_power[0] = power; m_power[1] = 0;	m_power[2] = Negpower; 	m_power[3] = 0;
      default:
        break;
    }
  } else {
    for (i = 0; i < 4; i++)
      m_power[i] = 0;
  }
  for (i = 0; i < 4; i++)    // モーターに回転を与える。
    m_power[i] = m_power[i] + rotation;

  max_power = 0;
  for (i = 0; i < 4; i++) {   //  m_powerを100以下に正規化する。
    if (abs(m_power[i]) > max_power)
      max_power = abs(m_power[i]);
  };
  if (max_power > 100) {
    fm_power = ((float)max_power) / 100.0;
    for (i = 0; i < 4; i++)
      m_power[i] = (int)((float)m_power[i] / fm_power);
  }

  motorCh1(m_power[0]); motorCh2(m_power[1]); motorCh3(m_power[2]); motorCh4(m_power[3]);

  //  Serial.print(" ch1=");
  //  Serial.print(m_power[0]);
  //  Serial.print(" ch2=");
  //  Serial.print(m_power[1]);
  //  Serial.print(" ch3=");
  //  Serial.print(m_power[2]);
  //  Serial.print(" ch4=");
  //  Serial.println(m_power[3]);

  return;
}

void motorfunction(float z, int power) {
  static float x1,x2,x3,x4;

  x1=-(sin(z-3.14/4.0)*power);
  x2=-(sin(z-3*3.14/4.0)*power);
  x3=-(sin(z-5*3.14/4.0)*power);
  x4=-(sin(z-7*3.14/4.0)*power);

    //Serial.print(" x1=");
    //Serial.print(x1);
    //Serial.print(" x2=");
    //Serial.print(x2);
    //Serial.print(" x3=");
    //Serial.print(x3);
    //Serial.print(" x4=");
    //Serial.println(x4);


  motorCh1(x1);
  motorCh2(x2);
  motorCh3(x3);
  motorCh4(x4);
}

void motorfunctionminus(float z, int power) {

  static float x1,x2,x3,x4;

  x1=-(sin(z-3.14/4.0)*power);
  x2=-(sin(z-3*3.14/4.0)*power);
  x3=-(sin(z-5*3.14/4.0)*power);
  x4=-(sin(z-7*3.14/4.0)*power);

    //Serial.print(" x1=");
    //Serial.print(x1);
    //Serial.print(" x2=");
    //Serial.print(x2);
    //Serial.print(" x3=");
    //Serial.print(x3);
    //Serial.print(" x4=");
    //Serial.println(x4);


  motorCh1(-x1);
  motorCh2(-x2);
  motorCh3(-x3);
  motorCh4(-x4);
}

//  時計回りに回転する

void turnCW(int power) {
  motorCh1(-power); motorCh2(-power); motorCh3(-power); motorCh4(-power);
}

//  反時計回りに回転する

void turnCCW(int power) {
  motorCh1(power); motorCh2(power); motorCh3(power); motorCh4(power);
}
