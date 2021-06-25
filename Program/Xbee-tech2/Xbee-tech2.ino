/*
  RCJ 2021 WORLDWIDE
  Technical Challenge 2
  Team NT
  ロボット側(SofrwareSerial)から来た
  データをそのままxbee(HardwareSerial)
  に送り出す。
  25.june.2021 by A.Saeki
*/

#include <SoftwareSerial.h>
SoftwareSerial xbeeSerial(10, 11); // メインCPUに繋がるシリアルポート

int led = 4;
uint8_t mydata = 0;
uint8_t partnerdata = 0;

void setup() {
  Serial.begin(9600); // Xbee向けのシリアルポートを初期化
  xbeeSerial.begin(9600); //　メインCPU向けのシリアルポートを初期化

  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  Serial.println(" Ready!");  // Xbeeに文字を出力
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);
  digitalWrite(led, LOW);   // turn the LED off (LOW is the voltage level)
  xbeeSerial.listen();  // メインCPUからのデータを受信する

}

void loop() {
  if (xbeeSerial.available() > 0) { //Is there date from maincpu?
    mydata =  xbeeSerial.read();//xbeeSerial.read = from maincpu
    Serial.write(mydata);  // メインCPUから来たデータをXbeeに送り出す。
  }
  if (Serial.available() > 0) {
    partnerdata = Serial.read();
    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
    xbeeSerial.write(partnerdata);
    digitalWrite(led, LOW);   // turn the LED off (LOW is the voltage level)
  }  
}
