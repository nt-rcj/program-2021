/*
   ロボット側(SofrwareSerial)から来た
   データをそのままxbee(HardwareSerial)
   に送り出す。
   23.nov.2020 by H.Saeki
*/

#include <SoftwareSerial.h>
SoftwareSerial xbeeSerial(10, 11); // メインCPUに繋がるシリアルポート

int led = 4;
uint8_t me = 0;
uint8_t you = 0;
void setup() {
  Serial.begin(9600); // Xbee向けのシリアルポートを初期化
  xbeeSerial.begin(9600); //　メインCPU向けのシリアルポートを初期化

  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  //Serial.println(" Ready!");  // Xbeeに文字を出力
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);
  digitalWrite(led, LOW);   // turn the LED off (LOW is the voltage level)
  xbeeSerial.listen();  // メインCPUからのデータを受信する

}

void loop() {
  if (xbeeSerial.available() > 0) {
    me =  xbeeSerial.read();
    Serial.write(me);  // メインCPUから来たデータをXbeeに送り出す。
  }
  if (Serial.available() > 0) {
    you = Serial.read();
    if (me <= you) {
      //attacker
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      xbeeSerial.write("1");
      digitalWrite(led, LOW);   // turn the LED off (LOW is the voltage level)
    } else {
      //keeper
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      xbeeSerial.write("2");
      digitalWrite(led, LOW);   // turn the LED off (LOW is the voltage level)
    }
  }
}
