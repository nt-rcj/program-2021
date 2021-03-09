#include <SoftwareSerial.h>
SoftwareSerial xbeeSerial(10, 11); //メインCPUに繋がるシリアルポート

int led = 4;
void setup(){
    Serial.begin(9600); //Xbee向けのシリアルポートを初期化
    xbeeSerial.begin(9600); //メインCPU向けのシリアルポートを初期化

    // initalize the digital pin as an output.
    pinMode(led, OUTPUT);
    Serial.println(" Ready!"); //Xbeeに文字出力
    digitalWrite(led, HIGH); //turn the LED on
    delay(100);
    digitalWrite(led, LOW); //turn the LED off
}

void loop(){
    char buf[4];
    xbeeSerial.listen();
    if(xbeeSerial.available() > 0){
        digitalWrite(led, HIGH);
        Serial.write(xbeeSerial.read()); //メインCPUから来たデータをXbeeに送り出す
        digitalWrite(led, LOW);
    }
}