//int 整數 //float 小數點(浮點數) char 字元 str 字串 void空的 bool 布林 變數型態
#include <Servo.h>
#include <PID_v1.h>
#include "Wire.h"
#include "sensorbar.h"
int trigPin = 7;    // 超音波感測器 Trig腳接 Arduino pin 6
int echoPin = 6;    //超音波感測器 Echo 腳接 Arduino pin 7
long duration,cm ;

char cmd[9] ;
Servo servo1;
Servo servo2;
bool ArrayStatus[8];
int8_t rawValue;
//int k ;
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
int speed=30;
SensorBar mySensorBar(SX1509_ADDRESS);
void readLine() {
  rawValue = mySensorBar.getRaw();
  delay(10);
}
void forward() { //前
  
  servo1.writeMicroseconds(1536+speed);
  servo2.writeMicroseconds(1538-speed);
}
void L1() { 
  servo1.writeMicroseconds(1536+speed);
  servo2.writeMicroseconds(1538-speed-20);
}
void L2() {  
  servo1.writeMicroseconds(1536+speed);
  servo2.writeMicroseconds(1538-speed-40);
}
void L3() {  
  servo1.writeMicroseconds(1536+speed);
  servo2.writeMicroseconds(1538-speed-50);
}
void L4() {  
  servo1.writeMicroseconds(1536+speed);
  servo2.writeMicroseconds(1538-speed-60);
}
void L5() {  
  servo1.writeMicroseconds(1536+speed);
  servo2.writeMicroseconds(1538-speed-70);
}
void L6() {  
  servo1.writeMicroseconds(1536+speed);
  servo2.writeMicroseconds(1538-speed-90);
}
void R1() {  //右
  servo1.writeMicroseconds(1536+speed+20);
  servo2.writeMicroseconds(1538-speed);
}
void R2() {  //慢右
  servo1.writeMicroseconds(1536+speed+40);
  servo2.writeMicroseconds(1538-speed);
}
void R3() {
  servo1.writeMicroseconds(1536+speed+50);
  servo2.writeMicroseconds(1538-speed);
}
void R4() {
  servo1.writeMicroseconds(1536+speed+60);
  servo2.writeMicroseconds(1538-speed);
}
void R5() {
  servo1.writeMicroseconds(1536+speed+70);
  servo2.writeMicroseconds(1538-speed);
}
void R6() {
  servo1.writeMicroseconds(1536+speed+90);
  servo2.writeMicroseconds(1538-speed);
}
void stop1(){
  servo1.writeMicroseconds(1536);
  servo2.writeMicroseconds(1538);
}

void setup() {
  servo1.attach(11);
  servo2.attach(13);
  Serial.begin(115200);  // start serial for output
  Serial.println("Program started.");
  Serial.println();
  pinMode(trigPin, OUTPUT);      //Arduino 對外啟動距離感測器Trig腳，射出超音波 
  pinMode(echoPin, INPUT);       //超音波被障礙物反射後，Arduino讀取感測器Echo腳的時間差
  uint8_t returnStatus = mySensorBar.begin();
  mySensorBar.clearInvertBits();
  mySensorBar.setInvertBits();
}
void loop(){
  
  readLine();//讀線狀態，回傳8個位元的值
//  Serial.println(rawValue);
  rawValue=mySensorBar.getPosition();

  Serial.println(rawValue);
  switch(rawValue){
  case 0:
    forward();
    Serial.println("0");
    break;
  case 4:
    L1();
    break;
  case 15:
    L2();
    break;
  case 9:
    L3();
    break;
  case 26:
    L4();
    break;
  case 13:
    L5();
    break;
  case 37:
    L6();
    break;
  case -4:
    R1();
    break;
  case -15:
    R2();
    break;
  case -9:
    R3();
    break;
  case -26:
    R4();
    break;
  case -13:
    R5();
    break;
  case -37:
    R6();
    break;
  default:
    stop1();
    break;
  }
/*
  digitalWrite(trigPin, LOW); //程式計算出距離值 cm
  delayMicroseconds(1000);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(1000);
  digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  cm = (duration/2) / 29.1;  
*/
}
