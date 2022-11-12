//int 整數 //float 小數點(浮點數) char 字元 str 字串 void空的 bool 布林 變數型態
#include <Servo.h>
#include <PID_v1.h>
#include "Wire.h"
#include "sensorbar.h"

int trigPin = 7;    // 超音波感測器 Trig腳接 Arduino pin 6
int echoPin = 6;    //超音波感測器 Echo 腳接 Arduino pin 7
long duration,cm ;

double Setpoint, Input, Output;
int y =0;
//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=4, consKi=0.05, consKd=0.05;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, P_ON_M);

char cmd[9] ;
Servo servo1;
Servo servo2;
bool ArrayStatus[8];
int8_t rawValue;
//int k ;
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)

SensorBar mySensorBar(SX1509_ADDRESS);
void readLine() {
  rawValue = mySensorBar.getRaw();
  delay(10);
}
void forword() { //前
  servo1.write(1300);
  servo2.write(1700);
}
void turnLeft() { //左
  servo1.write(1480);
  servo2.write(1533);
}
void turnLeftslow() {  //慢左
  servo1.write(1500);
  servo2.write(1533);
}
void turnRight() {  //右
  servo1.write(1533);
  servo2.write(1588);
}
void turnRightslow() {  //慢右
  servo1.write(1533);
  servo2.write(1480);
}
void turnLeft1() {
  servo1.write(1700);
  servo2.write(1300);
}
void stop1(){
  servo1.write(1433);
  servo2.write(1604);
}

void setup() {
  
  servo1.attach(10);
  servo2.attach(11);
  Setpoint = 0;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-127, 127);
  Serial.begin(9600);  // start serial for output
  Serial.println("Program started.");
  Serial.println();

  pinMode(trigPin, OUTPUT);      //Arduino 對外啟動距離感測器Trig腳，射出超音波 
  pinMode(echoPin, INPUT);       //超音波被障礙物反射後，Arduino讀取感測器Echo腳的時間差

//  mySensorBar.setBarStrobe();
  
  uint8_t returnStatus = mySensorBar.begin();
  mySensorBar.clearInvertBits();
  mySensorBar.setInvertBits();
}
void loop(){
  
  readLine();//讀線狀態，回傳8個位元的值
//  Serial.println(rawValue);
  rawValue=mySensorBar.getPosition();
  Input=rawValue;
//  rawValue=0;
  Serial.print(Output);
  Serial.print("\t");
  Serial.println(mySensorBar.getPosition());

  digitalWrite(trigPin, LOW); //程式計算出距離值 cm
  delayMicroseconds(1000);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(1000);
  digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  cm = (duration/2) / 29.1;  
  Serial.print(cm);     //印出距離值 cm 在序列埠監控顯示器 單位公分
  Serial.println(" cm");
  
//  servo1.write(1300);
//  servo2.write(1700);

  if (cm <= 10) {  //避障
   turnLeft(); //左
   delay(800);
   forword();  //直
   delay(1000);
   turnRight(); //右
   delay(1800);
   forword();  //直
   delay(1000);
   turnLeft();  //左
   delay(800);
   forword();  //直
   delay(1000);
  
  }
  else if (rawValue < -35){
//    turnLeft();
     int x1 = map(Output, 0, -127, 1500, 1700);
     
     servo1.write(1450);
     servo2.write(x1);
     

  }
 else if (rawValue > 35){
//  turnRight();
     int y1 = map(Output, 0, 127, 1500, 1300);
     servo1.write(y1);
     servo2.write(1550);
 }
 else{
  servo1.write(1300);
  servo2.write(1700);
 }
 myPID.Compute();
}
