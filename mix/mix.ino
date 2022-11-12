//int 整數 //float 小數點(浮點數) char 字元 str 字串 void空的 bool 布林 變數型態
#include"ServoTimer2.h"
//#include <PID_v1.h>
#include <TimerOne.h>
#include "Wire.h"
#include "sensorbar.h"
int trigPin = 7;    // 超音波感測器 Trig腳接 Arduino pin 6
int echoPin = 6;    //超音波感測器 Echo 腳接 Arduino pin 7
long duration,cm ;
float error;
float de=0;//微分項誤差
float ie=0;//積分項誤差
float error_befor[5]={0,0,0,0,0};//前項誤差
float dt =0.1; //單次執行離散週期
float kp=5;
float ki=1.6;
float kd=1.8;
float fix_value =0;//修正量
char cmd[9] ;
ServoTimer2 servo1;
ServoTimer2 servo2;
bool ArrayStatus[8];
uint8_t rawValue;
//int k ;
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
int speed=30;
SensorBar mySensorBar(SX1509_ADDRESS);
void readLine() {
  rawValue = mySensorBar.getRaw();
  delay(10);
}
void forward() { //前
  servo1.write(1536+speed);
  servo2.write(1538-speed);
}

void stop1(){
  servo1.write(1536);
  servo2.write(1538);
}
void fix(float v){
  if(v<-10 && v>-60){
    servo1.write(1536+speed-int(v));
    servo2.write(1538-speed-int(v));
    Serial.print("向右");
  }
  else if (v>10 && v< 60){
    servo1.write(1536+speed-int(v));
    servo2.write(1538-speed-int(v));
    Serial.print("向左");
  }
  else if(v<10 || v<-10){
    forward();
    Serial.print("do forward()");
  }
  /*
  else if(v>60){
    servo1.write(1536+speed-int(v));
    servo2.write(1538-50);
    Serial.print("向右");    
  }
  else if(v<-60){
    servo1.write(1536+50);
    servo2.write(1538-speed-int(v));
    Serial.print("向左");
   
  }
   */
  Serial.println(fix_value);
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
  Timer1.initialize(100000);
  Timer1.attachInterrupt(PID); 
}
void loop(){
  readLine();//讀線狀態，回傳8個位元的值
  //Serial.println(rawValue);
  rawValue=mySensorBar.getRaw();//.getPosition();
  
}
void PID(){
  if(rawValue!=0&& rawValue!=255){
  error = log(rawValue)/log(2)-4.58;
  calculation(error);
  fix(fix_value);
  }
  else if(rawValue ==0 || rawValue ==255){
    stop1();
  }
}
float buffer_right_move(float *buffer , int buf_len)
{
	int i ;
	char tmp = buffer[buf_len - 1];
	for(i = buf_len ; i > 0 ; i--)
	{
		buffer[i] = buffer[i-1] ; 
	}
	buffer[0] = error ;
}

void calculation(float cerror){
  ie = ie+(error*dt);
  /*
  for(int j=0;j<5;j++){
    if(j<4)
    de = de+((error_befor[j]-error_befor[j+1])/dt);
    else
    de = de+(error_befor[j]-error_befor[0])/dt;
    de = de;
  }
  */
  de = (cerror-error_befor[0])/dt;
  error_befor[0]=cerror;
  //buffer_right_move(error_befor,5) ; 

  fix_value = kp*cerror+ki*ie+kd*de;
  /*
  if(fix_value>70)
  fix_value=70;
  else if(fix_value<-70)
  fix_value=-70;
  */
}
