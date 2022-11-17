//int 整數 //float 小數點(浮點數) char 字元 str 字串 void空的 bool 布林 變數型態
#include"ServoTimer2.h"
//#include <PID_v1.h>
#include <math.h>
#include <TimerOne.h>
#include "Wire.h"
#include "sensorbar.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED 寬度像素
#define SCREEN_HEIGHT 64 // OLED 高度像素
// 設定OLED
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int trigPin = 7;    // 超音波感測器 Trig腳接 Arduino pin 6
int echoPin = 6;    //超音波感測器 Echo 腳接 Arduino pin 7
long duration,cm ;
float error;
float de=0;//微分項誤差
float ie=0;//積分項誤差
float error_befor[5]={0,0,0,0,0};//前項誤差
float dt =0.1; //單次執行離散週期
int speed=80;
float kp=20.6;
float ki=0.3;
float kd=6;
float fix_value =0;//修正量
char cmd[9] ;
ServoTimer2 servo1;
ServoTimer2 servo2;
uint8_t stage = 0;
uint8_t time_counter=0;
bool ini=true;
bool start =false;
bool ArrayStatus[8];
uint8_t rawValue;
int mode=0;
uint8_t block = 0;
uint8_t part =0;//斷線 共有四段空白
bool befor_online=true;
bool turn_done=false;
int8_t light=0;
//int k ;
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)

SensorBar mySensorBar(SX1509_ADDRESS);
void readLine() {
  rawValue = mySensorBar.getRaw();
}
void forward() { //前
  servo1.write(1536+speed);
  servo2.write(1538-speed);
}

void stop1(){
  servo1.write(1536);
  servo2.write(1538);
}
void R90(int time_counter,int max_time){
  if(time_counter <max_time){
  servo1.write(1536+62);
  servo2.write(1538+62);
  }
  else{
    servo1.write(1536);
    servo2.write(1538);
    time_counter=0;
    turn_done = true;
  }
}
void L90(int time_counter,int max_time){
  if(time_counter <max_time){
  servo1.write(1536-62);
  servo2.write(1538-62);
  }
  else{
    servo1.write(1536);
    servo2.write(1538);
    time_counter =0;
    turn_done = true;
  }
}
void goF(int time_counter,int max_time){
  if(time_counter <max_time){
  servo1.write(1536+speed);
  servo2.write(1538-speed);
  }
  else{
    servo1.write(1536);
    servo2.write(1538);
    time_counter =0;
    turn_done = true;
  }
}
void printf(){
  if(mode ==0){
    display.clearDisplay();
    display.setTextSize(1);             // 設定文字大小
    display.setTextColor(1);        // 1:OLED預設的顏色(這個會依該OLED的顏色來決定)
    display.setCursor(0,0);             // 設定起始座標
    display.print("block ");        // 要顯示的字串
    display.print(block);
    display.setCursor(0,20);             // 設定起始座標
    display.print("part ");        // 要顯示的字串
    display.print(part);        // 要顯示的字串
    display.setCursor(0,40);             // 設定起始座標
    display.print("stage");
    display.print(stage);        // 要顯示的字串
    display.setCursor(45,20);             // 設定起始座標
    display.print("rawValue");
    display.print(rawValue);        // 要顯示的字串
    display.setCursor(52,0);             // 設定起始座標
    display.print("befor ");
    display.print(befor_online);        // 要顯示的字串
    display.setCursor(45,40);             // 設定起始座標
    display.print("timer ");
    display.print(time_counter);        // 要顯示的字串
    display.display();                  // 要有這行才會把文字顯示出來
    }
  if(mode==1){//顯示正在PID
    display.clearDisplay();
    display.setTextSize(4);             // 設定文字大小
    display.setTextColor(1);        // 1:OLED預設的顏色(這個會依該OLED的顏色來決定)
    display.setCursor(0,0);             // 設定起始座標
    display.print(" PID ");        // 要顯示的字串
    display.setCursor(0,40);             // 設定起始座標
    display.print(stage);        // 要顯示的字串
    display.display();                  // 要有這行才會把文字顯示出來
    }
  if(mode==2){//顯示正在直角(左轉)
    display.clearDisplay();
    display.setTextSize(4);             // 設定文字大小
    display.setTextColor(1);        // 1:OLED預設的顏色(這個會依該OLED的顏色來決定)
    display.setCursor(0,0);             // 設定起始座標
    display.print(" TR ");        // 要顯示的字串
    display.setCursor(0,40);             // 設定起始座標
    display.print(stage);        // 要顯示的字串
    display.display();                  // 要有這行才會把文字顯示出來
    }
  if(mode==3){//顯示正在直角(右轉)
    display.clearDisplay();
    display.setTextSize(4);             // 設定文字大小
    display.setTextColor(1);        // 1:OLED預設的顏色(這個會依該OLED的顏色來決定)
    display.setCursor(0,0);             // 設定起始座標
    display.print(" TL ");        // 要顯示的字串
    display.setCursor(0,40);             // 設定起始座標
    display.print(stage);        // 要顯示的字串
    display.display();                  // 要有這行才會把文字顯示出來
    }
    /*
    Serial.print("if block ");
    Serial.print(block);
    Serial.print("part ");
    Serial.print(part);
    Serial.print(" stage ");
    Serial.print(stage);
    Serial.print(" ie ");
    Serial.print(ie);
    Serial.print(" de ");
    Serial.print(de);
    Serial.print(" error ");
    Serial.print(error);
    Serial.print(" fix_value ");
    Serial.println(fix_value);
    */
}

void fix(float v){
  if(v<-10){
    servo1.write(1536+speed-int(v/2)-int(error*speed/4));
    servo2.write(1538-speed-int(v));
  }
  else if (v>10 ){
    servo1.write(1536+speed-int(v));
    servo2.write(1538-speed-int(v/2)-int(error*speed/4));
  }
  else if(v<10 || v<-10){
    servo1.write(1536+speed);
  servo2.write(1538-speed);
  }
  //Serial.print("rawValue = ");  
  //Serial.println(rawValue); 
}
void setup() {
  servo1.attach(11);
  servo2.attach(13);
  pinMode(A2,INPUT);
  stop1();
  Serial.begin(115200);  // start serial for output
  Serial.println("Program started.");
  Serial.println();
  pinMode(trigPin, OUTPUT);      //Arduino 對外啟動距離感測器Trig腳，射出超音波 
  pinMode(echoPin, INPUT);       //超音波被障礙物反射後，Arduino讀取感測器Echo腳的時間差
  uint8_t returnStatus = mySensorBar.begin();
  mySensorBar.clearInvertBits();
 
   if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 一般1306 OLED的位址都是0x3C
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  // 顯示Adafruit的LOGO，算是開機畫面
  display.display();
  delay(1000); // 停1秒
  // 清除畫面
  display.clearDisplay();
  testdrawstyles();    // 測試文字
  Timer1.initialize(100000);
  Timer1.attachInterrupt(PID); 
}
void loop(){
  readLine();//讀線狀態，回傳8個位元的值
  //Serial.println(rawValue);
  
  light = analogRead(A2);
  Serial.println(light);
  //rawValue=mySensorBar.getRaw();//.getPosition();
  printf();
  
}
void testdrawstyles(void) {
  display.clearDisplay();
  display.setTextSize(1.5);             // 設定文字大小
  display.setTextColor(1);        // 1:OLED預設的顏色(這個會依該OLED的顏色來決定)
  display.setCursor(0,0);             // 設定起始座標
  display.print("Test Display");        // 要顯示的字串
  delay(1000);
}
void PID(){
  
  // 屏蔽測試
  if(light<15 && ini==true){//停在起跑線上的條件
    block = 1;
    time_counter++;//每跳1=0.1sec
    stop1();
    if(time_counter==10)
    {
      stage=1;//跳掉下一階段
      time_counter =0;//計時歸零
      ini =false;//初始狀態切掉
    }
  }  
  if (stage==1 && ini==false && start==false){//起跑
    time_counter++;
    block=2;
    forward();
    if(time_counter==10){
      time_counter =0;//歸零
      start=true;//起跑功能已使用過
    } 
  } 
  if(stage==1 && ini==false && start==true && rawValue!=0){//進入循跡模式
    block=3;
    mode =1;
    calculation();
    fix(fix_value); 
  }
  if(rawValue ==0 && stage==1 && start){//進入第二階段斷線模式
    block=4;
    mode =0;
    stop1();
    befor_online=true;
    stage=2;//進入到第二階段
    part=1;
    //Serial.println("disconnect");
  }
  
  if(stage==2 && part<5 && befor_online==true && rawValue==0){//如果在第二階段並且段線的段落沒有跑完並且上次還在線上此刻卻不再線上 
    block=5;     
    part++;//段落數往後推進一段
    befor_online =false;//上一刻線上狀態改否
    forward();//往前跑
  }
  else if(stage==2 && rawValue!=0){//如果在第二階段並且碰到線了 開啟PID循線模式
    block=6;
    mode =1;
    befor_online=true;
    calculation();
    fix(fix_value);
  }
  if(stage==2 && part==5){//第二階段完成，檢測是否進入第一次直角彎
    if( rawValue ==224 || rawValue==240){
    mode =3;
    block=7;
    stop1();
    stage=3;
    part = 0;
    }
  }
  if(stage==3 && part==0 && turn_done==false){//執行左轉
    block=8;
    time_counter++;        
    L90(time_counter,8);
   }
  if(stage==3 && turn_done==true){//結束左轉
    block=9;
    part =1;
    turn_done=false;
    stop1();
    time_counter=0;
   }
  if(stage==3 && part==1 && rawValue!=224 && rawValue!=240 && turn_done==false){//第一次左轉後的循跡
    mode =1;
    block=10;
    Serial.println("第一次左轉後的循跡");
    befor_online=true;
    Serial.println(rawValue);
    //error = log(rawValue)/log(2)-4.58;
    calculation();
    fix(fix_value);
  }
  if(stage==3 && part==1){//檢測是否進入第二次直角彎
    mode =3;
    if( rawValue ==224 || rawValue==240){
    Serial.println("檢測是否進入第二次直角彎");
    block=11;
    stop1();
    part = 2;
    }
  }
  if(stage==3 && part==2 && turn_done==false){//執行第二次左轉
    block=12;
    mode=0;
    time_counter++;        
    L90(time_counter,9);
    
    Serial.println("執行第二次左轉");
   }
  if(stage==3 && part==2 && turn_done==true){//停止左轉
  Serial.println("停止左轉");
    block=13;
    part =3;
    time_counter=0;
    turn_done=false;
    stop1();
   }
  if(stage==3 && part==3){ //第二次左轉後的循跡
  Serial.println("第二次左轉後的循跡");
    mode =1;
    block=14;
    befor_online=true;
    //error = log(rawValue)/log(2)-4.58;
    calculation();
    fix(fix_value);
  }
  
  if(stage==3 && part==3){//檢測是否進入第一次右轉直角彎
    if( rawValue ==15 || rawValue==7){
    block=15;
    stop1();//停止
    stage=4;
    part = 0;
    }
  }
  if(stage==4 && part==0 && turn_done==false){//執行第一次右轉
    block=16;
    time_counter++;        
    R90(time_counter,8);
   }
  if(stage==4 && part==0 && turn_done==true){//停止右轉
    block=17;
    part =1;
    turn_done=false;
    stop1();
   }
  if(stage==4 && part==1){ //第一次右轉轉後的循跡
  Serial.println("第一次右轉轉後的循跡");
  if(rawValue==0){//繞過寶特瓶
    stop1();
    stage=5;
    part =0;
  }
  if(rawValue!=0){
    mode =1;
    block=18;
    befor_online=true;
    //error = log(rawValue)/log(2)-4.58;
    calculation();
    fix(fix_value);
    }
  }
  if(stage ==5 && part ==0 && turn_done==false){//左轉
  time_counter++; 
  L90(time_counter,8);    
  }
  if(stage ==5&& part==0 && turn_done==true ){//清除狀態
    turn_done=false;
    part=1;    
  }
  if(stage ==5&& part==1 && turn_done==false ){//直走一段路
    time_counter++; 
    goF(time_counter,10);   
  }
  if(stage ==5&& part==1 && turn_done==true ){//清除狀態
    turn_done=false;
    part=2;    
  }
  if(stage ==5&& part==2 && turn_done==false ){//右轉
    time_counter++; 
    R90(time_counter,8);   
  }
  if(stage ==5&& part==2 && turn_done==true ){//清除狀態
    turn_done=false;
    part=3;    
  }
  if(stage ==5&& part==3 && turn_done==false ){//直走一段路
    time_counter++; 
    goF(time_counter,40);   
  }
  if(stage ==5&& part==3 && turn_done==true ){//清除狀態
    turn_done=false;
    part=4;  
  }
  if(stage ==5&& part==4 && turn_done==false ){//右轉
    time_counter++; 
    R90(time_counter,8);   
  }
  if(stage ==5&& part==4 && turn_done==true ){//清除狀態
    turn_done=false;
    part=5;  
  }
  if(stage ==5&& part==5 && turn_done==false ){//直走一段路
    time_counter++; 
    goF(time_counter,40);   
  }
  if(stage ==5&& part==5 && turn_done==true ){//清除狀態
    turn_done=false;
    stop1();
    part=6;  
  }
  if(stage ==5 && part ==6 && turn_done==false){//左轉
  time_counter++; 
  L90(time_counter,8);    
  }
  if(stage ==5&& part==6 && turn_done==true ){//清除狀態
    turn_done=false;
    stop1();
    part=7;  
  }
  if(stage==5 && part==7){ //右半邊開始的循跡
    mode =1;
    block=14;
    befor_online=true;
    //error = log(rawValue)/log(2)-4.58;
    calculation();
    fix(fix_value);
  }
   if(stage==5 && part==7){//檢測是否進入左轉直角彎
    if( rawValue ==224 || rawValue==240){
    mode =3;
    block=7;
    stop1();
    stage=6;
    part = 0;
    }
  }
  if(stage==6 && part==0 && turn_done==false){//執行左轉
    block=8;
    time_counter++;        
    L90(time_counter,8);
   }
  if(stage==6 && turn_done==true){//結束左轉
    block=9;
    part =1;
    turn_done=false;
    stop1();
    time_counter=0;
   }
  if(stage==6 && part==1 && rawValue!=224 && rawValue!=240 && turn_done==false){//左轉後的循跡
    mode =1;
    block=10;
    befor_online=true;
    calculation();
    fix(fix_value);
  }
   if(stage==6 && part==1){//檢測是否進入左轉直角彎
    if( rawValue ==224 || rawValue==240){
    mode =3;
    block=7;
    stop1();
    stage=7;
    part = 2;
    }
  }
  if(stage==7 && part==2 && turn_done==false){//執行左轉
    block=8;
    time_counter++;        
    L90(time_counter,8);
   }
  if(stage==7 && turn_done==true){//結束左轉
    block=9;
    part =3;
    turn_done=false;
    stop1();
    time_counter=0;
   }
  if(stage==7 && part==3 && rawValue!=224 && rawValue!=240 && turn_done==false){//左轉後的循跡
    mode =1;
    block=10;
    befor_online=true;
    calculation();
    fix(fix_value);
  }
}
void calculation(){
  if(rawValue<0)
  error = log(-1*rawValue)/log(2)-4.58;  
  else if(rawValue==0)
  error = 0;
  else
  error =log(rawValue)/log(2)-4.58;
  ie = ie+(error*dt);
  if(error-error_befor[0]!=0){
  de = (error-error_befor[0])/dt;
  }
  else if(error-error_befor[0]==0){
    de=0;
  }
  error_befor[0]=error;
  fix_value = kp*error+ki*ie+kd*de;
}
