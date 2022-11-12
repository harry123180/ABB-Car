#include"ServoTimer2.h"
#include <TimerOne.h>
// 右輪停止界線1538 
// 左輪停止界線1536
ServoTimer2 left;
ServoTimer2 right;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  left.attach(11);
  right.attach(13);
   Timer1.initialize(100000);
  Timer1.attachInterrupt(PID); 

}
void PID(){
  
}
void loop() {
  // put your main code here, to run repeatedly:
  int k = 30;
  left.write(1536+60);
  //delay(1000);
  right.write(1538-k);



}
