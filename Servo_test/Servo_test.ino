#include"ServoTimer2.h"
#include <TimerOne.h>
// 右輪停止界線1538 
// 左輪停止界線1536
ServoTimer2 left;
ServoTimer2 right;
int ini=0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  left.attach(11);
  right.attach(13);
  left.write(1536);
  right.write(1538); 
   Timer1.initialize(100000);
  Timer1.attachInterrupt(PID); 

}
void PID(){
  
}
void loop() {
  // put your main code here, to run repeatedly:
  left.write(1536);
  right.write(1538); 
  delay(1000);
  int k = 30;
  if(ini==0){
  left.write(1536+60);
  right.write(1538+60);
  delay(760);
  ini++;
  }
  else if(ini==1){
    ini++;
  left.write(1536);
  right.write(1538);   
  delay(760); 
  }
  else if(ini==2){
    ini++;
  left.write(1536-60);
  right.write(1538-60);   
  delay(760); 
  }


}
