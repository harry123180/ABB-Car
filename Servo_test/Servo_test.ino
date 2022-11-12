#include <Servo.h>
// 右輪停止界線1538 
// 左輪停止界線1536
Servo left;
Servo right;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  left.attach(11);
  right.attach(13);

}

void loop() {
  // put your main code here, to run repeatedly:
  int k = 0;
  left.writeMicroseconds(1536+k);
  //delay(1000);
  right.writeMicroseconds(1538-k);



}
