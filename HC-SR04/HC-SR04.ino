

int trigPin = 6;    // 超音波感測器 Trig腳接 Arduino pin 6
int echoPin = 7;    //超音波感測器 Echo 腳接 Arduino pin 7
long duration,cm ;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);      //Arduino 對外啟動距離感測器Trig腳，射出超音波 
  pinMode(echoPin, INPUT);       //超音波被障礙物反射後，Arduino讀取感測器Echo腳的時間差
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);     // 給 Trig 高電位，持續 10微秒
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  pinMode(echoPin, INPUT);             // 讀取 echo 的電位
  duration = pulseIn(echoPin, HIGH);   // 收到高電位時的時間
 
  cm = (duration/2) / 29.1;         // 將時間換算成距離 cm 
  Serial.println(cm);
}
