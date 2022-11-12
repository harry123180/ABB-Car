#include "sensorbar.h"
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
SensorBar mySensorBar(SX1509_ADDRESS);//實例化 SX1509
uint8_t rawValue;
void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
uint8_t returnStatus = mySensorBar.begin();
  mySensorBar.clearInvertBits();//暗背景找亮線
  mySensorBar.setInvertBits();//亮背景找黑線
}

void loop() {
  // put your main code here, to run repeatedly:
  rawValue=mySensorBar.getRaw();
  Serial.println(rawValue);
}
