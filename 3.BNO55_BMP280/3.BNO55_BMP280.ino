/*!
  * imu_show.ino
  *
  * Download this demo to show attitude on [imu_show](https://github.com/DFRobot/DFRobot_IMU_Show)
  * Attitude will show on imu_show
  *
  * Product: https://www.dfrobot.com.cn/goods-1860.html
  * Copyright   [DFRobot](https://www.dfrobot.com), 2016
  * Copyright   GNU Lesser General Public License
  *
  * version  V1.0
  * date  07/03/2019
  */

#include "DFRobot_BNO055.h"
#include "Wire.h"

// Gunakan Wire1 untuk I2C2 (pin 16: SCL, 17: SDA) pada Teensy 4.1
typedef DFRobot_BNO055_IIC    BNO;
// DFRobot_BNO055_IIC bno(&Wire1, 0x28); // 0x28 adalah alamat default dari sensor
BNO   bno(&Wire1, 0x28);    // input TwoWire interface and IIC address


// void printLastOperateStatus(){}
// show last sensor operate status
void printLastOperateStatus(BNO::eStatus_t eStatus)
{
  switch(eStatus) {
  case BNO::eStatusOK:    Serial.println("everything ok"); break;
  case BNO::eStatusErr:   Serial.println("unknow error"); break;
  case BNO::eStatusErrDeviceNotDetect:    Serial.println("device not detected"); break;
  case BNO::eStatusErrDeviceReadyTimeOut: Serial.println("device ready time out"); break;
  case BNO::eStatusErrDeviceStatus:       Serial.println("device internal status error"); break;
  default: Serial.println("unknow status"); break;
  }
}


void setup()
{
  Serial.begin(115200);
  // Inisialisasi Wire1 (I2C2)
  // Wire1.begin();
  // Wire1.setSDA(17);
  // Wire1.setSDA(16);
Serial.println("Serial begin");
  bno.reset();
Serial.println("bno reset");
  while(bno.begin() != BNO::eStatusOK) {
    Serial.println("bno begin faild");
    
    printLastOperateStatus(bno.lastOperateStatus);
    delay(2000);
  }
Serial.println("bno begin");


  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delay(4000);
  digitalWrite(2, LOW);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);

  Serial.println("bno begin success");
}

void loop()
{
  BNO::sEulAnalog_t   sEul;
  sEul = bno.getEul();
  Serial.print("pitch:");
  Serial.print(sEul.pitch, 3);
  Serial.print(" ");
  Serial.print("roll:");
  Serial.print(sEul.roll, 3);
  Serial.print(" ");
  Serial.print("yaw:");
  Serial.print(sEul.head, 3);
  Serial.println(" ");
  delay(80);
}