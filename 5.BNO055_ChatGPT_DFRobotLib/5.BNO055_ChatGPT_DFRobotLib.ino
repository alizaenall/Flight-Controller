#include <Wire.h>
#include "DFRobot_BNO055.h"

// Gunakan Wire1 untuk I2C2 (pin 16: SCL, 17: SDA) pada Teensy 4.1
DFRobot_BNO055_IIC bno(&Wire1, 0x28); // 0x28 adalah alamat default dari sensor

void setup() {
  Serial.begin(115200);

  // Inisialisasi Wire1 (I2C2)
  Wire1.begin();

  // Inisialisasi sensor BNO055
  if (!bno.begin()) {
    Serial.println("Gagal menginisialisasi sensor BNO055, periksa koneksi!");
    while (1);
  }

  // Atur ke mode operasi NDOF (menyertakan orientasi dengan 9DOF)
  // bno.setMode(OPERATION_MODE_NDOF);
  delay(1000);
  
  Serial.println("Sensor siap!");
}

void loop() {
  // Membaca data orientasi (Euler angles)
  // float euler_angles[3];
  // if (bno.getEul(euler_angles)) {
  //   Serial.print("Orientasi X: "); Serial.print(euler_angles[0]);
  //   Serial.print(" Y: "); Serial.print(euler_angles[1]);
  //   Serial.print(" Z: "); Serial.println(euler_angles[2]);
  // } else {
  //   Serial.println("Gagal membaca data orientasi.");
  // }

  // delay(500);
}
