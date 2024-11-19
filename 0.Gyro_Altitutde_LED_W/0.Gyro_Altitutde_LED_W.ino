/*
 * LED
 * BNO055 (Teruji di 500Hz)
 * BMP280 
 * Lidar (Max 100Hz) - 11/07/24
 * Voltage & Current Detection - 12/07/24
 *
 * version  V0.1
 * date  2024-11-7
 */
 
#include <Wire.h>
#include "DFRobot_BNO055.h" //IMU --> Gyroscope
#include "DFRobot_BMP280.h" //Pressure -> Altitude
#define LIDAR Serial5  // Define the Serial1 for TF-Luna communication
#define voltageSensorPin 22
#define currentSensorPin 23

//LIDAR
uint8_t recvBuffer[9];  // Buffer to store received data
int distance;           // Distance measured by the LiDAR
int strength;           // Signal strength

// Millis 1 - LED - 1 Hz
unsigned long pM1 = 0;                // PreviousMillis
const long interval1 = 1000;          // Interval to wait (1000 ms)
unsigned long cM1 = 0;                // CurrentMillis
// Millis 2 - IMU - 100 Hz
unsigned long pM2 = 0;                // PreviousMillis
const long interval2 = 10;            // Interval to wait (10 ms)
unsigned long cM2 = 0;                // CurrentMillis
unsigned long cM2_lidar = 0;
// Millis 3 - BMP - 10 Hz
unsigned long pM3 = 0;                // PreviousMillis
const long interval3 = 50;            // Interval to wait (100 ms)
unsigned long cM3 = 0;                // CurrentMillis

// Variable IMU
float roll_rate = 0;  // Roll Rate
float pitch_rate = 0;  // Pitch Rate
float yaw_rate = 0;  // Yaw Rate

// Variable BMP280
float   temp = 0;
uint32_t    press = 0;
float   alti = 0;


// BMP280
typedef DFRobot_BMP280_IIC    BMP;    // ******** use abbreviations instead of full names ********
BMP   bmp(&Wire, BMP::eSdoLow);       // WIRE0 SDA 18 SCL 19
#define SEA_LEVEL_PRESSURE    1015.0f   // sea level pressure
// show last sensor operate status
void printLastOperateStatus(BMP::eStatus_t eStatus)
{
  switch(eStatus) {
  case BMP::eStatusOK:    Serial.println("everything ok"); break;
  case BMP::eStatusErr:   Serial.println("unknow error"); break;
  case BMP::eStatusErrDeviceNotDetected:    Serial.println("device not detected"); break;
  case BMP::eStatusErrParameter:    Serial.println("parameter error"); break;
  default: Serial.println("unknow status"); break;
  }
}


DFRobot_BNO055 mpu;
void setup() 
{
//-----Setup Serial Monitor
  Serial.begin(115200);

//-----Setup LED
  pinMode(13, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(13, HIGH); // Indikator Setup
  delay(500);
  digitalWrite(13, LOW); // Indikator Setup

//----SETUP BNO055
  while (!mpu.init()){
    Serial.println("ERROR! Unable to initialize the chip!");
    delay(50);
  }
  // mpu.setMode(mpu.eNORMAL_POWER_MODE, mpu.eFASTEST_MODE);
  delay(100);
  Serial.println("Read Angular Velocity...");

//-----BMP280
  bmp.reset();
  Serial.println("bmp read data test");
  while(bmp.begin() != BMP::eStatusOK) {
    Serial.println("bmp begin faild");
    printLastOperateStatus(bmp.lastOperateStatus);
    delay(2000);
  }
  Serial.println("bmp begin success");
  
//-----Lidar
  LIDAR.begin(115200);        // LiDAR UART
  Serial.println("TF-Luna LiDAR End");

  Serial.println("\n\nEND OF SETUP\n\n");
}

void loop() 
{
  // 1 Hz Loop 
  cM1 = millis();
  if (cM1 - pM1 >= interval1) {
    digitalWrite(3, !digitalRead(3)); // Indikator Looping
    
    // Data IMU
    Serial.print("X: "); Serial.print(roll_rate, 3); Serial.print(" || "); 
    
    Serial.print("Y: "); Serial.print(pitch_rate, 3); Serial.print(" || ");
    
    Serial.print("Z: "); Serial.print(yaw_rate, 3); Serial.print(" || "); 
    
    // DATA BMP280
    Serial.print("temp: "); Serial.print(temp); Serial.print(" || ");
    Serial.print("p(PA): "); Serial.print(press); Serial.print(" || ");
    Serial.print("alt(m): "); Serial.println(alti);
    
    pM1 = cM1;
  }

  // 20 Hz Loop
  cM2 = millis();
  // cM2_lidar = millis();
  if (cM2 - pM2 >= interval2) {
    mpu.readAngularVelocity();  /* read Angular Velocity */
    // Variable IMU
    roll_rate = mpu.GyrData.x;  // Roll Rate
    pitch_rate = mpu.GyrData.y;  // Pitch Rate
    yaw_rate = mpu.GyrData.z;  // Yaw Rate
    Serial.print("PeriodeIMU: ");Serial.println(cM2 - pM2);
    pM2 = cM2;

    //Variable Lidar
    if (LIDAR.available()) {
    // Read one byte at a time into the buffer
      if (LIDAR.readBytes(recvBuffer, 9) == 9) {  // TF-Luna sends data in packets of 9 bytes
        // Verify if the packet is valid (check the frame header)
        if (recvBuffer[0] == 0x59 && recvBuffer[1] == 0x59) {
          distance = recvBuffer[2] + (recvBuffer[3] << 8);  // Calculate distance (low byte + high byte)
          strength = recvBuffer[4] + (recvBuffer[5] << 8);  // Calculate signal strength

          // Print the distance and signal strength
          Serial.print("Distance: ");
          Serial.print(distance);
          Serial.print(" cm\t");
          Serial.print("Signal Strength: ");
          Serial.println(strength);
          // Serial.print("PeriodeLidar: ");Serial.println(cM3 - pM2);
        }
      }
    }
  }

  // 10 Hz Loop
    cM3 = millis();
  if (cM3 - pM3 >= interval3) {
    temp = bmp.getTemperature();
    press = bmp.getPressure();
    alti = bmp.calAltitude(SEA_LEVEL_PRESSURE, press);

    pM3 = cM3;
  }
}

