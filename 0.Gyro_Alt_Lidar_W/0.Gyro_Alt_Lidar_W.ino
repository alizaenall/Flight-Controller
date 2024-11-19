/*
 * LED
 * BNO055 (Teruji di 500Hz) (perlu dikalibrasi)
 * BMP280 
 * Lidar (Max 100Hz)
 * Current & Votage Sensor (voltage oke)
 * Motor 
 * PID, blm selesai
 *
 *
 * version  V0.1
 * date  2024-11-8
 * Author : alizaenal
 */
 
#include <Wire.h>
// BNO055
#include "DFRobot_BNO055.h" //IMU --> Gyroscope
#include "DFRobot_BMP280.h" //Pressure -> Altitude
// Lidar
#define LIDAR Serial5  // Define the Serial1 for TF-Luna communication
// Power Module
#define voltageSensorPin 22   //Voltage Sensor from Power Module
#define currentSensorPin 23   //Current Sensor from Power Module
// LED
#define ledRed 30
#define ledGreen 31
#define ledTeensy 13
// Motor Control
#define receiverPin 9
#define mot1Pin 2 //M1
#define mot2Pin 3 //M2
#define mot3Pin 4 //M3
#define mot4Pin 5 //M4
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
int motorTestPin = mot2Pin;
float input_throttle = MIN_PULSE_LENGTH;
char data;

//LIDAR
uint8_t recvBuffer[9];  // Buffer to store received data
int distance;           // Distance measured by the LiDAR
int strength;           // Signal strength

// Current Voltage Sensor
int voltageBit,currentBit = 0;
float voltageVolt, currentVolt, voltage, current = 0;

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
// Micros 1 - PWM Motor - 4ms - 250Hz
unsigned long pM4 = 0;                // PreviousMillis
// Micros 2 - Loop Time
unsigned long loopTime = 0;
unsigned long pM5 = 0;

// Variable IMU
float RateRoll,RatePitch,RateYaw = 0;  // Gyro rate
float Roll, Pitch, Yaw = 0;               // Euler angle

// Variable BMP280
float   temp = 0;
uint32_t    press = 0;
float   alti = 0;

// PID 
float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber;

float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[]={0, 0, 0};
float PRateRoll=0.6 ; float PRatePitch=PRateRoll; float PRateYaw=2;
float IRateRoll=3.5 ; float IRatePitch=IRateRoll; float IRateYaw=12;
float DRateRoll=0.03 ; float DRatePitch=DRateRoll; float DRateYaw=0;

// BMP280
typedef DFRobot_BMP280_IIC    BMP;    // ******** use abbreviations instead of full names ********
BMP   bmp(&Wire, BMP::eSdoLow);       // WIRE0 SDA 18 SCL 19
#define SEA_LEVEL_PRESSURE    1015.0f   // sea level pressure
// show last sensor operate status
void printLastOperateStatus(BMP::eStatus_t eStatus){
  switch(eStatus) {
  case BMP::eStatusOK:    Serial.println("everything ok"); break;
  case BMP::eStatusErr:   Serial.println("unknow error"); break;
  case BMP::eStatusErrDeviceNotDetected:    Serial.println("device not detected"); break;
  case BMP::eStatusErrParameter:    Serial.println("parameter error"); break;
  default: Serial.println("unknow status"); break;
  }
}

DFRobot_BNO055 mpu;

void setup(){
//-----Setup Serial Monitor
  Serial.begin(115200);

//-----Setup LED 13-BuiltIn 30-RED 31-GREEN
  pinMode(13, OUTPUT); 
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
  digitalWrite(13, HIGH);
  digitalWrite(30, HIGH);
  digitalWrite(31, HIGH);

//----SETUP Voltage Current Sensor
    pinMode(voltageSensorPin, INPUT);
    pinMode(currentSensorPin, INPUT);

//----SETUP BNO055
  while (!mpu.init()){
    Serial.println("ERROR! Unable to initialize the chip!");
    delay(50);
  }
  // mpu.setMode(mpu.eNORMAL_POWER_MODE, mpu.eFASTEST_MODE);
  delay(100);
  Serial.println("MPUINITSuccess");
// Calibration Gyro
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  Serial.println("CalibrationDone");

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

// PWM Motor
  //RPM Freq 250Hz
  analogWriteFrequency(motorTestPin, 250);
  analogWriteResolution(12);


  Serial.println("\n\nEND OF SETUP\n\n");
  digitalWrite(13, LOW);
  LoopTimer=micros();
}

void loop(){
  // 1 Hz Loop 
  cM1 = millis();
  if (cM1 - pM1 >= interval1) {
    digitalWrite(ledRed, !digitalRead(ledRed)); // Indikator Looping
    
    // Data IMU BNO055
    Serial.print("GX: "); Serial.print(RateRoll, 3); Serial.print(" || "); 
    Serial.print("GY: "); Serial.print(RatePitch, 3); Serial.print(" || ");
    Serial.print("GZ: "); Serial.println(RateYaw, 3);
    Serial.print("Roll: ");Serial.print(Roll, 3); Serial.print(" || ");
    Serial.print("Pitch: ");Serial.print(Pitch, 3); Serial.print(" || ");
    Serial.print("Yaw: ");Serial.println(Yaw, 3); 

    // DATA BMP280
    Serial.print("temp: "); Serial.print(temp); Serial.print(" || ");
    Serial.print("p(PA): "); Serial.print(press); Serial.print(" || ");
    Serial.print("alt(m): "); Serial.println(alti);
    
    // Data Lidar
    Serial.print("Distance: ");Serial.print(distance);Serial.print(" cm\t");Serial.print("Signal Strength: ");Serial.println(strength);
    
    // Data Voltage Current Sensor
    Serial.print("currentBit: "); Serial.print(currentBit); Serial.print(" || voltagebit: "); Serial.print(voltageBit);
    Serial.print("|| current: "); Serial.print(current); Serial.print(" || voltage: "); Serial.println(voltage);
    
    Serial.println();
    pM1 = cM1;
  }

  // 20 Hz Loop
  cM2 = millis();
  // cM2_lidar = millis();
  if (cM2 - pM2 >= interval2) {
    mpu.readAngularVelocity();  /* read Angular Velocity */
    mpu.readEuler();  /* read euler angle */
    // Variable IMU
    RateRoll = mpu.GyrData.x;    // Roll Rate  (deg/s)
    RatePitch = mpu.GyrData.y;   // Pitch Rate (deg/s)
    RateYaw = mpu.GyrData.z;     // Yaw Rate   (deg/s)
    Roll = mpu.EulerAngles.x;     // Roll       (deg)
    Pitch = mpu.EulerAngles.y;    // Pitch      (deg)
    Yaw = mpu.EulerAngles.z;      // Pitch      (deg)
    
    // Serial.print("PeriodeIMU: ");Serial.println(cM2 - pM2);
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
          // Serial.print("Distance: ");Serial.print(distance);Serial.print(" cm\t");Serial.print("Signal Strength: ");Serial.println(strength);
          // Serial.print("PeriodeLidar: ");Serial.println(cM3 - pM2);
        }
      }
    }
  }

  // 250 Hz Loop
  if (micros() - pM4 < 4000){
  }
  else{
  pidLoop();
  // runMotor(input_throttle);
  Serial.print("PWM Cyle: ");Serial.println(micros() - pM4);
  pM4=micros();
  }

  // 10 Hz Loop
  cM3 = millis();
  if (cM3 - pM3 >= interval3) {
    temp = bmp.getTemperature();
    press = bmp.getPressure();
    alti = bmp.calAltitude(SEA_LEVEL_PRESSURE, press);

    // Voltage Current Sensor
    voltageCurrentRead();

    pM3 = cM3;
  }
  loopTime = micros() - pM5;
  pM5= micros();
  Serial.print("LoopTime: "); Serial.println(loopTime);

}

void voltageCurrentRead(){
    voltageBit = analogRead(voltageSensorPin); // Membaca nilai analog dari pin A0
    currentBit = analogRead(currentSensorPin); // Membaca nilai analog dari pin A0
    voltageVolt = (float)voltageBit/1023.0*3.3;
    currentVolt = (float)currentBit/1023.0*3.3;
    voltage = voltageVolt*35.34/3.3;  //maximum 30V, calibration value: 35.34/3.3
    current = currentVolt*90;      //maximum 90A, not yet calibrated
}

void runMotor(float thr){

    analogWrite(motorTestPin,1.024*thr);
    // Serial.println(thr);
}

// Gyro Signal a.k.a rate (deg/s)
void gyro_signals(void) {
    mpu.readAngularVelocity();  /* read Angular Velocity */
    mpu.readEuler();  /* read euler angle */
    // Variable IMU
    RateRoll = mpu.GyrData.x;    // Roll Rate  (deg/s)
    RatePitch = mpu.GyrData.y;   // Pitch Rate (deg/s)
    RateYaw = mpu.GyrData.z;     // Yaw Rate   (deg/s)
    Roll = mpu.EulerAngles.x;     // Roll       (deg)
    Pitch = mpu.EulerAngles.y;    // Pitch      (deg)
    Yaw = mpu.EulerAngles.z;      // Pitch      (deg)
}

void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
  float Pterm=P*Error;
  float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2;
  if (Iterm > 400) Iterm=400;
  else if (Iterm <-400) Iterm=-400;
  float Dterm=D*(Error-PrevError)/0.004;
  float PIDOutput= Pterm+Iterm+Dterm;
  if (PIDOutput>400) PIDOutput=400;
  else if (PIDOutput <-400) PIDOutput=-400;
  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}

void reset_pid(void) {
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
}

void pidLoop(){
    gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;

  DesiredRateRoll=0.15*(ReceiverValue[0]-1500);
  DesiredRatePitch=0.15*(ReceiverValue[1]-1500);
  InputThrottle=ReceiverValue[2];
  DesiredRateYaw=0.15*(ReceiverValue[3]-1500);
  ErrorRateRoll=DesiredRateRoll-RateRoll;
  ErrorRatePitch=DesiredRatePitch-RatePitch;
  ErrorRateYaw=DesiredRateYaw-RateYaw;

  ErrorRateRoll=DesiredRateRoll-RateRoll;
  ErrorRatePitch=DesiredRatePitch-RatePitch;
  ErrorRateYaw=DesiredRateYaw-RateYaw;

  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
       InputRoll=PIDReturn[0];
       PrevErrorRateRoll=PIDReturn[1]; 
       PrevItermRateRoll=PIDReturn[2];
  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
       InputPitch=PIDReturn[0]; 
       PrevErrorRatePitch=PIDReturn[1]; 
       PrevItermRatePitch=PIDReturn[2];
  pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
       InputYaw=PIDReturn[0]; 
       PrevErrorRateYaw=PIDReturn[1]; 
       PrevItermRateYaw=PIDReturn[2];

  if (InputThrottle > 1800) InputThrottle = 1800;
  MotorInput1= 1.024*(InputThrottle-InputRoll-InputPitch-InputYaw);
  MotorInput2= 1.024*(InputThrottle-InputRoll+InputPitch+InputYaw);
  MotorInput3= 1.024*(InputThrottle+InputRoll+InputPitch-InputYaw);
  MotorInput4= 1.024*(InputThrottle+InputRoll-InputPitch+InputYaw);

  if (MotorInput1 > 2000)MotorInput1 = 1999;
  if (MotorInput2 > 2000)MotorInput2 = 1999; 
  if (MotorInput3 > 2000)MotorInput3 = 1999; 
  if (MotorInput4 > 2000)MotorInput4 = 1999;

  int ThrottleIdle=1180;
  if (MotorInput1 < ThrottleIdle) MotorInput1 =  ThrottleIdle;
  if (MotorInput2 < ThrottleIdle) MotorInput2 =  ThrottleIdle;
  if (MotorInput3 < ThrottleIdle) MotorInput3 =  ThrottleIdle;
  if (MotorInput4 < ThrottleIdle) MotorInput4 =  ThrottleIdle;

  int ThrottleCutOff=1000;
  if (ReceiverValue[2]<1050) {
    MotorInput1=ThrottleCutOff; 
    MotorInput2=ThrottleCutOff;
    MotorInput3=ThrottleCutOff; 
    MotorInput4=ThrottleCutOff;
    reset_pid();
  }
  
  analogWrite(1,MotorInput1);
  analogWrite(2,MotorInput2);
  analogWrite(3,MotorInput3); 
  analogWrite(4,MotorInput4);
}