#include <Wire.h>
// BNO055
#include "DFRobot_BNO055.h" //IMU --> Gyroscope
#include "DFRobot_BMP280.h" //Pressure -> Altitude
DFRobot_BNO055 mpu;

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

float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
uint32_t LoopTimer;

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


void setup() {
  
  // Calbration Gyro
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

  LoopTimer=micros();
}

void loop() {
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

  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
}
