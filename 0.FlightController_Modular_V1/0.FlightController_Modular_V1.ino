/*
  USER INPUT INTEGER on SERIAL MONITOR
  // 18 Sending minimum throttle
  // 1 Sending maximum throttle
  // 2 Running test
  // 3 Throttle : 1100
  // 4 Throttle : 1200
  // 5 Throttle : 1300
  // 6 Throttle Add + 50us
  // 7 Throttle Subtract - 50us
  // 8 Throttle Down-3S
  // 9 Throttle Down-2S
  //10 SP_RPM: 1000
  //11 SP_RPM: 3000
  //12 SP_RPM: 5000
  //13 SP_RPM: 7000
  //14 SP_RPM: +500
  //15 SP_RPM: -500
  //16 SP_RPM: +1000
  //17 SP_RPM: -1000
  //23 RefRoll : ... Input...
  //
  //31 PAnglePitch : 5
    32 IAnglePitch : 0
    33 DAnglePitch : 0
    34 PRateRoll  : 1
    35 IRateRoll  : 0
    36 DRateRoll  : 0
  //
*/



// Enable or disable modules by commenting/uncommenting
#define ENABLE_CUSTOM

#define ENABLE_MOTOR              // 5        // DEFAULT QUAD
// #define ENABLE_MOTOR_HEXACOPTER             // UNCOMMENT FOR HEXACOPTER
// #define ENABLE_MOTOR_MANUAL_ALL
// #define ENABLE_MOTOR_MANUAL_PER_MOTOR
// #define ENABLE_MOTOR_MANUAL_1
// #define ENABLE_MOTOR_MANUAL_2
// #define ENABLE_MOTOR_MANUAL_3
// #define ENABLE_MOTOR_MANUAL_4

#define ENABLE_IMU              // 1  
// #define ENABLE_BMP              // 2  
// #define ENABLE_LIDAR            // 3  
// #define ENABLE_VOLTAGE_CURRENT  // 4  
// #define ENABLE_PID_RATE              // 6  // DURUNG
// #define ENABLE_PID_ANGLE
// #define ENABLE_PID_ANGLE_ONLY
// #define ENABLE_FUZZY_ROLL
// #define ENABLE_SD_CARD          // 7  
// #define ENABLE_RPM_1            // 8.1  // Durung
// #define ENABLE_RPM_2            // 8.2
// #define ENABLE_RPM_3            // 8.3
// #define ENABLE_RPM_4            // 8.4
// #define ENABLE_RTC
// #define ENABLE_RECEIVER         // 9 .
#define ENABLE_USER_INPUT      // 10
#define ENABLE_LED
  #define ENABLE_IMU_PRINT      // done
  // #define ENABLE_BMP_PRINT      
  // #define ENABLE_LIDAR_PRINT
  // #define ENABLE_VOLTAGE_CURRENT_PRINT
  // #define ENABLE_MOTOR_PRINT
  // #define ENABLE_PID_RATE_PRINT
  // #define ENABLE_PID_ANGLE_PRINT   
  // #define ENABLE_RPM_1_PRINT             // DURUNG
  // #define ENABLE_RTC_PRINT
  // #define ENABLE_RECEIVER_PRINT  // DURUNG
  #define ENABLE_CUSTOM_PRINT


#define ENABLE_INTERVAL  // WAJIB
    // #define ENABLE_MONITORING_1HZ
//           #define ENABLE_MONITORING_IMU_1HZ
//           #define ENABLE_MONITORING_BMP_1HZ
//           #define ENABLE_MONITORING_LIDAR_1HZ
//           #define ENABLE_MONITORING_VOLTAGE_CURRENT_1HZ
          // #define ENABLE_MONITORING_MOTOR_1HZ
          // #define ENABLE_MONITORING_PID_1HZ
          // #define ENABLE_MONITORING_RPM_1HZ
          // #define ENABLE_MONITORING_RTC_1HZ
    #define ENABLE_MONITORING_10HZ
          #define ENABLE_MONITORING_IMU_10HZ
          // #define ENABLE_MONITORING_BMP_10HZ
          // #define ENABLE_MONITORING_LIDAR_10HZ
          // #define ENABLE_MONITORING_VOLTAGE_CURRENT_10HZ
          // #define ENABLE_MONITORING_MOTOR_10HZ
          // #define ENABLE_MONITORING_PID_10HZ
          // #define ENABLE_MONITORING_RPM_10HZ
          // #define ENABLE_MONITORING_FUZZY_ROLL_10HZ //Masih Salah
          // #define ENABLE_MONITORING_RTC_10HZ
    // #define ENABLE_MONITORING_DYNAMIC              // BELUM BISA
  
#define ENABLE_SERIAL1            // COM15 // Tools > UBS_Type > Dual_Serial
  #define ENABLE_SERIAL1_INPUT
  //Choose ONE of SERIAL FREQUENCY from option BELOW/
  // #define ENABLE_SERIAL1_500HZ
  #define ENABLE_SERIAL1_100HZ
  // #define ENABLE_SERIAL1_10HZ
  // #define ENABLE_SERIAL1_1HZ


#include <Wire.h>

#ifdef ENABLE_INTERVAL
  // Motor && Kalman && PID- 500 Hz
  unsigned long pM500Hz = 0;                // PreviousMillis
  const long interval500Hz = 2000;      // Interval to wait (2 ms a.k.a 2000 us)
  // unsigned long cM500Hz = 0;                // CurrentMillis

  // IMU && Lidar - 100 Hz
  unsigned long pM100Hz = 0;                // PreviousMillis
  const long interval100Hz = 10;        // Interval to wait (10 ms)
  // unsigned long cM100Hz = 0;                // CurrentMillis
  // unsigned long cM2_lidar = 0;
  
  // GPS - 10 Hz
  unsigned long pM10Hz = 0;                // PreviousMillis
  const long interval10Hz = 100;        // Interval to wait (100 ms)
  // unsigned long cM10Hz = 0;                // CurrentMillis

  // LED & SerialMonitor - 1/5 Hz
  unsigned long pM1Hz = 0;                // PreviousMillis
  const long interval1Hz = 1000;        // Interval to wait (1000 ms)
  // unsigned long cM1Hz = 0;                // CurrentMillis

  unsigned long cMicros = 0;
  unsigned long cMillis = 0;
  unsigned long iteration = 0;


  void initInterval(){
    cMicros = micros();
    cMillis = millis();
  }
#elif
  // Motor && Kalman && PID- 500 Hz
  unsigned long pM500Hz = 0;                // PreviousMillis
  const long interval500Hz = 2000;      // Interval to wait (2 ms a.k.a 2000 us)
  // unsigned long cM500Hz = 0;                // CurrentMillis

  // IMU && Lidar - 100 Hz
  unsigned long pM100Hz = 0;                // PreviousMillis
  const long interval100Hz = 0;        // Interval to wait (10 ms)
  // unsigned long cM100Hz = 0;                // CurrentMillis
  // unsigned long cM2_lidar = 0;
  
  // GPS - 10 Hz
  unsigned long pM10Hz = 0;                // PreviousMillis
  const long interval10Hz = 0;        // Interval to wait (100 ms)
  // unsigned long cM10Hz = 0;                // CurrentMillis

  // LED & SerialMonitor - 1/5 Hz
  unsigned long pM1Hz = 0;                // PreviousMillis
  const long interval1Hz = 0;        // Interval to wait (1000 ms)
  // unsigned long cM1Hz = 0;                // CurrentMillis
  void initInterval(){
    cMicros = micros();
    cMillis = millis();
  }

#endif

// 1. BNO055 - IMU --> Gyroscope & Acc & EulerAngle 
#ifdef ENABLE_IMU
  #include "DFRobot_BNO055.h" //IMU --> Gyroscope / I2C / SDA0 SCL0
  DFRobot_BNO055 mpu;
  // Variable BNO055
  float RateRoll,RatePitch,RateYaw;     // Gyro rate
  float Roll, Pitch, Yaw;               // Euler angle
  float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
  float AngleCalibrationPitch, AngleCalibrationRoll, AngleCalibrationYaw;
  float RateRollCal = 0, RatePitchCal = 0, RateYawCal = 0;
  float RollCal = 0, PitchCal = 0, YawCal = 0;
  int RateCalibrationNumber;
  
  //----SETUP BNO055
  void initImu(){
    while (!mpu.init()){
      Serial.println("ERROR! Unable to initialize the chip!");
      delay(50);
    }
    mpu.setMode(mpu.eNORMAL_POWER_MODE, mpu.eFASTEST_MODE); // Fastest 100Hz
    delay(100);
    Serial.println("MPUINITSuccess");
  // Calibration Gyro
    for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
      readImu();
      RateCalibrationRoll+=RateRoll;
      RateCalibrationPitch+=RatePitch;
      RateCalibrationYaw+=RateYaw;
      AngleCalibrationPitch+=Pitch;
      AngleCalibrationRoll+=Roll;
      AngleCalibrationYaw+=Yaw;

      delay(1);
    }
    RateCalibrationRoll/=2000;
    RateCalibrationPitch/=2000;
    RateCalibrationYaw/=2000;
    
    AngleCalibrationPitch/=2000;
    AngleCalibrationRoll/=2000;
    AngleCalibrationYaw/=2000;

    RateRollCal = RateCalibrationRoll; 
    RatePitchCal = RateCalibrationPitch; 
    RateYawCal = RateCalibrationYaw;
    RollCal = AngleCalibrationRoll; 
    PitchCal = AngleCalibrationPitch; 
    YawCal = AngleCalibrationYaw;

    Serial.println("CalibrationDone");
  }

  void readImu(){
      mpu.readAngularVelocity();  /* read Angular Velocity */
      mpu.readEuler();  /* read euler angle */
      // Variable IMU
      RateRoll = mpu.GyrData.x - RateRollCal;     // Roll Rate  (deg/s)
      RatePitch = mpu.GyrData.y - RatePitchCal;   // Pitch Rate (deg/s)
      RateYaw = mpu.GyrData.z - RateYawCal;       // Yaw Rate   (deg/s)
      Roll = mpu.EulerAngles.z - RollCal;         // Roll       (deg)
      Pitch = mpu.EulerAngles.y - PitchCal;       // Pitch      (deg)
      Yaw = mpu.EulerAngles.x - YawCal;           // Yaw        (deg)
    
      #ifdef ENABLE_IMU_PRINT
        Serial.print("IMU| ");
        Serial.print("R:");Serial.print(Roll,2); Serial.print(" | ");
        Serial.print("P:");Serial.print(Pitch,2); Serial.print(" | ");
        Serial.print("Y:");Serial.print(Yaw,2);Serial.print(" | ");
        Serial.print("Gx:");Serial.print(RateRoll,2); Serial.print(" | ");
        Serial.print("Gy:");Serial.print(RatePitch,2); Serial.print(" | ");
        Serial.print("Gz:");Serial.println(RateYaw,2); 
        
      #endif
  }
#endif

// 2. BMP280 - Atmosphere Pressure Sensor --> Altitude 
#ifdef ENABLE_BMP
  #include "DFRobot_BMP280.h"

  typedef DFRobot_BMP280_IIC    BMP;    // ******** use abbreviations instead of full names ********
  BMP   bmp(&Wire, BMP::eSdoLow);
  #define SEA_LEVEL_PRESSURE    1015.0f   // sea level pressure
  
  // Variable BMP280
  float   temp = 0;
  uint32_t    press = 0;
  float   alti = 0;

  //Setup BMP280
  void initBmp(){
    bmp.reset();
    Serial.println("bmp read data test");
    bmp.begin();
    // while(bmp.begin() != BMP::eStatusOK) {
    //   Serial.println("bmp begin faild");
    //   // printLastOperateStatus(bmp.lastOperateStatus);
    //   delay(2000);
    // }
    Serial.println("bmp begin success");
    delay(100);
  }

  // int printLastOperateStatus(BMP::eStatus_t eStatus){
  // //   switch(eStatus) {
  // //   case BMP::eStatusOK:    Serial.println("everything ok"); break;
  // //   case BMP::eStatusErr:   Serial.println("unknow error"); break;
  // //   case BMP::eStatusErrDeviceNotDetected:    Serial.println("device not detected"); break;
  // //   case BMP::eStatusErrParameter:    Serial.println("parameter error"); break;
  // //   default: Serial.println("unknow status"); break;
  // //   }
  // }

  void readBmp(){
    temp = bmp.getTemperature();
    press = bmp.getPressure();
    alti = bmp.calAltitude(SEA_LEVEL_PRESSURE, press);

    #ifdef ENABLE_BMP_PRINT
      Serial.print("BMP| ");
      Serial.print("Alt:");Serial.print(alti); Serial.print(" | ");
      Serial.print("Pressure:");Serial.print(press); Serial.print(" | ");
      Serial.print("Temp:");Serial.println(temp);
    #endif
  }

#endif

// 3. LIDAR
#ifdef ENABLE_LIDAR
  #define LIDAR Serial5  // Define the Serial1 for TF-Luna communication

  uint8_t recvBuffer[9];  // Buffer to store received data
  int distance;           // Distance measured by the LiDAR
  int strength;           // Signal strength
  
  void initLidar(){
  LIDAR.begin(115200);        // LiDAR UART
  Serial.println("TF-Luna LiDAR End");
  }

  void readLidar(){
    if (LIDAR.available()) {
    // Read one byte at a time into the buffer
      if (LIDAR.readBytes(recvBuffer, 9) == 9) {  // TF-Luna sends data in packets of 9 bytes
        // Verify if the packet is valid (check the frame header)
        if (recvBuffer[0] == 0x59 && recvBuffer[1] == 0x59) {
          distance = recvBuffer[2] + (recvBuffer[3] << 8);  // Calculate distance (low byte + high byte)
          strength = recvBuffer[4] + (recvBuffer[5] << 8);  // Calculate signal strength
          
          #ifdef ENABLE_LIDAR_PRINT
            Serial.print("Lidar| ");
            Serial.print("Dist: "); Serial.print(distance); Serial.print(" | ");
            Serial.print("Strength: "); Serial.println(strength);
          #endif
        }
      }
    }
  }

#endif

// 4. Voltage and Current Sensor
#ifdef ENABLE_VOLTAGE_CURRENT
  
  #define voltageSensorPin 22   //Voltage Sensor from Power Module
  #define currentSensorPin 23   //Current Sensor from Power Module

  int voltageBit,currentBit = 0;
  float voltageVolt, currentVolt, voltage, current = 0;


  void initVoltageCurrent(){
    pinMode(voltageSensorPin, INPUT);
    pinMode(currentSensorPin, INPUT);
  }

  void readVoltageCurrent(){
    voltageBit = analogRead(voltageSensorPin); // Membaca nilai analog dari pin A0
    currentBit = analogRead(currentSensorPin); // Membaca nilai analog dari pin A0
    voltageVolt = (float)voltageBit/1023.0*3.3;
    currentVolt = (float)currentBit/1023.0*3.3;
    voltage = voltageVolt*35.34/3.3;  //maximum 30V, calibration value: 35.34/3.3
    current = currentVolt*90;      //maximum 90A, not yet calibrated

    #ifdef ENABLE_VOLTAGE_CURRENT_PRINT
      Serial.print("Analog| ");
      Serial.print("V: ");Serial.print(voltage); Serial.print(" | ");
      Serial.print("I: ");Serial.print(current); Serial.print(" | ");
      Serial.print("V_volt: ");Serial.print(voltageVolt); Serial.print(" | ");
      Serial.print("I_volt: ");Serial.println(currentVolt);
    #endif
  }

#endif

// 5. Motor Control

#ifdef ENABLE_MOTOR
  // DEFAULT Quad 4 Motor
  #define MotorPin1 2 //M1 - Quad
  #define MotorPin2 3 //M2 - Quad
  #define MotorPin3 4 //M3 - Quad 
  #define MotorPin4 5 //M4 - Quad


  #ifdef ENABLE_MOTOR_HEXACOPTER
    #define MotorPin5 6 //M5 - Hexa
    #define MotorPin6 7 //M6 - Hexa
  #endif

  #define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
  #define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs

  float MotorInput1 = MIN_PULSE_LENGTH , MotorInput2 = MIN_PULSE_LENGTH, MotorInput3 = MIN_PULSE_LENGTH; 
  float MotorInput4 = MIN_PULSE_LENGTH , MotorInput5 = MIN_PULSE_LENGTH, MotorInput6 = MIN_PULSE_LENGTH;


  float input_throttle = MIN_PULSE_LENGTH;
  float motor1_throttle = 0;
  float motor2_throttle = 0;
  float motor3_throttle = 0;
  float motor4_throttle = 0;

  void initMotor(){
    analogWriteFrequency(MotorPin1, 500);
    analogWriteFrequency(MotorPin2, 500);
    analogWriteFrequency(MotorPin3, 500);
    analogWriteFrequency(MotorPin4, 500);

    #ifdef ENABLE_MOTOR_HEXACOPTER
    analogWriteFrequency(MotorPin5, 500);
    analogWriteFrequency(MotorPin6, 500);
    #endif
    
    analogWriteResolution(11);
  }

  void runMotor(float m1=1000.0, float m2=1000.0, float m3=1000.0, float m4=1000.0, float m5=1000.0, float m6=1000.0){
    // analogWrite(motorTestPin,1.024*thr);
    // Serial.println(thr);
    // float m1= m1;
    analogWrite(MotorPin1, m1+12);
    analogWrite(MotorPin2, m2+12);
    analogWrite(MotorPin3, m3+12); 
    analogWrite(MotorPin4, m4+12);

    #ifdef ENABLE_MOTOR_HEXACOPTER
    analogWrite(MotorPin5, m5+12);
    analogWrite(MotorPin6, m6+12);
    #endif

    #ifdef ENABLE_MOTOR_PRINT
      Serial.print("Motor| ");
      Serial.print("M1: "); Serial.print(m1); Serial.print("| M2: "); Serial.print(m2);
      Serial.print("| M3: "); Serial.print(m3); Serial.print("| M4: "); Serial.print(m4);
      Serial.print("| M5: "); Serial.print(MotorInput5); Serial.print("| M6: "); Serial.println(MotorInput6);
    #endif
  }
#endif

// 8. PID
float setPointRpm = 0;
float RefRateRoll = 0;
float RefRatePitch = 0;
float RefRateYaw = 0;
#ifdef ENABLE_PID_RATE
  // Inner Control
  // setPointRpm = 0;
  // float DesiredRpm = setPointRpm;

  // Supervisory Control
  float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
  float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
  float InputRoll, InputThrottle, InputPitch, InputYaw;
  float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
  float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
  float PIDReturn[]={0, 0, 0};
  float PRateRoll=1 ; float PRatePitch=0; float PRateYaw=0;
  float IRateRoll=0; float IRatePitch=0; float IRateYaw=0;
  float DRateRoll=0 ; float DRatePitch=0; float DRateYaw=0;
  // float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
  // SetPoint Controller PID
  float ManualThrottle = input_throttle;
  float ManualRateRoll = RefRateRoll;
  float ManualRatePitch = RefRatePitch;
  float ManualRateYaw = RefRateYaw;

  

  void initPidRate(){
    // DesiredRateRoll=0.15*(ReceiverValue[0]-1500);
    DesiredRateRoll=ManualRateRoll;
    // DesiredRatePitch=0.15*(ReceiverValue[1]-1500);
    DesiredRatePitch=ManualRatePitch;
    // InputThrottle=ReceiverValue[2];
    InputThrottle=ManualThrottle;
    // DesiredRateYaw=0.15*(ReceiverValue[3]-1500);
    DesiredRateYaw=ManualRateYaw;
  }

  void resetPidRate(void) {
    PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
    PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
  }


  void pidRateEquation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
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

  void errorPidRate(){
    ErrorRateRoll=DesiredRateRoll-RateRoll;
    ErrorRatePitch=DesiredRatePitch-RatePitch;
    ErrorRateYaw=DesiredRateYaw-RateYaw;
  }
  void loopPidRate(){
    errorPidRate();
    pidRateEquation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
        InputRoll=PIDReturn[0];
        PrevErrorRateRoll=PIDReturn[1]; 
        PrevItermRateRoll=PIDReturn[2];
    pidRateEquation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
        InputPitch=PIDReturn[0]; 
        PrevErrorRatePitch=PIDReturn[1]; 
        PrevItermRatePitch=PIDReturn[2];
    pidRateEquation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
        InputYaw=PIDReturn[0]; 
        PrevErrorRateYaw=PIDReturn[1]; 
        PrevItermRateYaw=PIDReturn[2];

    InputThrottle = input_throttle;

    if (InputThrottle > 1500) InputThrottle = 1500;
   


    // + configuration
    /*      M1
            |
            |
      M4--------- M2
            |
            |
            M3
    */

    MotorInput1= 1.024*(InputThrottle-InputPitch);
    MotorInput2= 1.024*(InputThrottle-InputRoll);
    MotorInput3= 1.024*(InputThrottle+InputPitch);
    MotorInput4= 1.024*(InputThrottle+InputRoll);

    int MaxMotorInput=1700;
    if (MotorInput1 > MaxMotorInput)MotorInput1 = MaxMotorInput;
    if (MotorInput2 > MaxMotorInput)MotorInput2 = MaxMotorInput; 
    if (MotorInput3 > MaxMotorInput)MotorInput3 = MaxMotorInput; 
    if (MotorInput4 > MaxMotorInput)MotorInput4 = MaxMotorInput;

    int ThrottleIdle=1100;
    if (MotorInput1 < ThrottleIdle) MotorInput1 =  ThrottleIdle;
    if (MotorInput2 < ThrottleIdle) MotorInput2 =  ThrottleIdle;
    if (MotorInput3 < ThrottleIdle) MotorInput3 =  ThrottleIdle;
    if (MotorInput4 < ThrottleIdle) MotorInput4 =  ThrottleIdle;

    int ThrottleCutOff=1000;
    if (InputThrottle<1050) {
      MotorInput1=ThrottleCutOff; 
      MotorInput2=ThrottleCutOff;
      MotorInput3=ThrottleCutOff; 
      MotorInput4=ThrottleCutOff;
      resetPidRate();
    }
    
    #ifdef ENABLE_PID_RATE_PRINT
      Serial.print("PID| ");
      Serial.print("iT:");Serial.print(InputThrottle); Serial.print(" | ");
      Serial.print("iR:");Serial.print(InputRoll); Serial.print(" | ");
      Serial.print("iP:");Serial.print(InputPitch); Serial.print(" | ");
      Serial.print("iY:");Serial.println(InputYaw);
      
      Serial.print("MotorPID| ");
      Serial.print("M1P:");Serial.print(MotorInput1); Serial.print(" | ");
      Serial.print("M2P:");Serial.print(MotorInput2); Serial.print(" | ");
      Serial.print("M3P:");Serial.print(MotorInput3); Serial.print(" | ");
      Serial.print("M4P:");Serial.print(MotorInput4); Serial.print(" | ");
      Serial.print("M5P:");Serial.print(MotorInput5); Serial.print(" | ");
      Serial.print("M6P:");Serial.println(MotorInput6);

    #endif
  }

#endif

// float setPointRpm = 0;
// float RefRateRoll = 0;
// float RefRatePitch = 0;
// float RefRateYaw = 0;
float refRoll = 0;
float refPitch = 0;
float refYaw = 0;

#ifdef ENABLE_PID_ANGLE

  #ifdef ENABLE_PID_ANGLE_ONLY
    float InputThrottle = 1000, InputRoll = 0, InputPitch = 0, InputYaw = 0;
  #endif

  // Supervisory Control
  float DesiredRoll, DesiredPitch, DesiredYaw;
  float ErrorRoll, ErrorPitch, ErrorYaw;
  float PrevErrorRoll, PrevErrorPitch, PrevErrorYaw;
  float PrevItermRoll, PrevItermPitch, PrevItermYaw;
  float PIDAngleReturn[]={0, 0, 0};
  float PAngleRoll=1 ; float PAnglePitch=0; 
  float IAnglePitch=2; float IAnglePitch=0; 
  float DAngleRoll=0 ; float DAnglePitch=0; 

  float PYaw=0; float IYaw= 0; float DYaw=0;
  // float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
  // SetPoint Controller PID
  // float ManualThrottle = input_throttle;
  float ManualRoll = refRoll;               // deg
  float ManualPitch = refPitch;              // deg
  float ManualYaw = refYaw;                // deg

  void initPidAngle(){
    // DesiredRateRoll=0.15*(ReceiverValue[0]-1500);
    DesiredRoll=ManualRoll;
    // DesiredRatePitch=0.15*(ReceiverValue[1]-1500);
    DesiredPitch=ManualPitch;
    // InputThrottle=ReceiverValue[2];
    // InputThrottle=ManualThrottle;
    // DesiredRateYaw=0.15*(ReceiverValue[3]-1500);
    DesiredYaw=ManualYaw;
  }

   void pidAngleEquation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
      float Pterm=P*Error;
      float Iterm=PrevIterm+I*(Error+PrevError)*0.01/2;
      if (Iterm > 400) Iterm=400;
      else if (Iterm <-400) Iterm=-400;
      float Dterm=D*(Error-PrevError)/0.004;
      float PIDOutput= Pterm+Iterm+Dterm;
      if (PIDOutput>300) PIDOutput=300;
      else if (PIDOutput <-300) PIDOutput=-300;
      PIDAngleReturn[0]=PIDOutput;
      PIDAngleReturn[1]=Error;
      PIDAngleReturn[2]=Iterm;
  }

  void resetPidAngle(void) {
    PrevErrorRoll=0; PrevErrorPitch=0; PrevErrorYaw=0;
    PrevItermRoll=0; PrevItermPitch=0; PrevItermYaw=0;
  }

  void errorPidAngle(){
    ErrorRoll=DesiredRoll-Roll;
    ErrorPitch=DesiredPitch-Pitch;
    ErrorYaw=DesiredYaw-Yaw;
  }

  #ifdef ENABLE_PID_ANGLE_ONLY

  void loopPidAngle(){
    errorPidAngle();
    pidAngleEquation(ErrorRoll, PAngleRoll, IAnglePitch, DAngleRoll, PrevErrorRoll, PrevItermRoll);
        InputRoll=PIDAngleReturn[0];
        PrevErrorRoll=PIDAngleReturn[1]; 
        PrevItermRoll=PIDAngleReturn[2];
    pidAngleEquation(ErrorPitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorPitch, PrevItermPitch);
        InputPitch=PIDAngleReturn[0]; 
        PrevErrorPitch=PIDAngleReturn[1]; 
        PrevItermPitch=PIDAngleReturn[2];
    pidAngleEquation(ErrorYaw, PYaw, IYaw, DYaw, PrevErrorYaw, PrevItermYaw);
        InputYaw=PIDAngleReturn[0]; 
        PrevErrorYaw=PIDAngleReturn[1]; 
        PrevItermYaw=PIDAngleReturn[2];

    InputThrottle = input_throttle;

    if (InputThrottle > 1500) InputThrottle = 1500;
    if (InputRoll > 200) InputRoll = 200;
    else if (InputRoll < -200) InputRoll = -200;
    
    if (InputPitch > 200) InputPitch = 200;
    else if (InputPitch < -200) InputPitch = 200;

    if (InputYaw > 200) InputYaw = 200;
    else if (InputPitch < -200) InputPitch = 200;
    
    // + configuration
    /*      M1
            |
            |
      M4--------- M2
            |
            |
            M3
    */

    MotorInput1= 1.024*(InputThrottle-InputPitch);
    MotorInput2= 1.024*(InputThrottle-InputRoll);
    MotorInput3= 1.024*(InputThrottle+InputPitch);
    MotorInput4= 1.024*(InputThrottle+InputRoll);

    int MaxMotorInput=1700;
    if (MotorInput1 > MaxMotorInput) MotorInput1 = MaxMotorInput;
    if (MotorInput2 > MaxMotorInput) MotorInput2 = MaxMotorInput; 
    if (MotorInput3 > MaxMotorInput) MotorInput3 = MaxMotorInput; 
    if (MotorInput4 > MaxMotorInput) MotorInput4 = MaxMotorInput;

    int ThrottleIdle=1100;
    if (MotorInput1 < ThrottleIdle) MotorInput1 =  ThrottleIdle;
    if (MotorInput2 < ThrottleIdle) MotorInput2 =  ThrottleIdle;
    if (MotorInput3 < ThrottleIdle) MotorInput3 =  ThrottleIdle;
    if (MotorInput4 < ThrottleIdle) MotorInput4 =  ThrottleIdle;

    int ThrottleCutOff=1000;
    if (InputThrottle<1050) {
      MotorInput1=ThrottleCutOff; 
      MotorInput2=ThrottleCutOff;
      MotorInput3=ThrottleCutOff; 
      MotorInput4=ThrottleCutOff;
      resetPidAngle();
    }


    #ifdef ENABLE_PID_ANGLE_PRINT
    Serial.print("PIDAngle| ");
    Serial.print("iT:");Serial.print(InputThrottle); Serial.print(" | ");
    Serial.print("rR:");Serial.print(InputRoll); Serial.print(" | ");
    Serial.print("rP:");Serial.print(InputPitch); Serial.print(" | ");
    Serial.print("rY:");Serial.println(InputYaw);
    
    Serial.print("MotorPID| ");
    Serial.print("M1P:");Serial.print(MotorInput1); Serial.print(" | ");
    Serial.print("M2P:");Serial.print(MotorInput2); Serial.print(" | ");
    Serial.print("M3P:");Serial.print(MotorInput3); Serial.print(" | ");
    Serial.print("M4P:");Serial.print(MotorInput4); Serial.print(" | ");
    Serial.print("M5P:");Serial.print(MotorInput5); Serial.print(" | ");
    Serial.print("M6P:");Serial.println(MotorInput6);
    #endif
  }

  #elif
  void loopPidAngle(){
    errorPidAngle();
    pidAngleEquation(ErrorRoll, PAngleRoll, IAnglePitch, DAngleRoll, PrevErrorRoll, PrevItermRoll);
        RefRateRoll=PIDAngleReturn[0];
        PrevErrorRoll=PIDAngleReturn[1]; 
        PrevItermRoll=PIDAngleReturn[2];
    pidAngleEquation(ErrorPitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorPitch, PrevItermPitch);
        RefRatePitch=PIDAngleReturn[0]; 
        PrevErrorPitch=PIDAngleReturn[1]; 
        PrevItermPitch=PIDAngleReturn[2];
    pidAngleEquation(ErrorYaw, PYaw, IYaw, DYaw, PrevErrorYaw, PrevItermYaw);
        RefRateYaw=PIDAngleReturn[0]; 
        PrevErrorYaw=PIDAngleReturn[1]; 
        PrevItermYaw=PIDAngleReturn[2];

    #ifdef ENABLE_PID_ANGLE_PRINT
    Serial.print("PIDAngle| ");
    // Serial.print("iT:");Serial.print(InputThrottle); Serial.print(" | ");
    Serial.print("rR:");Serial.print(RefRateRoll); Serial.print(" | ");
    Serial.print("rP:");Serial.print(RefRatePitch); Serial.print(" | ");
    Serial.print("rY:");Serial.println(RefRateYaw);
    
    // Serial.print("MotorPID| ");
    // Serial.print("M1P:");Serial.print(MotorInput1); Serial.print(" | ");
    // Serial.print("M2P:");Serial.print(MotorInput2); Serial.print(" | ");
    // Serial.print("M3P:");Serial.print(MotorInput3); Serial.print(" | ");
    // Serial.print("M4P:");Serial.print(MotorInput4); Serial.print(" | ");
    // Serial.print("M5P:");Serial.print(MotorInput5); Serial.print(" | ");
    // Serial.print("M6P:");Serial.println(MotorInput6);
  #endif
  }
  #endif
  
  
  
  
  
#endif

// #ifdef ENABLE_PID_ANGLE_ONLY
//   // Supervisory Control
//   float DesiredRoll, DesiredPitch, DesiredYaw;
//   float ErrorRoll, ErrorPitch, ErrorYaw;
//   float PrevErrorRoll, PrevErrorPitch, PrevErrorYaw;
//   float PrevItermRoll, PrevItermPitch, PrevItermYaw;
//   float PIDAngleReturn[]={0, 0, 0};
//   float PAngleRoll=2 ; float PAnglePitch=PAngleRoll; 
//   float IAnglePitch=0; float IAnglePitch=IRateRoll; 
//   float DAngleRoll=0 ; float DAnglePitch=DAngleRoll; 

//   float PYaw=0; float IYaw= 0; float DYaw=0;
//   // float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
//   // SetPoint Controller PID
//   // float ManualThrottle = input_throttle;
//   float ManualRoll = refRoll;               // deg
//   float ManualPitch = refPitch;              // deg
//   float ManualYaw = refYaw;                // deg

//   void initPidAngle(){
//     // DesiredRateRoll=0.15*(ReceiverValue[0]-1500);
//     DesiredRoll=ManualRoll;
//     // DesiredRatePitch=0.15*(ReceiverValue[1]-1500);
//     DesiredPitch=ManualPitch;
//     // InputThrottle=ReceiverValue[2];
//     // InputThrottle=ManualThrottle;
//     // DesiredRateYaw=0.15*(ReceiverValue[3]-1500);
//     DesiredYaw=ManualYaw;
//   }

//    void pidAngleEquation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
//       float Pterm=P*Error;
//       float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2;
//       if (Iterm > 400) Iterm=400;
//       else if (Iterm <-400) Iterm=-400;
//       float Dterm=D*(Error-PrevError)/0.004;
//       float PIDOutput= Pterm+Iterm+Dterm;
//       if (PIDOutput>400) PIDOutput=400;
//       else if (PIDOutput <-400) PIDOutput=-400;
//       PIDAngleReturn[0]=PIDOutput;
//       PIDAngleReturn[1]=Error;
//       PIDAngleReturn[2]=Iterm;
//   }

//   void resetPidAngle(void) {
//     PrevErrorRoll=0; PrevErrorPitch=0; PrevErrorYaw=0;
//     PrevItermRoll=0; PrevItermPitch=0; PrevItermYaw=0;
//   }

//   void errorPidAngle(){
//     ErrorRoll=DesiredRoll-Roll;
//     ErrorPitch=DesiredPitch-Pitch;
//     ErrorYaw=DesiredYaw-Yaw;
//   }
//   void loopPidAngle(){
//     errorPidAngle();
//     pidAngleEquation(ErrorRoll, PAngleRoll, IAnglePitch, DAngleRoll, PrevErrorRoll, PrevItermRoll);
//         InputRoll=PIDAngleReturn[0];
//         PrevErrorRoll=PIDAngleReturn[1]; 
//         PrevItermRoll=PIDAngleReturn[2];
//     pidAngleEquation(ErrorPitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorPitch, PrevItermPitch);
//         InputPitch=PIDAngleReturn[0]; 
//         PrevErrorPitch=PIDAngleReturn[1]; 
//         PrevItermPitch=PIDAngleReturn[2];
//     pidAngleEquation(ErrorYaw, PYaw, IYaw, DYaw, PrevErrorYaw, PrevItermYaw);
//         InputYaw=PIDAngleReturn[0]; 
//         PrevErrorYaw=PIDAngleReturn[1]; 
//         PrevItermYaw=PIDAngleReturn[2];

//   InputThrottle = input_throttle;

//     if (InputThrottle > 1500) InputThrottle = 1500;

//     // + configuration
//     /*      M1
//             |
//             |
//       M4--------- M2
//             |
//             |
//             M3
//     */

//     MotorInput1= 1.024*(InputThrottle-InputPitch);
//     MotorInput2= 1.024*(InputThrottle-InputRoll);
//     MotorInput3= 1.024*(InputThrottle+InputPitch);
//     MotorInput4= 1.024*(InputThrottle+InputRoll);

//     int MaxMotorInput=1700;
//     if (MotorInput1 > MaxMotorInput)MotorInput1 = MaxMotorInput;
//     if (MotorInput2 > MaxMotorInput)MotorInput2 = MaxMotorInput; 
//     if (MotorInput3 > MaxMotorInput)MotorInput3 = MaxMotorInput; 
//     if (MotorInput4 > MaxMotorInput)MotorInput4 = MaxMotorInput;

//     int ThrottleIdle=1100;
//     if (MotorInput1 < ThrottleIdle) MotorInput1 =  ThrottleIdle;
//     if (MotorInput2 < ThrottleIdle) MotorInput2 =  ThrottleIdle;
//     if (MotorInput3 < ThrottleIdle) MotorInput3 =  ThrottleIdle;
//     if (MotorInput4 < ThrottleIdle) MotorInput4 =  ThrottleIdle;

//     int ThrottleCutOff=1000;
//     if (InputThrottle<1050) {
//       MotorInput1=ThrottleCutOff; 
//       MotorInput2=ThrottleCutOff;
//       MotorInput3=ThrottleCutOff; 
//       MotorInput4=ThrottleCutOff;
//       resetPidRate();
//     }
  
//   #ifdef ENABLE_PID_ANGLE_PRINT
//     Serial.print("PIDAngle| ");
//     Serial.print("iT:");Serial.print(InputThrottle); Serial.print(" | ");
//     Serial.print("rR:");Serial.print(RefRateRoll); Serial.print(" | ");
//     Serial.print("rP:");Serial.print(RefRatePitch); Serial.print(" | ");
//     Serial.print("rY:");Serial.println(RefRateYaw);
    
//     Serial.print("MotorPID| ");
//     Serial.print("M1P:");Serial.print(MotorInput1); Serial.print(" | ");
//     Serial.print("M2P:");Serial.print(MotorInput2); Serial.print(" | ");
//     Serial.print("M3P:");Serial.print(MotorInput3); Serial.print(" | ");
//     Serial.print("M4P:");Serial.print(MotorInput4); Serial.print(" | ");
//     Serial.print("M5P:");Serial.print(MotorInput5); Serial.print(" | ");
//     Serial.print("M6P:");Serial.println(MotorInput6);
//   #endif  

// #endif

#ifdef ENABLE_FUZZY_ROLL
  #include <Fuzzy.h>
  
  //Fuzzy Initial
  Fuzzy *fuzzy = new Fuzzy();

  //INPUT Sudut Roll
  FuzzySet *RollVeryLow = new FuzzySet(-100, -100, -40, -13.33);
  FuzzySet *RollLow = new FuzzySet(-40, -13.33, -13.33, 4.668);
  FuzzySet *RollNormal = new FuzzySet(-10, 0, 0, 10);
  FuzzySet *RollHigh = new FuzzySet(-4.668, 13.33, 13.33, 40);
  FuzzySet *RollVeryHigh = new FuzzySet(13.33, 40, 100, 100);

  //INPUT Kecepatan R
  FuzzySet *SpeedRVeryLow = new FuzzySet(-500, -500, -150, -49.98);
  FuzzySet *SpeedRLow = new FuzzySet(-150, -49.98, -49.98, 17.51);
  FuzzySet *SpeedRNormal = new FuzzySet(-37.5, 0, 0, 37.5);
  FuzzySet *SpeedRHigh = new FuzzySet(-17.51, 49.98, 49.98, 150);
  FuzzySet *SpeedRVeryHigh = new FuzzySet(49.98, 150, 500, 500);

  //OUTPUT PWM
  FuzzySet *RPWMVeryLow = new FuzzySet(-1.5, -1.5, -1, -0.5);
  FuzzySet *RPWMLow = new FuzzySet(-1, -0.5, -0.5, 0);
  FuzzySet *RPWMNormal = new FuzzySet(-0.5, 0, 0, 0.5);
  FuzzySet *RPWMHigh = new FuzzySet(0, 0.5, 0.5, 1);
  FuzzySet *RPWMVeryHigh = new FuzzySet(0.5, 1, 1.5, 1.5);
  // FuzzySet *RPWMVeryLow = new FuzzySet(-30, -30, -20, -10);
  // FuzzySet *RPWMLow = new FuzzySet(-20, -10, -10, 0);
  // FuzzySet *RPWMNormal = new FuzzySet(-10, 0, 0, 10);
  // FuzzySet *RPWMHigh = new FuzzySet(0, 10, 10, 20);
  // FuzzySet *RPWMVeryHigh = new FuzzySet(10, 20, 30, 30);

  float Throttle_Roll, InputRoll, prev_InputRoll, InputThrottle;
  bool StartFuzzyRoll = 0;

  void initFuzzyRoll(){
    refRoll = 0;
    Throttle_Roll = 0;

    // FuzzyInput Jarak Z
    FuzzyInput *RollFuzzy = new FuzzyInput(1);
    RollFuzzy->addFuzzySet(RollVeryLow);
    RollFuzzy->addFuzzySet(RollLow);
    RollFuzzy->addFuzzySet(RollNormal);
    RollFuzzy->addFuzzySet(RollHigh);
    RollFuzzy->addFuzzySet(RollVeryHigh);
    fuzzy->addFuzzyInput(RollFuzzy);

    // FuzzyInput Kecepatan R
    FuzzyInput *SpeedRFuzzy = new FuzzyInput(2);
    SpeedRFuzzy->addFuzzySet(SpeedRVeryLow);
    SpeedRFuzzy->addFuzzySet(SpeedRLow);
    SpeedRFuzzy->addFuzzySet(SpeedRNormal);
    SpeedRFuzzy->addFuzzySet(SpeedRHigh);
    SpeedRFuzzy->addFuzzySet(SpeedRVeryHigh);
    fuzzy->addFuzzyInput(SpeedRFuzzy);

    //FuzzyOutput PWM
    FuzzyOutput *RPWM = new FuzzyOutput(1);
    RPWM->addFuzzySet(RPWMVeryLow);
    RPWM->addFuzzySet(RPWMLow);
    RPWM->addFuzzySet(RPWMNormal);
    RPWM->addFuzzySet(RPWMHigh);
    RPWM->addFuzzySet(RPWMVeryHigh);
    fuzzy->addFuzzyOutput(RPWM);

    //Building FuzzyRule
    //--Roll Very Low
    FuzzyRuleAntecedent* ifRollVeryLowAndSpeedRVeryLow = new FuzzyRuleAntecedent();
    ifRollVeryLowAndSpeedRVeryLow->joinWithAND(RollVeryLow, SpeedRVeryLow);
    FuzzyRuleConsequent *thenRPWMVeryHigh = new FuzzyRuleConsequent(); 
    thenRPWMVeryHigh->addOutput(RPWMVeryHigh); 
    FuzzyRule *rule1 = new FuzzyRule(1, ifRollVeryLowAndSpeedRVeryLow, thenRPWMVeryHigh); 
    fuzzy->addFuzzyRule(rule1);

    FuzzyRuleAntecedent* ifRollVeryLowAndSpeedRLow = new FuzzyRuleAntecedent();
    ifRollVeryLowAndSpeedRLow->joinWithAND(RollVeryLow, SpeedRLow);
    // FuzzyRuleConsequent *thenRPWMVeryHigh = new FuzzyRuleConsequent(); 
    // thenRPWMVeryHigh->addOutput(RPWMVeryHigh); 
    FuzzyRule *rule2 = new FuzzyRule(2, ifRollVeryLowAndSpeedRLow, thenRPWMVeryHigh); 
    fuzzy->addFuzzyRule(rule2);

    FuzzyRuleAntecedent* ifRollVeryLowAndSpeedRNormal = new FuzzyRuleAntecedent();
    ifRollVeryLowAndSpeedRNormal->joinWithAND(RollVeryLow, SpeedRNormal);
    // FuzzyRuleConsequent *thenRPWMVeryHigh = new FuzzyRuleConsequent(); 
    // thenRPWMVeryHigh->addOutput(RPWMVeryHigh); 
    FuzzyRule *rule3 = new FuzzyRule(3, ifRollVeryLowAndSpeedRNormal, thenRPWMVeryHigh); 
    fuzzy->addFuzzyRule(rule3);

    FuzzyRuleAntecedent* ifRollVeryLowAndSpeedRHigh = new FuzzyRuleAntecedent();
    ifRollVeryLowAndSpeedRHigh->joinWithAND(RollVeryLow, SpeedRHigh);
    FuzzyRuleConsequent *thenRPWMHigh = new FuzzyRuleConsequent(); 
    thenRPWMHigh->addOutput(RPWMHigh); 
    FuzzyRule *rule4 = new FuzzyRule(4, ifRollVeryLowAndSpeedRHigh, thenRPWMHigh); 
    fuzzy->addFuzzyRule(rule4);
    
    FuzzyRuleAntecedent* ifRollVeryLowAndSpeedRVeryHigh = new FuzzyRuleAntecedent();
    ifRollVeryLowAndSpeedRVeryHigh->joinWithAND(RollVeryLow, SpeedRVeryHigh);
    FuzzyRuleConsequent *thenRPWMNormal = new FuzzyRuleConsequent(); 
    thenRPWMNormal->addOutput(RPWMNormal); 
    FuzzyRule *rule5 = new FuzzyRule(5, ifRollVeryLowAndSpeedRVeryHigh, thenRPWMNormal); 
    fuzzy->addFuzzyRule(rule5);

    //--Roll Low
    FuzzyRuleAntecedent* ifRollLowAndSpeedRVeryLow = new FuzzyRuleAntecedent();
    ifRollLowAndSpeedRVeryLow->joinWithAND(RollLow, SpeedRVeryLow);
    // FuzzyRuleConsequent *thenRPWMVeryHigh = new FuzzyRuleConsequent(); 
    // thenRPWMVeryHigh->addOutput(RPWMVeryHigh); 
    FuzzyRule *rule6 = new FuzzyRule(6, ifRollLowAndSpeedRVeryLow, thenRPWMVeryHigh); 
    fuzzy->addFuzzyRule(rule6);

    FuzzyRuleAntecedent* ifRollLowAndSpeedRLow = new FuzzyRuleAntecedent();
    ifRollLowAndSpeedRLow->joinWithAND(RollLow, SpeedRLow);
    // FuzzyRuleConsequent *thenRPWMVeryHigh = new FuzzyRuleConsequent(); 
    // thenRPWMVeryHigh->addOutput(RPWMVeryHigh); 
    FuzzyRule *rule7 = new FuzzyRule(7, ifRollLowAndSpeedRLow, thenRPWMVeryHigh); 
    fuzzy->addFuzzyRule(rule7);

    FuzzyRuleAntecedent* ifRollLowAndSpeedRNormal = new FuzzyRuleAntecedent();
    ifRollLowAndSpeedRNormal->joinWithAND(RollLow, SpeedRNormal);
    // FuzzyRuleConsequent *thenRPWMHigh = new FuzzyRuleConsequent(); 
    // thenRPWMHigh->addOutput(RPWMHigh); 
    FuzzyRule *rule8 = new FuzzyRule(8, ifRollLowAndSpeedRNormal, thenRPWMHigh); 
    fuzzy->addFuzzyRule(rule8);

    FuzzyRuleAntecedent* ifRollLowAndSpeedRHigh = new FuzzyRuleAntecedent();
    ifRollLowAndSpeedRHigh->joinWithAND(RollLow, SpeedRHigh);
    // FuzzyRuleConsequent *thenRPWMNormal = new FuzzyRuleConsequent(); 
    // thenRPWMNormal->addOutput(RPWMNormal); 
    FuzzyRule *rule9 = new FuzzyRule(9, ifRollLowAndSpeedRHigh, thenRPWMNormal); 
    fuzzy->addFuzzyRule(rule9);
    
    FuzzyRuleAntecedent* ifRollLowAndSpeedRVeryHigh = new FuzzyRuleAntecedent();
    ifRollLowAndSpeedRVeryHigh->joinWithAND(RollLow, SpeedRVeryHigh);
    FuzzyRuleConsequent *thenRPWMLow = new FuzzyRuleConsequent(); 
    thenRPWMLow->addOutput(RPWMLow); 
    FuzzyRule *rule10 = new FuzzyRule(10, ifRollLowAndSpeedRVeryHigh, thenRPWMLow); 
    fuzzy->addFuzzyRule(rule10);

    //--Roll Normal
    FuzzyRuleAntecedent* ifRollNormalAndSpeedRVeryLow = new FuzzyRuleAntecedent();
    ifRollNormalAndSpeedRVeryLow->joinWithAND(RollNormal, SpeedRVeryLow);
    // FuzzyRuleConsequent *thenRPWMVeryHigh = new FuzzyRuleConsequent(); 
    // thenRPWMVeryHigh->addOutput(RPWMVeryHigh); 
    FuzzyRule *rule11 = new FuzzyRule(11, ifRollNormalAndSpeedRVeryLow, thenRPWMVeryHigh); 
    fuzzy->addFuzzyRule(rule11);

    FuzzyRuleAntecedent* ifRollNormalAndSpeedRLow = new FuzzyRuleAntecedent();
    ifRollNormalAndSpeedRLow->joinWithAND(RollNormal, SpeedRLow);
    // FuzzyRuleConsequent *thenRPWMVeryHigh = new FuzzyRuleConsequent(); 
    // thenRPWMVeryHigh->addOutput(RPWMVeryHigh); 
    FuzzyRule *rule12 = new FuzzyRule(12, ifRollNormalAndSpeedRLow, thenRPWMHigh); 
    fuzzy->addFuzzyRule(rule12);

    FuzzyRuleAntecedent* ifRollNormalAndSpeedRNormal = new FuzzyRuleAntecedent();
    ifRollNormalAndSpeedRNormal->joinWithAND(RollNormal, SpeedRNormal);
    // FuzzyRuleConsequent *thenRPWMVeryHigh = new FuzzyRuleConsequent(); 
    // thenRPWMVeryHigh->addOutput(RPWMVeryHigh); 
    FuzzyRule *rule13 = new FuzzyRule(13, ifRollNormalAndSpeedRNormal, thenRPWMNormal); 
    fuzzy->addFuzzyRule(rule13);

    FuzzyRuleAntecedent* ifRollNormalAndSpeedRHigh = new FuzzyRuleAntecedent();
    ifRollNormalAndSpeedRHigh->joinWithAND(RollNormal, SpeedRHigh);
    // FuzzyRuleConsequent *thenRPWMLow = new FuzzyRuleConsequent(); 
    // thenRPWMLow->addOutput(RPWMLow);
    FuzzyRule *rule14 = new FuzzyRule(14, ifRollNormalAndSpeedRHigh, thenRPWMLow); 
    fuzzy->addFuzzyRule(rule14);

    FuzzyRuleAntecedent* ifRollNormalAndSpeedRVeryHigh = new FuzzyRuleAntecedent();
    ifRollNormalAndSpeedRVeryHigh->joinWithAND(RollNormal, SpeedRVeryHigh);
    FuzzyRuleConsequent *thenRPWMVeryLow = new FuzzyRuleConsequent(); 
    thenRPWMVeryLow->addOutput(RPWMVeryLow); 
    FuzzyRule *rule15 = new FuzzyRule(15, ifRollNormalAndSpeedRVeryHigh, thenRPWMVeryLow); 
    fuzzy->addFuzzyRule(rule15);

    //--Roll High
    FuzzyRuleAntecedent* ifRollHighAndSpeedRVeryLow = new FuzzyRuleAntecedent();
    ifRollHighAndSpeedRVeryLow->joinWithAND(RollHigh, SpeedRVeryLow);
    FuzzyRule *rule16 = new FuzzyRule(16, ifRollHighAndSpeedRVeryLow, thenRPWMHigh); 
    fuzzy->addFuzzyRule(rule16);

    FuzzyRuleAntecedent* ifRollHighAndSpeedRLow = new FuzzyRuleAntecedent();
    ifRollHighAndSpeedRLow->joinWithAND(RollHigh, SpeedRLow);
    FuzzyRule *rule17 = new FuzzyRule(17, ifRollHighAndSpeedRLow, thenRPWMNormal); 
    fuzzy->addFuzzyRule(rule17);

    FuzzyRuleAntecedent* ifRollHighAndSpeedRNormal = new FuzzyRuleAntecedent();
    ifRollHighAndSpeedRNormal->joinWithAND(RollHigh, SpeedRNormal);
    FuzzyRule *rule18 = new FuzzyRule(18, ifRollHighAndSpeedRNormal, thenRPWMLow); 
    fuzzy->addFuzzyRule(rule18);

    FuzzyRuleAntecedent* ifRollHighAndSpeedRHigh = new FuzzyRuleAntecedent();
    ifRollHighAndSpeedRHigh->joinWithAND(RollHigh, SpeedRHigh);
    FuzzyRule *rule19 = new FuzzyRule(19, ifRollHighAndSpeedRHigh, thenRPWMVeryLow); 
    fuzzy->addFuzzyRule(rule19);

    FuzzyRuleAntecedent* ifRollHighAndSpeedRVeryHigh = new FuzzyRuleAntecedent();
    ifRollHighAndSpeedRVeryHigh->joinWithAND(RollHigh, SpeedRVeryHigh);
    FuzzyRule *rule20 = new FuzzyRule(20, ifRollHighAndSpeedRVeryHigh, thenRPWMVeryLow); 
    fuzzy->addFuzzyRule(rule20);

    //--Roll Very High
    FuzzyRuleAntecedent* ifRollVeryHighAndSpeedRVeryLow = new FuzzyRuleAntecedent();
    ifRollVeryHighAndSpeedRVeryLow->joinWithAND(RollVeryHigh, SpeedRVeryLow);
    FuzzyRule *rule21 = new FuzzyRule(21, ifRollVeryHighAndSpeedRVeryLow, thenRPWMNormal); 
    fuzzy->addFuzzyRule(rule21);

    FuzzyRuleAntecedent* ifRollVeryHighAndSpeedRLow = new FuzzyRuleAntecedent();
    ifRollVeryHighAndSpeedRLow->joinWithAND(RollVeryHigh, SpeedRLow); 
    FuzzyRule *rule22 = new FuzzyRule(22, ifRollVeryHighAndSpeedRLow, thenRPWMLow); 
    fuzzy->addFuzzyRule(rule22);

    FuzzyRuleAntecedent* ifRollVeryHighAndSpeedRNormal = new FuzzyRuleAntecedent();
    ifRollVeryHighAndSpeedRNormal->joinWithAND(RollVeryHigh, SpeedRNormal);
    FuzzyRule *rule23 = new FuzzyRule(23, ifRollVeryHighAndSpeedRNormal, thenRPWMVeryLow); 
    fuzzy->addFuzzyRule(rule23);

    FuzzyRuleAntecedent* ifRollVeryHighAndSpeedRHigh = new FuzzyRuleAntecedent();
    ifRollVeryHighAndSpeedRHigh->joinWithAND(RollVeryHigh, SpeedRHigh);
    FuzzyRule *rule24 = new FuzzyRule(24, ifRollVeryHighAndSpeedRHigh, thenRPWMVeryLow); 
    fuzzy->addFuzzyRule(rule24);

    FuzzyRuleAntecedent* ifRollVeryHighAndSpeedRVeryHigh = new FuzzyRuleAntecedent();
    ifRollVeryHighAndSpeedRVeryHigh->joinWithAND(RollVeryHigh, SpeedRVeryHigh); 
    FuzzyRule *rule25 = new FuzzyRule(25, ifRollVeryHighAndSpeedRVeryHigh, thenRPWMVeryLow); 
    fuzzy->addFuzzyRule(rule25);
  }

  void runFuzzyRoll(){
    fuzzy->setInput(1, Roll - refRoll);
    fuzzy->setInput(2, RateRoll);
    fuzzy->fuzzify(); 
    if (StartFuzzyRoll == 1){
      Throttle_Roll += fuzzy->defuzzify(1);
    }
    // InputRoll = prev_InputRoll + Throttle_Roll;
    
    InputThrottle = input_throttle;

    if (InputThrottle > 1500) InputThrottle = 1500;

    // + configuration
    /*      M1
            |
            |
      M4--------- M2
            |
            |
            M3
    */

    MotorInput1= InputThrottle;
    MotorInput2= InputThrottle - Throttle_Roll;
    MotorInput3= InputThrottle;
    MotorInput4= InputThrottle + Throttle_Roll;

    int MaxMotorInput=1700;
    if (MotorInput1 > MaxMotorInput)MotorInput1 = MaxMotorInput;
    if (MotorInput2 > MaxMotorInput)MotorInput2 = MaxMotorInput; 
    if (MotorInput3 > MaxMotorInput)MotorInput3 = MaxMotorInput; 
    if (MotorInput4 > MaxMotorInput)MotorInput4 = MaxMotorInput;

    int ThrottleIdle=1100;
    if (MotorInput1 < ThrottleIdle) MotorInput1 =  ThrottleIdle;
    if (MotorInput2 < ThrottleIdle) MotorInput2 =  ThrottleIdle;
    if (MotorInput3 < ThrottleIdle) MotorInput3 =  ThrottleIdle;
    if (MotorInput4 < ThrottleIdle) MotorInput4 =  ThrottleIdle;

    int ThrottleCutOff=1000;
      if (InputThrottle<1050) {
        MotorInput1=ThrottleCutOff; 
        MotorInput2=ThrottleCutOff;
        MotorInput3=ThrottleCutOff; 
        MotorInput4=ThrottleCutOff;
        resetFuzzyRoll();
      }
    Serial.print("IMU DATA | ");
    Serial.print("Gx:");Serial.print(RateRoll); Serial.print(" | ");
    Serial.print("Gy:");Serial.print(RatePitch); Serial.print(" | ");
    Serial.print("Gz:");Serial.print(RateYaw); Serial.print(" | ");
    Serial.print("R:");Serial.print(Roll); Serial.print(" | ");
    Serial.print("P:");Serial.print(Pitch); Serial.print(" | ");
    Serial.print("Y:");Serial.println(Yaw);

    Serial.print("Fuzzy Roll | ");
    Serial.print("Error Roll : "); Serial.print(Roll - refRoll); Serial.print(" | ");
    Serial.print("iR:");Serial.print(Throttle_Roll); Serial.print(" | ");

    Serial.print("MotorFuzzy | ");
    Serial.print("M1P:");Serial.print(MotorInput1); Serial.print(" | ");
    Serial.print("M2P:");Serial.print(MotorInput2); Serial.print(" | ");
    Serial.print("M3P:");Serial.print(MotorInput3); Serial.print(" | ");
    Serial.print("M4P:");Serial.print(MotorInput4); Serial.println(" | ");
  }
  
  void resetFuzzyRoll(){
    Throttle_Roll = 0;
    MotorInput1 = InputThrottle;
    MotorInput2 = InputThrottle;
    MotorInput3 = InputThrottle;
    MotorInput4 = InputThrottle;
  }
#endif

// 7. RTC BuiltIn
uint8_t currentHour, currentMinute, currentSecond;    // Global Variable for ENABLE_SD_CARD
#ifdef ENABLE_RTC
  #include <TimeLib.h>
  

  void initRtc(){
    if (Teensy3Clock.get()) {
      setSyncProvider(Teensy3Clock.get); // Sinkronkan waktu dengan RTC
      if (timeStatus() == timeSet) {
        Serial.println("RTC time successfully synced!");
      } 
      else {
        Serial.println("RTC time is invalid.");
      }
    }
    else {
      Serial.println("RTC not initialized, setting default time...");
      setTime(10, 15, 30, 16, 11, 2024); // Atur waktu default jika RTC belum diatur
      Teensy3Clock.set(now());           // Sinkronkan RTC dengan waktu default
      Serial.println("Default time set to RTC.");
    }
    // Tampilkan waktu saat ini
  }

  void rtc(){
    currentHour = hour();
    currentMinute = minute();
    currentSecond = second();

    #ifdef ENABLE_RTC_PRINT
      Serial.print("RTC| ");
      Serial.print(currentHour);Serial.print(":"); 
      Serial.print(currentMinute);Serial.print(":");
      Serial.println(currentSecond);
    #endif
  }

#endif

// 6. SD CARD BuiltIn
#ifdef ENABLE_SD_CARD
  #include <SdFat.h>
  #define SD_CONFIG SdioConfig(FIFO_SDIO)
  #define LOG_FILENAME "imu_lidar_rtc_3.csv"
  const char *note = ", Pengujian RTC Integrated\n";
  const size_t BUFFER_SIZE = 100 * 500 * 5;  // buffer = char per iteration * Hz * Second
  char buffer[BUFFER_SIZE];        // Temporary buffer for storing data
  size_t bufferIndex = 0;          // Current position in the buffer

  SdFs sd;
  FsFile file;

  void initSd(){
    if (!sd.begin(SD_CONFIG)) {
      Serial.println("Failed to initialize SD card!");
      while (1) {}
    }
    Serial.println("SD card initialized.");

    if (!sd.exists(LOG_FILENAME)) {
      // Jika file belum ada, buat file baru dan tulis header
      file = sd.open(LOG_FILENAME, O_RDWR | O_CREAT | O_APPEND);
      // Write CSV header
      const char *fileName = LOG_FILENAME;
      file.write(fileName);
      file.write(note);
      const char *header = "Hours,Minute,Second,Timestamp(ms),Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z\n";
      file.write(header);
      file.sync();
      file.close();
    } else {
      Serial.println("File already exists. Appending data...");
    }

    file = sd.open(LOG_FILENAME, O_RDWR | O_APPEND);
    if (!file) {
      Serial.println("Failed to open file for appending!");
      while (1) {}
    }
    file.write("\nSTART LOGGING\n");
  }

  void logData() {
    unsigned long timestamp = millis();
    int len = snprintf(buffer + bufferIndex, BUFFER_SIZE - bufferIndex,
                      "%d,%d,%d,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n",
                      currentHour,currentMinute,currentSecond,timestamp,
                      RateRoll, RatePitch, RateYaw, Roll, Pitch, Yaw, distance);

    // Check if the buffer has enough space
    if (bufferIndex + len >= BUFFER_SIZE) {
      // Write buffer to SD card
      file.write(buffer, bufferIndex);
      file.sync();
      bufferIndex = 0;  // Reset buffer index
      memset(buffer, 0, BUFFER_SIZE);

    }
    // Append the current data to the buffer
    bufferIndex += len;
  }

#endif

// 8.1. rpm Sensor 1

  float rpm_1 = 0;
  volatile unsigned long pulseInterval_1 = 0;
#ifdef ENABLE_RPM_1
  #include <IntervalTimer.h>
  IntervalTimer rpmTimer_1;
  volatile unsigned long lastPulseTime_1 = 0;

  #define inputRPM_1 6 
  
  FASTRUN void pulseHandler_1() {
  unsigned long currentTime_1 = micros(); // Current time in microseconds
  pulseInterval_1 = currentTime_1 - lastPulseTime_1; // Time between pulses
  lastPulseTime_1 = currentTime_1;
  }

  void initRPM_1(){
    pinMode(inputRPM_1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(inputRPM_1), pulseHandler_1, RISING); // Detect rising edges
    rpmTimer_1.begin(calculateRPM_1, 10000); // Call calculateRPM every 100ms
  }

  void calculateRPM_1() {
  if (pulseInterval_1 > 0 && ((unsigned long)micros()) - lastPulseTime_1 <= 20000) {
    float frequency_1 = 1000000.0  / pulseInterval_1; // Convert µs interval to frequency (Hz)
    rpm_1 = (frequency_1 * 60.0) / 7;

    // pada fungsi interupt, hindari penggunaan serial print
    // Serial.print("RPM: ");
    // Serial.println(rpm);
    // Serial.print(" | Interval: ");
    // Serial.print(pulseInterval);
  } else if (((unsigned long)micros()) - lastPulseTime_1 > 20000) {
    // Serial.println("No pulses detected");
    rpm_1 = 0;
  }
  }

  void printRpm_1(){
    Serial.print("RPM 1: ");
    Serial.print(rpm_1);
    Serial.print(" | Interval 1: ");
    Serial.println(pulseInterval_1);
  } 
#endif

// 8.2. rpm Sensor 2

  float rpm_2 = 0;
  volatile unsigned long pulseInterval_2 = 0;
#ifdef ENABLE_RPM_2
  #include <IntervalTimer.h>
  IntervalTimer rpmTimer_2;
  volatile unsigned long lastPulseTime_2 = 0;

  #define inputRPM_2 7 
  
  FASTRUN void pulseHandler_2() {
  unsigned long currentTime_2 = micros(); // Current time in microseconds
  pulseInterval_2 = currentTime_2 - lastPulseTime_2; // Time between pulses
  lastPulseTime_2 = currentTime_2;
  }

  void initRPM_2(){
    pinMode(inputRPM_2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(inputRPM_2), pulseHandler_2, RISING); // Detect rising edges
    rpmTimer_2.begin(calculateRPM_2, 10000); // Call calculateRPM every 100ms
  }

  void calculateRPM_2() {
  if (pulseInterval_2 > 0 && ((unsigned long)micros()) - lastPulseTime_2 <= 20000) {
    float frequency_2 = 1000000.0  / pulseInterval_2; // Convert µs interval to frequency (Hz)
    rpm_2 = (frequency_2 * 60.0) / 7;

    // pada fungsi interupt, hindari penggunaan serial print
    // Serial.print("RPM: ");
    // Serial.println(rpm);
    // Serial.print(" | Interval: ");
    // Serial.print(pulseInterval);
  } else if (((unsigned long)micros()) - lastPulseTime_2 > 20000) {
    // Serial.println("No pulses detected");
    rpm_2 = 0;
  }
  }

  void printRpm_2(){
    Serial.print("RPM 2: ");
    Serial.print(rpm_2);
    Serial.print(" | Interval 2: ");
    Serial.println(pulseInterval_2);
  } 
#endif

// 8.3. rpm Sensor 3

  float rpm_3 = 0;
  volatile unsigned long pulseInterval_3 = 0;
#ifdef ENABLE_RPM_3
  #include <IntervalTimer.h>
  IntervalTimer rpmTimer_3;
  volatile unsigned long lastPulseTime_3 = 0;

  #define inputRPM_3 8 
  
  FASTRUN void pulseHandler_3() {
  unsigned long currentTime_3 = micros(); // Current time in microseconds
  pulseInterval_3 = currentTime_3 - lastPulseTime_3; // Time between pulses
  lastPulseTime_3 = currentTime_3;
  }

  void initRPM_3(){
    pinMode(inputRPM_3, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(inputRPM_3), pulseHandler_3, RISING); // Detect rising edges
    rpmTimer_3.begin(calculateRPM_3, 10000); // Call calculateRPM every 100ms
  }

  void calculateRPM_3() {
  if (pulseInterval_3 > 0 && ((unsigned long)micros()) - lastPulseTime_3 <= 20000) {
    float frequency_3 = 1000000.0  / pulseInterval_3; // Convert µs interval to frequency (Hz)
    rpm_3 = (frequency_3 * 60.0) / 7;

    // pada fungsi interupt, hindari penggunaan serial print
    // Serial.print("RPM: ");
    // Serial.println(rpm);
    // Serial.print(" | Interval: ");
    // Serial.print(pulseInterval);
  } else if (((unsigned long)micros()) - lastPulseTime_3 > 20000) {
    // Serial.println("No pulses detected");
    rpm_3 = 0;
  }
  }

  void printRpm_3(){
    Serial.print("RPM 3 : ");
    Serial.print(rpm_3);
    Serial.print(" | Interval 3 : ");
    Serial.println(pulseInterval_3);
  } 
#endif

// 8.4. rpm Sensor 4

  float rpm_4 = 0;
  volatile unsigned long pulseInterval_4 = 0;
#ifdef ENABLE_RPM_4
  #include <IntervalTimer.h>
  IntervalTimer rpmTimer_4;
  volatile unsigned long lastPulseTime_4 = 0;

  #define inputRPM_4 9 
  
  FASTRUN void pulseHandler_4() {
  unsigned long currentTime_4 = micros(); // Current time in microseconds
  pulseInterval_4 = currentTime_4 - lastPulseTime_4; // Time between pulses
  lastPulseTime_4 = currentTime_4;
  }

  void initRPM_4(){
    pinMode(inputRPM_4, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(inputRPM_4), pulseHandler_4, RISING); // Detect rising edges
    rpmTimer_4.begin(calculateRPM_4, 10000); // Call calculateRPM every 100ms
  }

  void calculateRPM_4() {
  if (pulseInterval_4 > 0 && ((unsigned long)micros()) - lastPulseTime_4 <= 20000) {
    float frequency_4 = 1000000.0  / pulseInterval_4; // Convert µs interval to frequency (Hz)
    rpm_4 = (frequency_4 * 60.0) / 7;

    // pada fungsi interupt, hindari penggunaan serial print
    // Serial.print("RPM: ");
    // Serial.println(rpm);
    // Serial.print(" | Interval: ");
    // Serial.print(pulseInterval);
  } else if (((unsigned long)micros()) - lastPulseTime_4 > 20000) {
    // Serial.println("No pulses detected");
    rpm_4 = 0;
  }
  }

  void printRpm_4(){
    Serial.print("RPM 4 : ");
    Serial.print(rpm_4);
    Serial.print(" | Interval 4 : ");
    Serial.println(pulseInterval_4);
  } 
#endif

#ifdef ENABLE_CUSTOM

float dRefRoll = 5; // deg/s
float dt = 0.01;          // 10ms

// volatile float RatePitch, RateRoll, RateYaw;
// float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw,AccXCalibration,AccYCalibration,AccZCalibration;

float PAngleRoll=0; float PAnglePitch=2;
float IAngleRoll=0; float IAnglePitch=0.5;
float DAngleRoll=0; float DAnglePitch=0.007;

float PRateRoll = 0;
float IRateRoll = 0;
float DRateRoll = 0;

float PRatePitch = 0.625;
float IRatePitch = 0.01;
float DRatePitch = 0.0088;

// float PAngleRoll=6; float PAnglePitch=5;
// float IAngleRoll=0.1; float IAnglePitch=1;
// float DAngleRoll=0.005; float DAnglePitch=0.005;

// float PRateRoll = 0.852;
// float IRateRoll = 0.1;
// float DRateRoll = 0.005;

// float PRatePitch = 0.6;
// float IRatePitch = 0;
// float DRatePitch = 0;

float PRateYaw = 0;
float IRateYaw = 0;
float DRateYaw = 0;

volatile float PtermRoll; 
volatile float ItermRoll; 
volatile float DtermRoll; 
volatile float PIDOutputRoll; 
volatile float PtermPitch; 
volatile float ItermPitch; 
volatile float DtermPitch; 
volatile float PIDOutputPitch; 
volatile float PtermYaw; 
volatile float ItermYaw; 
volatile float DtermYaw; 
volatile float PIDOutputYaw; 

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[] = {0, 0, 0};

float setPointRoll = 0, setPointPitch = 0;
volatile float DesiredAngleRoll = setPointRoll, DesiredAnglePitch = setPointPitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+dt*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + dt * dt * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}


void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
  float Pterm = P * Error;
  float Iterm = PrevIterm +( I * (Error + PrevError) * (dt/2));
  if (Iterm > 400)
  {
    Iterm = 400;
  }
  else if (Iterm < -400)
  {
  Iterm = -400;
  }
  float Dterm = D *( (Error - PrevError)/dt);
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400)
  {
    PIDOutput = 400;
  }
  else if (PIDOutput < -400)
  {
    PIDOutput = -400;
  }
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void resetPidAngleRate(){
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
  PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;    
  PrevItermAngleRoll=0; PrevItermAnglePitch=0;
}

void loopPidAngleRate(){
  if (setPointRoll < refRoll) {
    // Increase setpoint towards reference
    setPointRoll += dRefRoll * dt;
    
    // Ensure setPointRoll does not exceed refRoll
    if (setPointRoll > refRoll) {
      setPointRoll = refRoll;  // Cap at the refRoll
    }
  }
  else if (setPointRoll > refRoll) {
    // Decrease setpoint towards reference
    setPointRoll -= dRefRoll * dt;

    // Ensure setPointRoll does not drop below refRoll
    if (setPointRoll < refRoll) {
      setPointRoll = refRoll;  // Cap at the refRoll
    }
  }

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, Roll);
  KalmanAngleRoll=Kalman1DOutput[0]; KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, Pitch);
  KalmanAnglePitch=Kalman1DOutput[0]; KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

  DesiredAngleRoll = setPointRoll;
  DesiredAnglePitch = refPitch;
  ErrorAngleRoll = DesiredAngleRoll - Roll;
  PtermRoll = PAngleRoll * ErrorAngleRoll;
  ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (dt / 2));
  ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
  DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / dt);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
  DesiredRateRoll = PIDOutputRoll;
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;


  ErrorAnglePitch = DesiredAnglePitch - Pitch;
  PtermPitch = PAnglePitch * ErrorAnglePitch;
  ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (dt / 2));
  ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
  DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / dt);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  // Compute errors
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  // Roll Axis PID
  PtermRoll = PRateRoll * ErrorRateRoll;
  ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (dt / 2));
  ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
  DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / dt);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);

  // Update output and previous values for Roll
  InputRoll = PIDOutputRoll;
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = ItermRoll;

  // Pitch Axis PID
  PtermPitch = PRatePitch * ErrorRatePitch;
  ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (dt / 2));
  ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
  DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / dt);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);

  // Update output and previous values for Pitch
  InputPitch = PIDOutputPitch;
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;

  // Yaw Axis PID
  PtermYaw = PRateYaw * ErrorRateYaw;
  ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (dt / 2));
  ItermYaw = (ItermYaw > 400) ? 400 : ((ItermYaw < -400) ? -400 : ItermYaw);  // Clamp ItermYaw to [-400, 400]
  DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / dt);
  PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
  PIDOutputYaw = (PIDOutputYaw > 400) ? 400 : ((PIDOutputYaw < -400) ? -400 : PIDOutputYaw);  // Clamp PIDOutputYaw to [-400, 400]

  // Update output and previous values for Yaw
  InputYaw = PIDOutputYaw;
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = ItermYaw;

  InputThrottle = input_throttle;
  
  if (InputThrottle > 1800)
  {
    InputThrottle = 1800;
  }

  
  MotorInput1 =  (InputThrottle + InputPitch); // front right - counter clockwise
  MotorInput2 =  (InputThrottle - InputRoll); // rear right - clockwise
  MotorInput3 =  (InputThrottle - InputPitch); // rear left  - counter clockwise
  MotorInput4 =  (InputThrottle + InputRoll); //front left - clockwise

  if (MotorInput1 > 2000)
  {
    MotorInput1 = 1999;
  }

  if (MotorInput2 > 2000)
  {
    MotorInput2 = 1999;
  }

  if (MotorInput3 > 2000)
  {
    MotorInput3 = 1999;
  }

  if (MotorInput4 > 2000)
  {
    MotorInput4 = 1999;
  }
       
        
int ThrottleIdle = 1100;
int ThrottleCutOff = 1000;
  if (MotorInput1 < ThrottleIdle)
  {
    MotorInput1 = ThrottleIdle;
  }
  if (MotorInput2 < ThrottleIdle)
  {
    MotorInput2 = ThrottleIdle;
  }
  if (MotorInput3 < ThrottleIdle)
  {
    MotorInput3 = ThrottleIdle;
  }
  if (MotorInput4 < ThrottleIdle)
  {
    MotorInput4 = ThrottleIdle;
  }

  // int ThrottleCutOff=1000;
    if (InputThrottle<1050) {
      MotorInput1=ThrottleCutOff; 
      MotorInput2=ThrottleCutOff;
      MotorInput3=ThrottleCutOff; 
      MotorInput4=ThrottleCutOff;
      resetPidAngleRate();
    }

  #ifdef ENABLE_CUSTOM_PRINT
    Serial.print("PIDOut| ");
    Serial.print("PTR:");Serial.print(PtermRoll,3); Serial.print(" | ");
    Serial.print("ITR:");Serial.print(ItermRoll,3); Serial.print(" | ");
    Serial.print("DTR:");Serial.print(DtermRoll,3); Serial.print("           |       ");
    Serial.print("PTP:");Serial.print(PtermPitch,3); Serial.print(" | ");
    Serial.print("ITP:");Serial.print(ItermPitch,3); Serial.print(" | ");
    Serial.print("DTP:");Serial.println(DtermPitch,3);

    Serial.print("PIDGain| ");
    Serial.print("A_PPitch: ");Serial.print(PAnglePitch,3); Serial.print(" | ");
    Serial.print("A_IPitch: ");Serial.print(IAnglePitch,3); Serial.print(" | ");
    Serial.print("A_DPitch: ");Serial.print(DAnglePitch,3); Serial.print(" | ");
    Serial.print("R_PPitch: ");Serial.print(PRatePitch,3); Serial.print(" | ");
    Serial.print("R_IPitch: ");Serial.print(IRatePitch,3); Serial.print(" | ");
    Serial.print("R_DPitch: ");Serial.println(DRatePitch,3);
    
    Serial.print("PIDAngleRate| ");
    Serial.print("KalmanR");Serial.print(KalmanAngleRoll); Serial.print(" | ");
    Serial.print("KalmanP");Serial.print(KalmanAnglePitch); Serial.print(" | ");
    Serial.print("rR");Serial.print(DesiredAngleRoll); Serial.print(" | ");
    Serial.print("rP");Serial.print(DesiredAnglePitch); Serial.print(" | ");
    Serial.print("rGx");Serial.print(DesiredRateRoll); Serial.print(" | ");
    Serial.print("rGy");Serial.print(DesiredRatePitch); Serial.print(" | ");
    Serial.print("iT:");Serial.print(InputThrottle); Serial.print(" | ");
    Serial.print("iR:");Serial.print(InputRoll); Serial.print(" | ");
    Serial.print("iP:");Serial.print(InputPitch); Serial.print(" | ");
    Serial.print("iY:");Serial.println(InputYaw);
    
    Serial.print("MotorPID| ");
    Serial.print("M1P:");Serial.print(MotorInput1); Serial.print(" | ");
    Serial.print("M2P:");Serial.print(MotorInput2); Serial.print(" | ");
    Serial.print("M3P:");Serial.print(MotorInput3); Serial.print(" | ");
    Serial.print("M4P:");Serial.println(MotorInput4);
    // Serial.print("M5P:");Serial.print(MotorInput5); Serial.print(" | ");
    // Serial.print("M6P:");Serial.print(MotorInput6); Serial.print(" | ");



  #endif
}
#endif

// 9. Receiver
#ifdef ENABLE_RECEIVER

  #include <PulsePosition.h>
  #define receiverPin 9
  PulsePositionInput ReceiverInput(RISING);
  float ReceiverValue[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  int ChannelNumber = 0;
  float InputThrottle = 0;

  void initReceiver(){
    ReceiverInput.begin(receiverPin);
    while (ReceiverValue[2] < 1005 ||
    ReceiverValue[2] > 1050) {
      readReceiver();
      Serial.println(ReceiverValue[2]);
      delay(4);
    }
  }

  void readReceiver() {
  ChannelNumber = ReceiverInput.available();
    if (ChannelNumber > 0) {
      for (int i = 1; i <= ChannelNumber; i++) {
      ReceiverValue[i - 1] = ReceiverInput.read(i);
      }
    }
  input_throttle=ReceiverValue[2];
  }

#endif

// 10. User input for Throttle
#ifdef ENABLE_USER_INPUT      
  int data;
  int timerCounter;
  void displayInstructions(){  
    Serial.println("\n\nUSER INPUT INTEGER on SERIAL MONITOR");
    Serial.println("    18  \tSending min throttle");
    Serial.println("    1  \tSending max throttle");
    Serial.println("    2  \tRunning test");
    Serial.println("    3  \tThrottle : 1100");
    Serial.println("    4  \tThrottle : 1200");
    Serial.println("    5  \tThrottle : 1300");
    Serial.println("    6  \tThrottle Add + 50us");
    Serial.println("    7  \tThrottle Subtract - 50us");
    Serial.println("    8  \tThrottle Down-3S");
    Serial.println("    9  \tThrottle Down-2S");
    Serial.println("   10  \tSP_RPM: 1000");
    Serial.println("   11  \tSP_RPM: 3000");
    Serial.println("   12  \tSP_RPM: 5000");
    Serial.println("   13  \tSP_RPM: 7000");
    Serial.println("   14  \tSP_RPM: +500");
    Serial.println("   15  \tSP_RPM: -500");
    Serial.println("   16  \tSP_RPM: +1000");
    Serial.println("   17  \tSP_RPM: -1000");
    Serial.println("   19 (4 digit PWM) \tSend Throttle to Motor 1 Manual");
    Serial.println("   20 (4 digit PWM) \tSend Throttle to Motor 2 Manual");
    Serial.println("   21 (4 digit PWM) \tSend Throttle to Motor 3 Manual");
    Serial.println("   22 (4 digit PWM) \tSend Throttle to Motor 4 Manual");
    Serial.println("   23 ref Roll (eg/ 23 10  --> ref roll 10 deg");
    Serial.println("   24 Start Fuzzy Roll Algorithm\n\n");
    Serial.println("   25 PAngleRoll\n\n");
    Serial.println("   26 PAnglePitch\n\n");
    Serial.println("   27 PAngleYaw\n\n");
  }

  void userInput(){
    float temp;
    if (Serial.available()) {
      
        data = Serial.parseInt();
        switch (data) {
            // 0 char  == 48 ascii
            case 18 : Serial.println("\nSending minimum throttle\n");
                      input_throttle = MIN_PULSE_LENGTH;
                      Serial.print("input_throttle min: ");Serial.println(input_throttle);
            break;

            // 1
            case 1 : Serial.println("\nSending maximum throttle\n");
                      // input_throttle = MAX_PULSE_LENGTH;
                      Serial.print("input_throttle max: "); Serial.println(input_throttle);
            break;

            // 2
            case 2 : Serial.println("\nRunning test in 3\n");
                      delay(1000);
                      Serial.print(" 2");
                      delay(1000);
                      Serial.println(" 1...");
                      delay(1000);
                      test();
            break;
            // 3
            case 3 : Serial.println("\nThrottle : 1100\n");
                      input_throttle = 1100;
                      Serial.print("input_throttle: "); Serial.println(input_throttle);
            break;
            // 4
            case 4 : Serial.println("\nThrottle : 1200\n");
                      input_throttle = 1200;
                      Serial.print("input_throttle: "); Serial.println(input_throttle);
            break;
            // 5
            case 5 : Serial.println("\nThrottle : 1300\n");
                      input_throttle = 1300;
                      Serial.print("input_throttle: "); Serial.println(input_throttle);
            break;
            // 6
            case 6 : Serial.println("\nThrottle Add + 50us\n");
                      input_throttle = input_throttle + 50;
                      Serial.print("input_throttle: "); Serial.println(input_throttle);
            break;
            // 7
            case 7 : Serial.println("\nThrottle  Subtract - 50us\n");
                      input_throttle = input_throttle - 50;
                      Serial.print("input_throttle: "); Serial.println(input_throttle);
            break;
            // 8
            case 8 : Serial.println("\nThrottle  Down-3S\n");
                      throttle_down_3s();
            break;
            // 9
            case 9 : Serial.println("\nThrottle Down-2S\n");
                      throttle_down_2s();
            break;
            //10
            case 10 : Serial.println("SP_RPM: 1000");
                      setPointRpm = 1000;
            break;
            //11
            case 11 : Serial.println("SP_RPM: 3000");
                      setPointRpm = 3000;
            break;
            //12
            case 12 : Serial.println("SP_RPM: 5000");
                      setPointRpm = 5000;
            break;
            //13
            case 13 : Serial.println("SP_RPM: 7000");
                      setPointRpm = 7000;
            break;
            //14
            case 14 : Serial.println("SP_RPM: +500");
                      setPointRpm =+ 500;
            break;
            //15
            case 15 : Serial.println("SP_RPM: -500");
                      setPointRpm =- 500;
            break;
            //16
            case 16 : Serial.println("SP_RPM: +1000");
                      setPointRpm =+ 1000;
            break;
            //17
            case 17 : Serial.println("SP_RPM: -1000");
                      setPointRpm =- 1000;
            break;
            case 19:
                Serial.println("\nSet Throttle for Motor 1\n");
                Serial.println("Enter value (e.g., 1100):");
                while (!Serial.available());
                motor1_throttle = Serial.parseInt();
                // Konfirmasi nilai throttle yang dimasukkan
                if (motor1_throttle > 999 && motor1_throttle < 2001) {
                  Serial.print("Motor 1 Throttle set to: "); 
                  Serial.println(motor1_throttle);
                } else {
                  Serial.println("Invalid input. Please enter a valid number.");
                }
                // Serial.print("Motor 1 Throttle set to: "); Serial.println(motor1_throttle);
                // runMotor1(motor1_throttle); // Fungsi untuk motor 1
                break;

            case 20:
                Serial.println("\nSet Throttle for Motor 2\n");
                Serial.println("Enter value (e.g., 1200):");
                while (!Serial.available());
                motor2_throttle = Serial.parseInt();
                if (motor2_throttle > 999 && motor2_throttle < 2001) {
                  Serial.print("Motor 2 Throttle set to: "); 
                  Serial.println(motor2_throttle);
                } else {
                  Serial.println("Invalid input. Please enter a valid number.");
                }
                // Serial.print("Motor 2 Throttle set to: "); Serial.println(motor2_throttle);
                // runMotor2(motor2_throttle); // Fungsi untuk motor 2
                break;

            case 21:
                Serial.println("\nSet Throttle for Motor 3\n");
                Serial.println("Enter value (e.g., 1300):");
                while (!Serial.available());
                motor3_throttle = Serial.parseInt();
                if (motor3_throttle > 999 && motor3_throttle < 2001) {
                  Serial.print("Motor 3 Throttle set to: "); 
                  Serial.println(motor3_throttle);
                } else {
                  Serial.println("Invalid input. Please enter a valid number.");
                }
                // Serial.print("Motor 3 Throttle set to: "); Serial.println(motor3_throttle);
                // runMotor3(motor3_throttle); // Fungsi untuk motor 3
                break;

            case 22:
                Serial.println("\nSet Throttle for Motor 4\n");
                Serial.println("Enter value (e.g., 1400):");
                while (!Serial.available());
                motor4_throttle = Serial.parseInt();
                if (motor4_throttle > 999 && motor4_throttle < 2001) {
                  Serial.print("Motor 4 Throttle set to: "); 
                  Serial.println(motor4_throttle);
                } else {
                  Serial.println("Invalid input. Please enter a valid number.");
                }
                // Serial.print("Motor 4 Throttle set to: "); Serial.println(motor4_throttle);
                // runMotor4(motor4_throttle); // Fungsi untuk motor 4
                break;
            case 23:
                while (!Serial.available());
                refRoll = Serial.parseInt();
                if (refRoll > -46 && refRoll < 46) {
                  Serial.print("Ref ROll SET TO: "); 
                  Serial.println(refRoll);
                } else {
                  Serial.println("Invalid input. Please enter a valid number.");
                }
                break;
            case 24:
                while (!Serial.available());
                #ifdef ENABLE_FUZZY_ROLL
                  StartFuzzyRoll = Serial.parseInt();
                #else
                  bool StartFuzzyRoll;
                #endif

                StartFuzzyRoll = Serial.parseInt();
                if (StartFuzzyRoll){
                  Serial.println("Starting Fuzzy Roll");
                } else if (StartFuzzyRoll == 0) {
                  Serial.println("Stoping Fuzzy Roll");

                  #ifdef ENABLE_FUZZY_ROLL
                  resetFuzzyRoll();
                  #endif
                }
                break;
            case 31:
                while (!Serial.available());
                temp = Serial.parseFloat();
                if (temp > 8 || temp < 0 ){
                  Serial.println("Out off Range PAngleRoll");
                } else{
                  PAnglePitch = temp;
                }
                break;
            case 32:
                while (!Serial.available());
                temp = Serial.parseFloat();
                if (temp > 8 || temp < 0 ){
                  Serial.println("Out off Range IAnglePitch");
                } else{
                  IAnglePitch = temp;
                }
                break;
            case 33:
                while (!Serial.available());
                temp = Serial.parseFloat();
                if (temp > 8 || temp < 0 ){
                  Serial.println("Out off Range DAngleRoll");
                } else{
                  DAnglePitch = temp;
                }
                break;
            case 34:
                while (!Serial.available());
                temp = Serial.parseFloat();
                if (temp > 8 || temp < 0 ){
                  Serial.println("Out off Range PRateRoll");
                } else{
                  PRatePitch = temp;
                }
                break;
            case 35:
                while (!Serial.available());
                temp = Serial.parseFloat();
                if (temp > 8 || temp < 0 ){
                  Serial.println("Out off Range PRateRoll");
                } else{
                  IRatePitch = temp;
                }
                break;
            case 36:
                while (!Serial.available());
                temp = Serial.parseFloat();
                if (temp > 8 || temp < 0 ){
                  Serial.println("Out off Range PRateRoll");
                } else{
                  DRatePitch = temp;
                }
                break;
        }
    }
  }

  void test(){
    for (int i = MIN_PULSE_LENGTH; i <= 1300; i += 10) {
        Serial.print("Pulse length = ");
        Serial.println(i);

        // analogWrite(motorTestPin,1.024*i);
        runMotor(i);

        delay(100);
    }
        for (int i = 1300; i >= MIN_PULSE_LENGTH; i -= 10) {
        Serial.print("Pulse length = ");
        Serial.println(i);

        runMotor(i);
        // analogWrite(motorTestPin,1.024*i);


        delay(100);
    }

    Serial.println("STOP");
    // analogWrite(motorTestPin,MIN_PULSE_LENGTH);
    input_throttle = MIN_PULSE_LENGTH;
    Serial.print("input_throttle: ");Serial.println(input_throttle);    
  }

  void throttle_down_3s(){
    for (int i = MAX_PULSE_LENGTH; i >= MIN_PULSE_LENGTH; i -= 20) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        // analogWrite(motorTestPin,1.024*i);
        runMotor(i);
        timerCounter += 60;
        Serial.println(timerCounter);
        // formula delay : t*1000 / ((max_length - min_length)/i_step)
        delay(60);
    }

    Serial.println("STOP");
    // analogWrite(motorTestPin,MIN_PULSE_LENGTH);
    runMotor(MIN_PULSE_LENGTH);  // M1
    input_throttle = MIN_PULSE_LENGTH;

    timerCounter = 0;
  }

  void throttle_down_2s(){
    for (int i = MAX_PULSE_LENGTH; i >= MIN_PULSE_LENGTH; i -= 20) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        // analogWrite(motorTestPin,1.024*i);
        runMotor(i);

        timerCounter += 40;
        Serial.println(timerCounter);
        // formula delay : t*1000 / ((max_length - min_length)/i_step)
        delay(40);
    }

    Serial.println("STOP");
    // analogWrite(motorTestPin,MIN_PULSE_LENGTH);
    runMotor(MIN_PULSE_LENGTH);  // M1
    input_throttle = MIN_PULSE_LENGTH;

    timerCounter = 0;
  }
#endif

// 11. PID for Controller
#ifdef ENABLE_LED
  #define ledRed 33
  #define ledGreen 32
  #define ledTeensy 13

  void initLed(){
    pinMode(ledTeensy, OUTPUT); 
    pinMode(ledRed, OUTPUT);
    pinMode(ledGreen, OUTPUT);
    digitalWrite(ledTeensy, HIGH);
    digitalWrite(ledRed, HIGH);
    digitalWrite(ledGreen, HIGH);
  }

  void ledBlink(){

  }
#endif


#ifdef ENABLE_MONITORING_1HZ
  void monitoring1Hz(){
    #ifdef ENABLE_MONITORING_IMU_1HZ
      Serial.print("IMU| ");
      Serial.print("Gx:");Serial.print(RateRoll); Serial.print(" | ");
      Serial.print("Gy:");Serial.print(RatePitch); Serial.print(" | ");
      Serial.print("Gz:");Serial.print(RateYaw); Serial.print(" | ");
      Serial.print("R:");Serial.print(Roll); Serial.print(" | ");
      Serial.print("P:");Serial.print(Pitch); Serial.print(" | ");
      Serial.print("Y:");Serial.println(Yaw);
    #endif
    #ifdef ENABLE_MONITORING_BMP_1HZ
      Serial.print("BMP| ");
      Serial.print("Alt:");Serial.print(alti); Serial.print(" | ");
      Serial.print("Pressure:");Serial.print(press); Serial.print(" | ");
      Serial.print("Temp:");Serial.println(temp);
    #endif
    #ifdef ENABLE_MONITORING_LIDAR_1HZ
      Serial.print("Lidar| ");
      Serial.print("Dist: "); Serial.print(distance); Serial.print(" | ");
      Serial.print("Strength: "); Serial.println(strength);
    #endif
    #ifdef ENABLE_MONITORING_VOLTAGE_CURRENT_1HZ
      Serial.print("Analog| ");
      Serial.print("V: ");Serial.print(voltage); Serial.print(" | ");
      Serial.print("I: ");Serial.print(current); Serial.print(" | ");
      Serial.print("V_volt: ");Serial.print(voltageVolt); Serial.print(" | ");
      Serial.print("I_volt: ");Serial.println(currentVolt);
    #endif
    #ifdef ENABLE_MONITORING_MOTOR_1HZ
      Serial.print("Motor| ");
      Serial.print("M1: "); Serial.print(MotorInput1); Serial.print("| M2: "); Serial.print(MotorInput2);
      Serial.print("| M3: "); Serial.print(MotorInput3); Serial.print("| M4: "); Serial.print(MotorInput4);
      Serial.print("| M5: "); Serial.print(MotorInput5); Serial.print("| M6: "); Serial.println(MotorInput6);
    #endif
    #ifdef ENABLE_MONITORING_PID_1HZ
      Serial.print("PID| ");
      Serial.print("iT:");Serial.print(InputThrottle); Serial.print(" | ");
      Serial.print("iR:");Serial.print(InputRoll); Serial.print(" | ");
      Serial.print("iP:");Serial.print(InputPitch); Serial.print(" | ");
      Serial.print("iY:");Serial.print(InputYaw); Serial.println(" | ");

      Serial.print("MotorPID| ");
      Serial.print("M1P:");Serial.print(MotorInput1); Serial.print(" | ");
      Serial.print("M2P:");Serial.print(MotorInput2); Serial.print(" | ");
      Serial.print("M3P:");Serial.print(MotorInput3); Serial.print(" | ");
      Serial.print("M4P:");Serial.print(MotorInput4); Serial.print(" | ");
      Serial.print("M5P:");Serial.print(MotorInput5); Serial.print(" | ");
      Serial.print("M6P:");Serial.println(MotorInput6);
    #endif
    #ifdef ENABLE_MONITORING_RPM_1HZ
      Serial.print("RPM| ");
      Serial.print("RPM:");Serial.println(measuredRPM);
      // Serial.print("RPM_Filtered");Serial.println(current);
    #endif
    #ifdef ENABLE_MONITORING_RTC_1HZ
      Serial.print("RTC| ");
      Serial.print(currentHour);Serial.print(":"); 
      Serial.print(currentMinute);Serial.print(":");
      Serial.println(currentSecond);
    #endif
  }
#endif

#ifdef ENABLE_MONITORING_10HZ
  void monitoring10Hz(){
    #ifdef ENABLE_MONITORING_IMU_10HZ
      Serial.print("IMU| ");
      Serial.print("Gx:");Serial.print(RateRoll); Serial.print(" | ");
      Serial.print("Gy:");Serial.print(RatePitch); Serial.print(" | ");
      Serial.print("Gz:");Serial.print(RateYaw); Serial.print(" | ");
      Serial.print("R:");Serial.print(Roll); Serial.print(" | ");
      Serial.print("P:");Serial.print(Pitch); Serial.print(" | ");
      Serial.print("Y:");Serial.println(Yaw);
    #endif
    #ifdef ENABLE_MONITORING_BMP_10HZ
      Serial.print("BMP| ");
      Serial.print("Alt:");Serial.print(alti); Serial.print(" | ");
      Serial.print("Pressure:");Serial.print(press); Serial.print(" | ");
      Serial.print("Temp:");Serial.println(temp);
    #endif
    #ifdef ENABLE_MONITORING_LIDAR_10HZ
      Serial.print("Lidar| ");
      Serial.print("Dist: "); Serial.print(distance); Serial.print(" | ");
      Serial.print("Strength: "); Serial.println(strength);
    #endif
    #ifdef ENABLE_MONITORING_VOLTAGE_CURRENT_10HZ
      Serial.print("Analog| ");
      Serial.print("V: ");Serial.print(voltage); Serial.print(" | ");
      Serial.print("I: ");Serial.print(current); Serial.print(" | ");
      Serial.print("V_volt: ");Serial.print(voltageVolt); Serial.print(" | ");
      Serial.print("I_volt: ");Serial.println(currentVolt);
    #endif
    #ifdef ENABLE_MONITORING_MOTOR_10HZ
      Serial.print("Motor| ");
      Serial.print("M1: "); Serial.print(MotorInput1); Serial.print("| M2: "); Serial.print(MotorInput2);
      Serial.print("| M3: "); Serial.print(MotorInput3); Serial.print("| M4: "); Serial.print(MotorInput4);
      Serial.print("| M5: "); Serial.print(MotorInput5); Serial.print("| M6: "); Serial.println(MotorInput6);
    #endif
    #ifdef ENABLE_MONITORING_PID_10HZ
      Serial.print("PID| ");
      Serial.print("iT:");Serial.print(InputThrottle); Serial.print(" | ");
      Serial.print("iR:");Serial.print(InputRoll); Serial.print(" | ");
      Serial.print("iP:");Serial.print(InputPitch); Serial.print(" | ");
      Serial.print("iY:");Serial.print(InputYaw); Serial.println(" | ");

      Serial.print("MotorPID| ");
      Serial.print("M1P:");Serial.print(MotorInput1); Serial.print(" | ");
      Serial.print("M2P:");Serial.print(MotorInput2); Serial.print(" | ");
      Serial.print("M3P:");Serial.print(MotorInput3); Serial.print(" | ");
      Serial.print("M4P:");Serial.print(MotorInput4); Serial.print(" | ");
      Serial.print("M5P:");Serial.print(MotorInput5); Serial.print(" | ");
      Serial.print("M6P:");Serial.println(MotorInput6);
    #endif
    #ifdef ENABLE_MONITORING_FUZZY_ROLL_10HZ
      Serial.print("Fuzzy Roll | ");
      Serial.print("iR:");Serial.print(Throttle_Roll); Serial.print(" | ");

      Serial.print("MotorFuzzy | ");
      Serial.print("M1P:");Serial.print(MotorInput1); Serial.print(" | ");
      Serial.print("M2P:");Serial.print(MotorInput2); Serial.print(" | ");
      Serial.print("M3P:");Serial.print(MotorInput3); Serial.print(" | ");
      Serial.print("M4P:");Serial.print(MotorInput4); Serial.print(" | ");
      // Serial.print("M5P:");Serial.print(MotorInput5); Serial.print(" | ");
      // Serial.print("M6P:");Serial.println(MotorInput6);
    #endif
    #ifdef ENABLE_MONITORING_RPM_10HZ
      Serial.print("RPM| ");
      Serial.print("RPM:");Serial.println(rpm);
      // Serial.print("RPM_Filtered");Serial.println(current);
    #endif
    #ifdef ENABLE_MONITORING_RTC_10HZ
      Serial.print("RTC| ");
      Serial.print(currentHour);Serial.print(":"); 
      Serial.print(currentMinute);Serial.print(":");
      Serial.println(currentSecond);
    #endif
  }
#endif

#ifdef ENABLE_SERIAL1
  float inputData[20];
  char bufferSerial1In[512];
  int lenSerial1In;
  int channelSerial1 = 0;

  void initSerial1(){
    Serial1.begin(115200);
    SerialUSB1.println("SerialUSB1 - ACTIVE");
  }
  void logSerial1(){
    char bufferSerial1[512];
  //                                                                 1   2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20
    int lenSerial1 = snprintf(bufferSerial1, sizeof(bufferSerial1), "%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 
                       cMillis/*1*/,input_throttle/*2*/,MotorInput1/*3*/, MotorInput2/*4*/, MotorInput3/*5*/, MotorInput4/*6*/, Roll/*7*/, Pitch/*8*/, Yaw/*9*/, RateRoll/*10*/,RatePitch/*11*/,RateYaw/*12*/,KalmanAngleRoll/*13*/, DesiredAngleRoll/*14*/,DesiredRateRoll/*15*/, InputRoll/*16*/ ,KalmanAnglePitch/*17*/,DesiredAnglePitch/*18*/, DesiredRatePitch/*19*/, InputPitch/*120*/);
    SerialUSB1.write(bufferSerial1, lenSerial1); // Write the entire buffer at once
  }
  void readSerial1(){
    // Serial.println(inputData[0]);
    // Serial.println(inputData[1]);

    if (SerialUSB1.available() > 0) {
    // Serial.println("\n---------INPUT from EXCEL-----------\n");
    for (channelSerial1 = 0; channelSerial1 < 20; channelSerial1++) {
      if (SerialUSB1.available() > 0) {
        inputData[channelSerial1] = SerialUSB1.parseFloat();  // Membaca nilai float dan menyimpannya ke array
      }
    }

    input_throttle = inputData[1]; 
    PAnglePitch = inputData[2];
    IAnglePitch = inputData[3];
    DAnglePitch = inputData[4];
    PRatePitch = inputData[5];
    IRatePitch = inputData[6];
    DRatePitch = inputData[7];
    



    // Serial.println("\nReceived Data:");
    // for (int i = 0; i < 20; i++) {
    //   Serial.print("Channel ");
    //   Serial.print(i + 1);
    //   Serial.print(": ");
    //   Serial.println(inputData[i]);
    // }
    }
}
#endif

void setup(){
  // Initialize each module based on preprocessor flags
  #ifdef ENABLE_SERIAL
    Serial.begin(115200);       // Serial initialization
    Serial.println("Serial Main - ACTIVE");
  #endif

  #ifdef ENABLE_SERIAL1
    initSerial1();
    // available @ SerialUSB1.println("");
    // available @ logSerial1();
    // available @ readSerial1();
  #endif

  #ifdef ENABLE_IMU
    initImu();
    // available @ readImu();
  #endif

  #ifdef ENABLE_BMP
    initBmp();
    // available @ readBmp();
  #endif

  #ifdef ENABLE_LIDAR
    initLidar();
    // available @ readLidar();
  #endif

  #ifdef ENABLE_VOLTAGE_CURRENT
    initVoltageCurrent();
    // available @ readVoltageCurrent();
  #endif

  #ifdef ENABLE_MOTOR
    initMotor();
    // available @ runMotor(m1,m2,m3,m4,m5,m6);
  #endif

  #ifdef ENABLE_PID_ANGLE
    initPidAngle();
    // available @ loopPidAngle();
  #endif

  #ifdef ENABLE_PID_RATE
    initPidRate();
    // available @ loopPidRate();
  #endif

  #ifdef ENABLE_FUZZY_ROLL
    initFuzzyRoll();
    // available @ runFuzzyRoll();
  #endif

  #ifdef ENABLE_RTC
    initRtc();
    rtc();
  #endif

  #ifdef ENABLE_SD_CARD
    initSd();
    // available @ logData();
  #endif

  #ifdef ENABLE_RECEIVER
    initReceiver();
    // available @ readReceiver();
  #endif

  #ifdef ENABLE_USER_INPUT 
  userInput();
  displayInstructions();
  #endif

  #ifdef ENABLE_RPM_1
    initRPM_1();
  #endif

  #ifdef ENABLE_RPM_2
    initRPM_2();
  #endif

  #ifdef ENABLE_RPM_3
    initRPM_3();
  #endif

  #ifdef ENABLE_RPM_4
    initRPM_4();
  #endif

  #ifdef ENABLE_CUSTOM
    
  #endif  

  #ifdef ENABLE_LED
  initLed();
  // available @ ledBlink();
  #endif

  #ifdef ENABLE_INTERVAL
  // initInterval(); // Tidak perlu
  #endif
}

void loop(){
  cMicros = micros();
  cMillis = millis();


  // 500Hz
  if (cMicros - pM500Hz >= interval500Hz){
    // Fuction & Command
    #ifdef ENABLE_MOTOR
      #ifdef ENABLE_MOTOR_MANUAL_ALL
        MotorInput1 = input_throttle;
        MotorInput2 = input_throttle;
        MotorInput3 = input_throttle;
        MotorInput4 = input_throttle;
      #elif defined(ENABLE_MOTOR_MANUAL_1) || defined(ENABLE_MOTOR_MANUAL_2) || defined(ENABLE_MOTOR_MANUAL_3) || defined(ENABLE_MOTOR_MANUAL_4)
        #ifdef ENABLE_MOTOR_MANUAL_1
            MotorInput1 = input_throttle;
        #endif
        #ifdef ENABLE_MOTOR_MANUAL_2
            MotorInput2 = input_throttle;
        #endif
        #ifdef ENABLE_MOTOR_MANUAL_3
            MotorInput3 = input_throttle;
        #endif
        #ifdef ENABLE_MOTOR_MANUAL_4
            MotorInput4 = input_throttle;
        #endif
      #elif defined(ENABLE_MOTOR_MANUAL_PER_MOTOR)
        MotorInput1 = motor1_throttle + input_throttle;
        MotorInput2 = motor2_throttle + input_throttle;
        MotorInput3 = motor3_throttle + input_throttle;
        MotorInput4 = motor4_throttle + input_throttle;
      #endif
      runMotor(MotorInput1, MotorInput2, MotorInput3, MotorInput4);
    #endif

    
    #ifdef ENABLE_SD_CARD
      logData();  
    #endif
    
    
    #ifdef ENABLE_SERIAL1_500HZ
      logSerial1();
    #endif

    
    
    // End Timer
    pM500Hz = cMicros;
  }

  // 100Hz
  if (cMillis - pM100Hz >= interval100Hz){
    // iteration ++;
  // if (iteration <= 2000) {
  //   // Increase setPointRoll from 0 to 20
  //   refRoll = map(iteration, 0, 2000, 0, 20);
  // }
  // else if (iteration <= 4000) {
  //   // Decrease setPointRoll from 20 to 0
  //   refRoll = map(iteration, 2001, 4000, 20, 0);
  // }
  // else if (iteration <= 6000) {
  //   // Decrease setPointRoll from 0 to -20
  //   refRoll = map(iteration, 4001, 6000, 0, -20);
  // }
  // else {
  //   refRoll = 0;  // Reset iteration to create a loop
  // }
    // Fuction & Command
    #ifdef ENABLE_SERIAL1_INPUT
      readSerial1();
    #endif

    #ifdef ENABLE_PID_ANGLE
      loopPidAngle();
    #endif
    #ifdef ENABLE_PID_RATE
      loopPidRate();
    #endif

    #ifdef ENABLE_USER_INPUT
      userInput();
    #endif

    #ifdef ENABLE_IMU
      readImu();
    #endif

    #ifdef ENABLE_FUZZY_ROLL
      runFuzzyRoll();
    #endif

    #ifdef ENABLE_RECEIVER
      readReceiver();
    #endif

    #ifdef ENABLE_LIDAR
      readLidar();
    #endif

    #ifdef ENABLE_SERIAL1_100HZ
      logSerial1();
    #endif

    #ifdef ENABLE_CUSTOM
      loopPidAngleRate();
    #endif
    
    #ifdef ENABLE_RPM_1
      printRpm_1();
    #endif
    
    #ifdef ENABLE_RPM_2
      printRpm_2();
    #endif

    #ifdef ENABLE_RPM_3
      printRpm_3();
    #endif

    #ifdef ENABLE_RPM_4
      printRpm_4();
    #endif

    // End Timer
    pM100Hz = cMillis;
  }

  // 10Hz
  if (cMillis - pM10Hz >= interval10Hz){
    // Fuction & Command
    #ifdef ENABLE_BMP
      readBmp();
    #endif
    #ifdef ENABLE_VOLTAGE_CURRENT
      readVoltageCurrent();
    #endif
    #ifdef ENABLE_MONITORING_10HZ
      monitoring10Hz();
    #endif

    #ifdef ENABLE_SERIAL1_10HZ
      logSerial1();
    #endif
    // Serial1.println("SERIAL1");

    // End Timer
    pM10Hz = cMillis;
  }

  // 1Hz
  if (cMillis - pM1Hz >= interval1Hz){
    // Fuction & Command
    #ifdef ENABLE_LED
      ledBlink();
    #endif
    
    #ifdef ENABLE_RTC
      rtc();
    #endif
    #ifdef ENABLE_MONITORING_1HZ
      monitoring1Hz();
    #endif

    #ifdef ENABLE_SERIAL1_1HZ
      logSerial1();
    #endif

    // End Timer
    pM1Hz = cMillis;
  }
  // End Loop
}