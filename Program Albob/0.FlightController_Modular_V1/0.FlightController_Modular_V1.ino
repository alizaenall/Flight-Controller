/*
  USER INPUT INTEGER on SERIAL MONITOR
  // 0 Sending minimum throttle
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
*/



// Enable or disable modules by commenting/uncommenting
// #define ENABLE_CUSTOM

#define ENABLE_MOTOR             // 5        // DEFAULT QUAD
// #define ENABLE_MOTOR_HEXACOPTER             // UNCOMMENT FOR HEXACOPTER
#define ENABLE_MOTOR_MANUAL_ALL
// #define ENABLE_MOTOR_MANUAL_1 
// #define ENABLE_MOTOR_MANUAL_2 
// #define ENABLE_MOTOR_MANUAL_3 
// #define ENABLE_MOTOR_MANUAL_4 

// #define ENABLE_IMU              // 1  
// #define ENABLE_BMP              // 2  
// #define ENABLE_LIDAR            // 3  
// #define ENABLE_VOLTAGE_CURRENT  // 4  
#define ENABLE_PID_RATE              // 6  // DURUNG
#define ENABLE_RATE_MODE
#define ENABLE_ANGLE_MODE
// #define ENABLE_SD_CARD          // 7  
#define ENABLE_RPM_1            // 8.1  // Durung
#define ENABLE_RPM_2            // 8.2
#define ENABLE_RPM_3            // 8.3
#define ENABLE_RPM_4            // 8.4
// #define ENABLE_RTC
// #define ENABLE_RECEIVER         // 9 .
#define ENABLE_USER_INPUT      // 10
#define ENABLE_LED
  // #define ENABLE_IMU_PRINT      // done
  // #define ENABLE_BMP_PRINT      
  // #define ENABLE_LIDAR_PRINT
  // #define ENABLE_VOLTAGE_CURRENT_PRINT
  // #define ENABLE_MOTOR_PRINT
  // #define ENABLE_PID_PRINT   
  // #define ENABLE_RPM_1_PRINT             // DURUNG
  // #define ENABLE_RTC_PRINT
  // #define ENABLE_RECEIVER_PRINT  // DURUNG

#define ENABLE_INTERVAL  // WAJIB
//     #define ENABLE_MONITORING_1HZ
//           #define ENABLE_MONITORING_IMU_1HZ
//           #define ENABLE_MONITORING_BMP_1HZ
//           #define ENABLE_MONITORING_LIDAR_1HZ
//           #define ENABLE_MONITORING_VOLTAGE_CURRENT_1HZ
//           #define ENABLE_MONITORING_MOTOR_1HZ
          // #define ENABLE_MONITORING_PID_1HZ
          // #define ENABLE_MONITORING_RPM_1HZ
          // #define ENABLE_MONITORING_RTC_1HZ
    // #define ENABLE_MONITORING_10HZ
          // #define ENABLE_MONITORING_IMU_10HZ
          // #define ENABLE_MONITORING_BMP_10HZ
          // #define ENABLE_MONITORING_LIDAR_10HZ
          // #define ENABLE_MONITORING_VOLTAGE_CURRENT_10HZ
          // #define ENABLE_MONITORING_MOTOR_10HZ
          // #define ENABLE_MONITORING_PID_10HZ
          // #define ENABLE_MONITORING_RPM_10HZ
          // #define ENABLE_MONITORING_RTC_10HZ
    // #define ENABLE_MONITORING_DYNAMIC              // BELUM BISA
  
// #define ENABLE_SERIAL1            // COM15 // Tools > UBS_Type > Dual_Serial
  /*Choose ONE of SERIAL FREQUENCY from option BELOW*/
  // #define ENABLE_SERIAL1_500HZ
  // #define ENABLE_SERIAL1_100HZ
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
      delay(1);
    }
    RateCalibrationRoll/=2000;
    RateCalibrationPitch/=2000;
    RateCalibrationYaw/=2000;
    Serial.println("CalibrationDone");
  }

  void readImu(){
      mpu.readAngularVelocity();  /* read Angular Velocity */
      mpu.readEuler();  /* read euler angle */
      // Variable IMU
      RateRoll = mpu.GyrData.x;    // Roll Rate  (deg/s)
      RatePitch = mpu.GyrData.y;   // Pitch Rate (deg/s)
      RateYaw = mpu.GyrData.z;     // Yaw Rate   (deg/s)
      Roll = mpu.EulerAngles.x;     // Roll      (deg)
      Pitch = mpu.EulerAngles.y;    // Pitch     (deg)
      Yaw = mpu.EulerAngles.z;      // Pitch     (deg)

      #ifdef ENABLE_IMU_PRINT
        Serial.print("IMU| ");
        Serial.print("Gx:");Serial.print(RateRoll); Serial.print(" | ");
        Serial.print("Gy:");Serial.print(RatePitch); Serial.print(" | ");
        Serial.print("Gz:");Serial.print(RateYaw); Serial.print(" | ");
        Serial.print("R:");Serial.print(Roll); Serial.print(" | ");
        Serial.print("P:");Serial.print(Pitch); Serial.print(" | ");
        Serial.print("Y:");Serial.println(Yaw);
        
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
#ifdef ENABLE_PID_RATE
  // Inner Control
  setPointRpm = 0;
  float DesiredRpm = setPointRpm;

  // Supervisory Control
  float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
  float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
  float InputRoll, InputThrottle, InputPitch, InputYaw;
  float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
  float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
  float PIDReturn[]={0, 0, 0};
  float PRateRoll=0.6 ; float PRatePitch=PRateRoll; float PRateYaw=2;
  float IRateRoll=3.5 ; float IRatePitch=IRateRoll; float IRateYaw=12;
  float DRateRoll=0.03 ; float DRatePitch=DRateRoll; float DRateYaw=0;
  // float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
  // SetPoint Controller PID
  float ManualThrottle = input_throttle;
  float ManualRateRoll = 0;
  float ManualRatePitch = 0;
  float ManualRateYaw = 0;

  void initPid(){
    // DesiredRateRoll=0.15*(ReceiverValue[0]-1500);
    DesiredRateRoll=ManualRateRoll;
    // DesiredRatePitch=0.15*(ReceiverValue[1]-1500);
    DesiredRatePitch=ManualRatePitch;
    // InputThrottle=ReceiverValue[2];
    InputThrottle=ManualThrottle;
    // DesiredRateYaw=0.15*(ReceiverValue[3]-1500);
    DesiredRateYaw=ManualRateYaw;
  }

  void pidEquation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
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
  void resetPid(void) {
    PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
    PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
  }
  void errorPid(){
    ErrorRateRoll=DesiredRateRoll-RateRoll;
    ErrorRatePitch=DesiredRatePitch-RatePitch;
    ErrorRateYaw=DesiredRateYaw-RateYaw;
  }
  void loopPid(){
    errorPid();
    pidEquation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
        InputRoll=PIDReturn[0];
        PrevErrorRateRoll=PIDReturn[1]; 
        PrevItermRateRoll=PIDReturn[2];
    pidEquation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
        InputPitch=PIDReturn[0]; 
        PrevErrorRatePitch=PIDReturn[1]; 
        PrevItermRatePitch=PIDReturn[2];
    pidEquation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
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

    int ThrottleIdle=1060;
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
      resetPid();
    }
    #ifdef ENABLE_PID_PRINT
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
  #include <IntervalTimer.h>
  IntervalTimer rpmTimer;
  volatile unsigned long lastPulseTime = 0;
  
  
  FASTRUN void pulseHandler() {
  unsigned long currentTime = micros(); // Current time in microseconds
  pulseInterval = currentTime - lastPulseTime; // Time between pulses
  lastPulseTime = currentTime;
  }

  void calculateRPM() {
  if (pulseInterval > 0 && ((unsigned long)micros()) - lastPulseTime <= 20000) {
    float frequency = 1000000.0  / pulseInterval; // Convert µs interval to frequency (Hz)
    rpm = (frequency * 60.0) / 7;

    // pada fungsi interupt, hindari penggunaan serial print
    // Serial.print("RPM: ");
    // Serial.println(rpm);
    // Serial.print(" | Interval: ");
    // Serial.print(pulseInterval);
  } else if (((unsigned long)micros()) - lastPulseTime > 20000) {
    // Serial.println("No pulses detected");
    rpm = 0;
  }
  }

  void printRpm(){
    Serial.print("RPM: ");
    Serial.print(rpm);
    Serial.print(" | Interval: ");
    Serial.println(pulseInterval);
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
    Serial.println("    18  \tSending minimum throttle");
    Serial.println("    1  \tSending maximum throttle");
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
    Serial.println("   17  \tSP_RPM: -1000\n\n");
  }

  void userInput(){
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
                      input_throttle = MAX_PULSE_LENGTH;
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
  #define ledRed 30
  #define ledGreen 31
  #define ledTeensy 13

  void initLed(){
    pinMode(13, OUTPUT); 
    pinMode(30, OUTPUT);
    pinMode(31, OUTPUT);
    digitalWrite(13, HIGH);
    digitalWrite(30, HIGH);
    digitalWrite(31, HIGH);
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
  void initSerial1(){
    Serial1.begin(115200);
    SerialUSB1.println("SerialUSB1 - ACTIVE");
  }
  void logSerial1(){
    char bufferSerial1[512];
    int lenSerial1 = snprintf(bufferSerial1, sizeof(bufferSerial1), "%lu,%f,%f\n", 
                       cMillis,input_throttle,rpm_3);
    SerialUSB1.write(bufferSerial1, lenSerial1); // Write the entire buffer at once
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

  #ifdef ENABLE_PID_RATE
    initPid();
    // available @ loopPid();
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
    pinMode(8, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(8), pulseHandler, RISING); // Detect rising edges
    rpmTimer.begin(calculateRPM, 10000); // Call calculateRPM every 100ms
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
    #ifdef ENABLE_PID_RATE
      loopPid();
    #endif
    #ifdef ENABLE_MOTOR
      #ifdef ENABLE_MOTOR_MANUAL_ALL
      MotorInput1 = input_throttle;
      MotorInput2 = input_throttle;
      MotorInput3 = input_throttle;
      MotorInput4 = input_throttle;
      #else
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
      #endif
    runMotor(MotorInput1,MotorInput2, MotorInput3,MotorInput4);
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
    // Fuction & Command
    #ifdef ENABLE_IMU
      readImu();
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
      printRpm();
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
    #ifdef ENABLE_USER_INPUT
      userInput();
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