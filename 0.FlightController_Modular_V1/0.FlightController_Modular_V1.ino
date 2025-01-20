#include <DFRobot_BNO055.h>

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
  //19 M1
  //20 M2
  //21 M3
  //22 M4
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

// Default to start flight controller : E_Motor, E_IMU, E_LIDAR, E_PID_ANGLE_RATE, E_RECEIVER, E_USER_INPUT, E_LED, E_IMU_PRINT, E_LIDAR_PRINT, E_PID_ANGLE_RATE_PRINT, E_RECEIVER_PRINT, 
// E_INTERVAL, E_SERIAL1, E_SERIAL1_PRINT, E_SERIAL1_100HZ

// Enable or disable modules by commenting/uncommenting

#define ENABLE_SEQUENCE_EXCECUTION
// #define ENABLE_CUSTOM
// #define ENABLE_CUSTOM_PRINT

#define ENABLE_MOTOR  // 5        // DEFAULT QUAD
// #define ENABLE_MOTOR_HEXACOPTER             // UNCOMMENT FOR HEXACOPTER
// #define ENABLE_MOTOR_MANUAL_ALL
// #define ENABLE_MOTOR_MANUAL_PER_MOTOR
// #define ENABLE_MOTOR_MANUAL_1
// #define ENABLE_MOTOR_MANUAL_2
// #define ENABLE_MOTOR_MANUAL_3
// #define ENABLE_MOTOR_MANUAL_4

#define ENABLE_IMU  // 1
// #define ENABLE_BMP              // 2
#define ENABLE_LIDAR            // 3
// #define ENABLE_VOLTAGE_CURRENT  // 4
// #define ENABLE_PID_RATE              // 6  // DURUNG
// #define ENABLE_PID_ANGLE
// #define ENABLE_PID_ANGLE_ONLY
// #define ENABLE_FUZZY_ROLL
#define ENABLE_PID_ANGLE_RATE
// #define ENABLE_PID_ANGLE_RATE_ALTITUDE
// #define ENABLE_PID_ANGLE_RATE_POSITION
// #define ENABLE_SD_CARD          // 7
// #define ENABLE_RPM_1            // 8.1  // Durung
// #define ENABLE_RPM_2            // 8.2
// #define ENABLE_RPM_3            // 8.3
// #define ENABLE_RPM_4            // 8.4
// #define ENABLE_RTC
#define ENABLE_RECEIVER        // 9
#define ENABLE_USER_INPUT      // 10
#define ENABLE_LED
#define ENABLE_IMU_PRINT             //  done
// #define ENABLE_BMP_PRINT
#define ENABLE_LIDAR_PRINT
// #define ENABLE_VOLTAGE_CURRENT_PRINT
// #define ENABLE_MOTOR_PRINT
// #define ENABLE_PID_RATE_PRINT
// #define ENABLE_PID_ANGLE_PRINT
#define ENABLE_PID_ANGLE_RATE_PRINT

// #define ENABLE_RPM_1_PRINT        // DURUNG
// #define ENABLE_RTC_PRINT
#define ENABLE_RECEIVER_PRINT     


#define ENABLE_INTERVAL  // WAJIB
// #define ENABLE_MONITORING_1HZ
//           #define ENABLE_MONITORING_IMU_1HZ
//           #define ENABLE_MONITORING_BMP_1HZ
//           #define ENABLE_MONITORING_LIDAR_1HZ
//           #define ENABLE_MONITORING_VOLTAGE_CURRENT_1HZ
//            #define ENABLE_MONITORING_MOTOR_1HZ
//            #define ENABLE_MONITORING_PID_1HZ
//            #define ENABLE_MONITORING_RPM_1HZ
//            #define ENABLE_MONITORING_RTC_1HZ
// #define ENABLE_MONITORING_10HZ
//            #define ENABLE_MONITORING_IMU_10HZ
//            #define ENABLE_MONITORING_BMP_10HZ
//            #define ENABLE_MONITORING_LIDAR_10HZ
//            #define ENABLE_MONITORING_VOLTAGE_CURRENT_10HZ
          //  #define ENABLE_MONITORING_MOTOR_10HZ
//            #define ENABLE_MONITORING_PID_10HZ
          //  #define ENABLE_MONITORING_RPM_10HZ
//            #define ENABLE_MONITORING_FUZZY_ROLL_10HZ //Masih Salah
//            #define ENABLE_MONITORING_RTC_10HZ
//            #define ENABLE_MONITORING_DYNAMIC              // BELUM BISA
#define ENABLE_MONITORING_100HZ
// #define ENABLE_LIDAR_PRINT_100HZ
#define ENABLE_MOTOR_PRINT_100HZ

#define ENABLE_SERIAL1  // COM15 // Tools > UBS_Type > Dual_Serial
#define ENABLE_SERIAL1_INPUT
//Choose ONE of SERIAL FREQUENCY from option BELOW/
// #define ENABLE_SERIAL1_500HZ
#define ENABLE_SERIAL1_100HZ
// #define ENABLE_SERIAL1_10HZ
// #define ENABLE_SERIAL1_1HZ

// #define ENABLE_GPS

// GLOBAL STATE
bool SafetyState = false;


#ifdef ENABLE_GPS

#include <Adafruit_GPS.h>
#define GPSSerial Serial7
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false
uint32_t TimerGPS = millis();

double latitude = 0.0;
double longitude = 0.0;
float altitude = 0.0;
float speed = 0.0;
float angle = 0.0;
int satellites = 0;
int GPSFix = 0;

const double PHI = 3.14159265358979323846;
const double EARTH_RADIUS = 6371000.0; // dalam meter

double lat1 = -7.285598, lon1 = 112.795677;
double lat2 = -7.285598, lon2 = 112.795677;
double x_1, y_1, x_2, y_2;
double deltaX;
double deltaY;
double xInit=0;
double yInit=0;
double xNow = 0;
double yNow = 0;
double xDesired = 0;
double yDesired = 0;
double xTarget = 0;
double yTarget = 0;
float dX = 0;
float dY = 0;
bool firstReading = true;

double degToRad(double deg) {
  return deg * (PHI / 180.0);
}

// Fungsi proyeksi Mercator
void mercatorProjection(double lat, double lon, double& x, double& y) {
  double latRad = degToRad(lat);
  double lonRad = degToRad(lon);
  x = EARTH_RADIUS * lonRad;
  y = EARTH_RADIUS * log(tan(PHI / 4.0 + latRad / 2.0));
}

void initGPS(){
  Serial.println("Adafruit GPS library basic parsing test!");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 5 Hz update rate
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ); // 5 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}
void loopGPS(){
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    // Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - TimerGPS > 100) {
    TimerGPS = millis(); // reset the timer
    // Serial.print("\nTime: ");
    // if (GPS.hour < 10) { Serial.print('0'); }
    // Serial.print(GPS.hour, DEC); Serial.print(':');
    // if (GPS.minute < 10) { Serial.print('0'); }
    // Serial.print(GPS.minute, DEC); Serial.print(':');
    // if (GPS.seconds < 10) { Serial.print('0'); }
    // Serial.print(GPS.seconds, DEC); Serial.print('.');
    // if (GPS.milliseconds < 10) {
    //   Serial.print("00");
    // } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
    //   Serial.print("0");
    // }
    GPSFix = (int)GPS.fix;
    // Serial.print(GPS.milliseconds);
    // Serial.print(" | ");
    // Serial.print(GPSFix);
    // Serial.print(" | ");
    // Serial.println((int)GPS.fixquality);
    // Serial.println((int)GPS.fixquality_3d);
    // Serial.print("Date: ");
    // Serial.print(GPS.day, DEC); Serial.print('/');
    // Serial.print(GPS.month, DEC); Serial.print("/20");
    // Serial.println(GPS.year, DEC);
    // Serial.print("Fix: "); Serial.print((int)GPS.fix);
    // Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPSFix) {

      if (firstReading) {  // Save the first set of coordinates
        lat1 = GPS.latitudeDegrees;
        lon1 = GPS.longitudeDegrees;
        firstReading = false;  // Reset the flag after saving first coordinates
      } else {  // Save subsequent coordinates
        lat2 = GPS.latitudeDegrees;
        lon2 = GPS.longitudeDegrees;
      }

      latitude = GPS.latitudeDegrees;  // Assign latitude in degrees (double)
      longitude = GPS.longitudeDegrees; // Assign longitude in degrees (double)
      altitude = GPS.altitude;          // Assign altitude (float)
      speed = GPS.speed;                // Assign speed (float)
      angle = GPS.angle;                // Assign angle (float)
      satellites = (int)GPS.satellites; // Assign number of satellites (int)

      mercatorProjection(lat1, lon1, x_1, y_1);
      mercatorProjection(latitude, longitude, x_2, y_2);



      deltaX = x_2 - x_1;
      deltaY = y_2 - y_1;

      // Serial.print("Location: ");
      // Serial.print(GPS.latitude, 6); Serial.print(GPS.lat);
      // Serial.print(" | ");
      // Serial.print(GPS.longitude, 4); Serial.print(GPS.lon);
      // Serial.print(" | ");
      // Serial.print(GPS.altitude);
      // Serial.print(" | ");
      // Serial.print(GPS.speed);
      // Serial.print(" | ");
      // Serial.print(GPS.angle);
      // Serial.print(" | ");
      // Serial.println((int)GPS.satellites);

      // Serial.print(latitude, 6);
      // Serial.print(" | ");
      // Serial.print(longitude, 6);
      // Serial.print(" | ");
      // Serial.print(altitude);
      // Serial.print(" | ");
      // Serial.print(speed);
      // Serial.print(" | ");
      // Serial.print(angle);
      // Serial.print(" | ");
      // Serial.println(satellites);

      // Serial.print("y1:");
      // Serial.print(y_1, 2);
      // Serial.print(",");
      // Serial.print("x1:");
      // Serial.print(x_1, 2);
      // Serial.print(" | ");
      // Serial.print("y2:");
      // Serial.print(y_2, 2);
      // Serial.print(",");
      // Serial.print("x1:");
      // Serial.print(x_2, 2);
      // Serial.print(" | ");
      // Serial.print("dx:");
      // Serial.print(deltaX, 2);
      // Serial.print(",");
      // Serial.print("dy:");
      // Serial.println(deltaY, 2);


      // Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
    }
  }
}

#endif

#include <Wire.h>

#ifdef ENABLE_INTERVAL
// Motor && Kalman && PID- 500 Hz
unsigned long pM500Hz = 0;        // PreviousMillis
const long interval500Hz = 2000;  // Interval to wait (2 ms a.k.a 2000 us)
// unsigned long cM500Hz = 0;                // CurrentMillis

// IMU && Lidar - 100 Hz
unsigned long pM100Hz = 0;      // PreviousMillis
const long interval100Hz = 10;  // Interval to wait (10 ms)
// unsigned long cM100Hz = 0;                // CurrentMillis
// unsigned long cM2_lidar = 0;

// 20Hz
unsigned long pM20Hz = 0;     // PreviousMillis
const long interval20Hz = 50;  // Interval to wait (10 ms)
// 5Hz
unsigned long pM5Hz = 0;     // PreviousMillis
const long interval5Hz = 200;  // Interval to wait (10 ms)

// GPS - 10 Hz
unsigned long pM10Hz = 0;       // PreviousMillis
const long interval10Hz = 100;  // Interval to wait (100 ms)
// unsigned long cM10Hz = 0;                // CurrentMillis

// LED & SerialMonitor - 1/5 Hz
unsigned long pM1Hz = 0;        // PreviousMillis
const long interval1Hz = 1000;  // Interval to wait (1000 ms)
// unsigned long cM1Hz = 0;                // CurrentMillis

unsigned long cMicros = 0;
unsigned long cMillis = 0;
unsigned long iteration = 0;


void initInterval() {
  cMicros = micros();
  cMillis = millis();
}

#elif
// Motor && Kalman && PID- 500 Hz
unsigned long pM500Hz = 0;        // PreviousMillis
const long interval500Hz = 2000;  // Interval to wait (2 ms a.k.a 2000 us)
// unsigned long cM500Hz = 0;                // CurrentMillis

// IMU && Lidar - 100 Hz
unsigned long pM100Hz = 0;     // PreviousMillis
const long interval100Hz = 0;  // Interval to wait (10 ms)
// unsigned long cM100Hz = 0;                // CurrentMillis
// unsigned long cM2_lidar = 0;

// 20Hz
unsigned long pM20Hz = 0;     // PreviousMillis
const long interval20Hz = 50;  // Interval to wait (10 ms)

// 5HZ
unsigned long pM5Hz = 0;     // PreviousMillis
const long interval5Hz = 200;  // Interval to wait (10 ms)


// GPS - 10 Hz
unsigned long pM10Hz = 0;     // PreviousMillis
const long interval10Hz = 0;  // Interval to wait (100 ms)
// unsigned long cM10Hz = 0;                // CurrentMillis

// LED & SerialMonitor - 1/5 Hz
unsigned long pM1Hz = 0;     // PreviousMillis
const long interval1Hz = 0;  // Interval to wait (1000 ms)
// unsigned long cM1Hz = 0;                // CurrentMillis
void initInterval() {
  cMicros = micros();
  cMillis = millis();
}

#endif

// 1. BNO055 - IMU --> Gyroscope & Acc & EulerAngle
#ifdef ENABLE_IMU
#include "DFRobot_BNO055.h"  //IMU --> Gyroscope / I2C / SDA0 SCL0
DFRobot_BNO055 mpu;
// Variable BNO055
float RateRoll, RatePitch, RateYaw;  // Gyro rate
float Roll, Pitch, Yaw, YawNorm;              // Euler angle
float manualCalPitch = 3.3; float manualCalRoll = -3;
float RateCalibrationPitch , RateCalibrationRoll, RateCalibrationYaw;
float AngleCalibrationPitch = 3.544, AngleCalibrationRoll = 3.038, AngleCalibrationYaw;
float RateRollCal = 0, RatePitchCal = 0, RateYawCal = 0;
float RollCal = 0, PitchCal = 0, YawCal = 0;
float AccXCal = 0, AccYCal = 0, AccZCal = 0;
float AccCalibrationX = 0, AccCalibrationY = 0, AccCalibrationZ = 0;
int RateCalibrationNumber;
float AccX,AccY,AccZ;
// Velocity and Position Estimation
float PrevVx=0, PrevVy=0, PrevVz=0;
float Vx, Vy, Vz;
float XPos1 = 0, YPos1 = 0, ZPos1 = 0;
float XPos2 = 0, YPos2 = 0, ZPos2 = 0;

// Quaternion components
float or_w, or_x, or_y, or_z;
// Calculate rotation matrix
float rotationMatrix[3][3];   // 3x3
float acc[3][0];              // 3x1
float result[3][0]; // Resultant matrix (3x1)

void quaternionToMatrix(float q0, float q1, float q2, float q3, float rotationMatrix[3][3]) {
  rotationMatrix[0][0] = 2 * (q0 * q0 + q1 * q1)-1;
  rotationMatrix[0][1] = 2 * (q1 * q2 - q0 * q3);
  rotationMatrix[0][2] = 2 * (q1 * q3 + q0 * q2);

  rotationMatrix[1][0] = 2 * (q1 * q2 + q0 * q3);
  rotationMatrix[1][1] = 2 * (q0 * q0 + q2 * q2)-1;
  rotationMatrix[1][2] = 2 * (q2 * q3 - q0 * q1);

  rotationMatrix[2][0] = 2 * (q1 * q3 - q0 * q2);
  rotationMatrix[2][1] = 2 * (q2 * q3 + q0 * q1);
  rotationMatrix[2][2] = 2 * (q0 * q0 + q3 * q3)-1;
}

//----SETUP BNO055
void initImu() {
  while (!mpu.init()) {
    Serial.println("ERROR! Unable to initialize the chip!");
    delay(50);
  }
  mpu.setMode(mpu.eNORMAL_POWER_MODE, mpu.eFASTEST_MODE);  // Fastest 100Hz
  delay(100);
  Serial.println("MPUINITSuccess");
  // Calibration Gyro
  // for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
  //   readImu();
  //   RateCalibrationRoll += RateRoll;
  //   RateCalibrationPitch += RatePitch;
  //   RateCalibrationYaw += RateYaw;
  //   // AngleCalibrationPitch += Pitch;
  //   // AngleCalibrationRoll += Roll;
  //   AngleCalibrationYaw += Yaw;
  //   AccCalibrationX += AccX;
  //   AccCalibrationY += AccY;
  //   AccCalibrationZ += AccZ;
  //   delay(1);
  // }
  // RateCalibrationRoll /= 2000;
  // RateCalibrationPitch /= 2000;
  // RateCalibrationYaw /= 2000;

  // // AngleCalibrationPitch /= 2000;
  // // AngleCalibrationRoll /= 2000;
  // AngleCalibrationYaw /= 2000;

  // AccCalibrationX /= 2000;
  // AccCalibrationY /= 2000;
  // AccCalibrationZ /= 2000;

  RateRollCal = RateCalibrationRoll;
  RatePitchCal = RateCalibrationPitch;
  RateYawCal = RateCalibrationYaw;
  RollCal = AngleCalibrationRoll;
  PitchCal = AngleCalibrationPitch;
  YawCal = AngleCalibrationYaw;
  AccXCal = AccCalibrationX;
  AccYCal = AccCalibrationY;
  AccZCal = AccCalibrationZ;

  Serial.println("CalibrationDone");
}

void readImu() {
  mpu.readAngularVelocity(); /* read Angular Velocity */
  mpu.readEuler();           /* read euler angle */
  mpu.readLinAcc();          /* read linear acceleration */
  mpu.readQua();
  // Variable IMU
  RateRoll = mpu.GyrData.x - RateRollCal;    // Roll Rate  (deg/s)
  RatePitch = mpu.GyrData.y - RatePitchCal;  // Pitch Rate  (deg/s)
  RateYaw = mpu.GyrData.z - RateYawCal;      // Yaw Rate  (deg/s)
  // Roll = mpu.EulerAngles.z - RollCal + manualCalRoll;        // Roll       (deg)
  // Pitch = -1*(mpu.EulerAngles.y + PitchCal + manualCalPitch );      // Pitch      (deg)
  // Roll = mpu.EulerAngles.z;
  
  Roll = mpu.EulerAngles.z + manualCalRoll;
  Pitch = -1*(mpu.EulerAngles.y + manualCalPitch);
  Yaw = 360 - mpu.EulerAngles.x + YawCal;          // Yaw        (deg)
  if (Yaw == 360){
    Yaw = 0;
  }
  YawNorm = Yaw;
  if (YawNorm > 180){
    YawNorm = YawNorm - 360;
  }
  AccX = -1*(mpu.LinAccData.x - AccXCal);
  AccY =  -1*(mpu.LinAccData.y - AccYCal);
  AccZ = -1*(mpu.LinAccData.z - AccZCal);

  or_w = mpu.QuaData.w;
  or_x = mpu.QuaData.x;
  or_y = mpu.QuaData.y;
  or_z = mpu.QuaData.z;


  if (Roll >= 60 || Roll <= -60 || Pitch >= 60 || Pitch <= -60 || Yaw > 361 || Yaw <= -60){
    SafetyState = false;
  }
  


#ifdef ENABLE_IMU_PRINT
  Serial.print("IMU| ");
  Serial.print("R:");
  Serial.print(Roll, 2);
  Serial.print(" | ");
  Serial.print("P:");
  Serial.print(Pitch, 2);
  Serial.print(" | ");
  Serial.print("Y:");
  Serial.print(Yaw, 2);
  Serial.print(" || ");
  Serial.print("Gx:");
  Serial.print(RateRoll, 2);
  Serial.print(" | ");
  Serial.print("Gy:");
  Serial.print(RatePitch, 2);
  Serial.print(" | ");
  Serial.print("Gz:");
  Serial.println(RateYaw, 2);
  
  // Serial.print("IMU_Pos| "); 
  // Serial.print("AccX:");
  // Serial.print(AccX, 2);
  // Serial.print(" | "); 
  // Serial.print("AccY:");
  // Serial.print(AccY, 2);
  // Serial.print(" | "); 
  // Serial.print("AccZ:");
  // Serial.print(AccZ, 2);
  // Serial.print(" || "); 
  // Serial.print("Vx:");
  // Serial.print(Vx, 2);
  // Serial.print(" | "); 
  // Serial.print("Vy:");
  // Serial.print(Vy, 2);
  // Serial.print(" | "); 
  // Serial.print("Vz:");
  // Serial.print(Vz, 2);
  // Serial.print(" || "); 
  // Serial.print("XPos1:");
  // Serial.print(XPos1, 2);
  // Serial.print(" | "); 
  // Serial.print("YPos1:");
  // Serial.print(YPos1, 2);
  // Serial.print(" | "); 
  // Serial.print("ZPos1:");
  // Serial.print(ZPos1, 2);
  // Serial.print(" || ");
  // Serial.print("XPos2:");
  // Serial.print(XPos2, 2);
  // Serial.print(" | "); 
  // Serial.print("YPos2:");
  // Serial.print(ZPos2, 2);
  // Serial.print(" | "); 
  // Serial.print("ZPos2:");
  // Serial.println(ZPos2, 2);
  
  // Serial.print("IMUCal| "); 
  // Serial.print("R:");
  // Serial.print(Roll, 3);
  // Serial.print(" | "); 
  // Serial.print("RCal :");
  // Serial.print(RollCal, 3);
  // Serial.print(" | "); 
  // Serial.print("RManual :");
  // Serial.print(manualCalRoll, 3);
  // Serial.print(" || "); 
  // Serial.print("P:");
  // Serial.print(Pitch, 3);
  // Serial.print(" | "); 
  // Serial.print("PCal :");
  // Serial.print(PitchCal, 3);
  // Serial.print(" | "); 
  // Serial.print("PManual :");
  // Serial.println(manualCalPitch, 3);
  
  
#endif
}

void positionImu(){
  // Cara Fadhly
  // quaternionToMatrix(or_x, or_y, or_z, or_w, rotationMatrix);
  // acc[0][0] = AccX;
  // acc[1][0] = AccY;
  // acc[2][0] = AccZ;

  // for (int i = 0; i < 3; i++) { // Iterate over rows of rotationMatrix
  //   result[i][0] = 0;           // Initialize result element to 0
  //   for (int j = 0; j < 3; j++) { // Iterate over columns of rotationMatrix and rows of acc
  //     result[i][0] += rotationMatrix[i][j] * acc[j][0];
  //     // Serial.print("Result: "); Serial.println(result[i][0]);
  //   }
  // }
  

  // float acc_world_x = rotationMatrix[0][0] * AccX + rotationMatrix[0][1] * AccY;
  // float acc_world_y = rotationMatrix[1][0] * AccX + rotationMatrix[1][1] * AccY;


  // Vx += acc_world_x * 0.01;
  // Vy += acc_world_y * 0.01;
  // XPos1 += Vx * 0.01;
  // YPos1 += Vy * 0.01;

  // // Vx = PrevVx + AccX * 0.01; // 0.01 = dt
  // // Vy = PrevVy + AccY * 0.01;
  // // Vz = PrevVy + AccZ * 0.01;

  // // PrevVx = Vx;
  // // PrevVy = Vy;
  // // PrevVz = Vz;  

  // // XPos1 = XPos1 + Vx*0.01;
  // // YPos1 = YPos1 + Vy*0.01;
  // // ZPos1 = ZPos1 + Vz*0.01;
  
  // // Cara Lain
  // // x = 1/2 * a * t^2
  // XPos2 = XPos2 + 0.5 * AccX * 0.01 * 0.01;
  // YPos2 = YPos2 + 0.5 * AccY * 0.01 * 0.01;
  // ZPos2 = ZPos2 + 0.5 * AccZ * 0.01 * 0.01;

}

#endif

// 2. BMP280 - Atmosphere Pressure Sensor --> Altitude
#ifdef ENABLE_BMP
#include "DFRobot_BMP280.h"

typedef DFRobot_BMP280_IIC BMP;  // ******** use abbreviations instead of full names ********
BMP bmp(&Wire, BMP::eSdoLow);
#define SEA_LEVEL_PRESSURE 1015.0f  // sea level pressure

// Variable BMP280
float temp = 0;
uint32_t press = 0;
float alti = 0;

//Setup BMP280
void initBmp() {
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

void readBmp() {
  temp = bmp.getTemperature();
  press = bmp.getPressure();
  alti = bmp.calAltitude(SEA_LEVEL_PRESSURE, press);

#ifdef ENABLE_BMP_PRINT
  Serial.print("BMP| ");
  Serial.print("Alt:");
  Serial.print(alti);
  Serial.print(" | ");
  Serial.print("Pressure:");
  Serial.print(press);
  Serial.print(" | ");
  Serial.print("Temp:");
  Serial.println(temp);
#endif
}

#endif

// 3. LIDAR
#ifdef ENABLE_LIDAR
#define LIDAR Serial5  // Define the Serial1 for TF-Luna communication

uint8_t recvBuffer[9];  // Buffer to store received data
float Altitude;           // Altitude measured by the LiDAR
int PrevAltitude;
int strength;           // Signal strength
float verticalVelocity = 0, prevAltitude = 0;
float lidarPeriod = 0;
float lidarTimer = 0, lidarTimerPrev = 0;

void initLidar() {
  LIDAR.begin(115200);  // LiDAR UART
  Serial.println("TF-Luna LiDAR End");
}

void readLidar() {
  if (LIDAR.available()) {
    // Read one byte at a time into the buffer
    if (LIDAR.readBytes(recvBuffer, 9) == 9) {  // TF-Luna sends data in packets of 9 bytes
      // Verify if the packet is valid (check the frame header)
      if (recvBuffer[0] == 0x59 && recvBuffer[1] == 0x59) {
        Altitude = recvBuffer[2] + (recvBuffer[3] << 8);  // Calculate Altitude (low byte + high byte)
        strength = recvBuffer[4] + (recvBuffer[5] << 8);  // Calculate signal strength
        if (Altitude < 3){
          Altitude = PrevAltitude;
          // PrevAltitude = Altitude;
        }
        else{
          PrevAltitude = Altitude;
        }
         
        
        unsigned long localMillis = millis();
        lidarTimer =  localMillis - lidarTimerPrev;
        lidarTimerPrev = localMillis;
        velocityLidar();
        
#ifdef ENABLE_LIDAR_PRINT
        Serial.print("Lidar| ");
        Serial.print("Dist: ");
        Serial.print(Altitude);
        Serial.print(" | ");
        Serial.print("VVelocity: ");
        Serial.print(verticalVelocity);
        Serial.print(" | ");
        Serial.print("Strength: ");
        Serial.print(strength);
        Serial.print(" | ");
        Serial.print("period: ");
        Serial.println(lidarTimer);
#endif
      }
    }
  }
}

void velocityLidar(){
  verticalVelocity = (Altitude - prevAltitude)/lidarTimer*1000/10;  // /1000 c/ Lidar Timer in ms -> s, /10 cause in cm -> 0.1m
  prevAltitude = Altitude;
}

#endif

// 4. Voltage and Current Sensor
#ifdef ENABLE_VOLTAGE_CURRENT

#define voltageSensorPin 22  //Voltage Sensor from Power Module
#define currentSensorPin 23  //Current Sensor from Power Module

int voltageBit, currentBit = 0;
float voltageVolt, currentVolt, voltage, current = 0;


void initVoltageCurrent() {
  pinMode(voltageSensorPin, INPUT);
  pinMode(currentSensorPin, INPUT);
}

void readVoltageCurrent() {
  voltageBit = analogRead(voltageSensorPin);  // Membaca nilai analog dari pin A0
  currentBit = analogRead(currentSensorPin);  // Membaca nilai analog dari pin A0
  voltageVolt = (float)voltageBit / 1023.0 * 3.3;
  currentVolt = (float)currentBit / 1023.0 * 3.3;
  voltage = voltageVolt * 35.34 / 3.3;  //maximum 30V, calibration value: 35.34/3.3
  current = currentVolt * 90;           //maximum 90A, not yet calibrated

#ifdef ENABLE_VOLTAGE_CURRENT_PRINT
  Serial.print("Analog| ");
  Serial.print("V: ");
  Serial.print(voltage);
  Serial.print(" | ");
  Serial.print("I: ");
  Serial.print(current);
  Serial.print(" | ");
  Serial.print("V_volt: ");
  Serial.print(voltageVolt);
  Serial.print(" | ");
  Serial.print("I_volt: ");
  Serial.println(currentVolt);
#endif
}

#endif

// 5. Motor Control

// 9. Receiver
#ifdef ENABLE_RECEIVER

#include <PulsePosition.h>
#define receiverPin 12
PulsePositionInput ReceiverInput(RISING);
float ReceiverValue[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int ChannelNumber = 6;

float channel_roll_pwm, channel_pitch_pwm, channel_thr_pwm, channel_yaw_pwm = 0;

void initReceiver() {
  // DesiredRateRoll = 0;
  ReceiverInput.begin(receiverPin);
  while (ReceiverValue[2] < 1005 || ReceiverValue[2] > 1050) {
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
  // if (ReceiverValue[0] > 1530 && ReceiverValue[0] < 1470){
  //   ReceiverValue[0] = 1500;
  // }
  // DesiredRateRoll = (ReceiverValue[0] - 1522.0)/500.0*(-30.0);
  // // Pitch = 1
  // input_throttle = ReceiverValue[2];
  // Yaw = 3
  channel_roll_pwm = ReceiverValue[0];   // 1500 initial [1000 - 1500 - 2000]
  channel_pitch_pwm = ReceiverValue[1];  // 1500 intial  [1000 - 1500 - 2000]
  channel_thr_pwm = ReceiverValue[2];    // 1000 initial [1000 - 2000]
  channel_yaw_pwm = ReceiverValue[3];    // 1500 initial [1000 - 1500 - 2000]

#ifdef ENABLE_RECEIVER_PRINT
  Serial.print("RECEIVER: ");
  Serial.print("T: ");
  Serial.print(channel_thr_pwm);
  Serial.print(" | ");
  Serial.print("R: ");
  Serial.print(channel_roll_pwm);
  Serial.print(" | ");
  Serial.print("P: ");
  Serial.print(channel_pitch_pwm);
  Serial.print(" | ");
  Serial.print("Y: ");
  Serial.println(channel_yaw_pwm);
#endif
}
#else
float channel_roll_pwm = 1500, channel_pitch_pwm = 1500, channel_thr_pwm = 1000, channel_yaw_pwm = 1500;
#endif

#ifdef ENABLE_MOTOR
// DEFAULT Quad 4 Motor
#define MotorPin1 2  //M1 - Quad
#define MotorPin2 3  //M2 - Quad
#define MotorPin3 4  //M3 - Quad
#define MotorPin4 5  //M4 - Quad


#ifdef ENABLE_MOTOR_HEXACOPTER
#define MotorPin5 6  //M5 - Hexa
#define MotorPin6 9  //M6 - Hexa
#endif

#define MIN_PULSE_LENGTH 1000  // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000  // Maximum pulse length in µs

float MotorInput1 = MIN_PULSE_LENGTH, MotorInput2 = MIN_PULSE_LENGTH, MotorInput3 = MIN_PULSE_LENGTH;
float MotorInput4 = MIN_PULSE_LENGTH, MotorInput5 = MIN_PULSE_LENGTH, MotorInput6 = MIN_PULSE_LENGTH;


float input_throttle = MIN_PULSE_LENGTH;
float motor1_throttle = 0;
float motor2_throttle = 0;
float motor3_throttle = 0;
float motor4_throttle = 0;
float motor5_throttle = 0;
float motor6_throttle = 0;

void initMotor() {
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

void runMotor(float m1 = 1000.0, float m2 = 1000.0, float m3 = 1000.0, float m4 = 1000.0, float m5 = 1000.0, float m6 = 1000.0) {
  // analogWrite(motorTestPin,1.024*thr);
  // Serial.println(thr);
  // float m1= m1;
  analogWrite(MotorPin1, m1 + 12);
  analogWrite(MotorPin2, m2 + 12);
  analogWrite(MotorPin3, m3 + 12);
  analogWrite(MotorPin4, m4 + 12);

#ifdef ENABLE_MOTOR_HEXACOPTER
  analogWrite(MotorPin5, m5 + 12);
  analogWrite(MotorPin6, m6 + 12);
#endif

#ifdef ENABLE_MOTOR_PRINT
  Serial.print("Motor| ");
  Serial.print("M1: ");
  Serial.print(m1);
  Serial.print("| M2: ");
  Serial.print(m2);
  Serial.print("| M3: ");
  Serial.print(m3);
  Serial.print("| M4: ");
  Serial.print(m4);
  Serial.print("| M5: ");
  Serial.print(m5);
  Serial.print("| M6: ");
  Serial.println(m6);
#endif
}
#endif

// 8. PID
float setPointRpm = 0;
float RefRateRoll = 0;
float RefRatePitch = 0;
float RefRateYaw = 0;
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw, DesiredVerticalVelocity, DesiredAltitude;
#ifdef ENABLE_PID_RATE
// Inner Control
// setPointRpm = 0;
// float DesiredRpm = setPointRpm;

// Supervisory Control

float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = { 0, 0, 0 };
float PRateRoll = 1;
float PRatePitch = 0;
float PRateYaw = 0;
float IRateRoll = 0;
float IRatePitch = 0;
float IRateYaw = 0;
float DRateRoll = 0;
float DRatePitch = 0;
float DRateYaw = 0;
// float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
// SetPoint Controller PID
float ManualThrottle = input_throttle;
float ManualRateRoll = RefRateRoll;
float ManualRatePitch = RefRatePitch;
float ManualRateYaw = RefRateYaw;



void initPidRate() {
  // DesiredRateRoll=0.15*(ReceiverValue[0]-1500);
  DesiredRateRoll = ManualRateRoll;
  // DesiredRatePitch=0.15*(ReceiverValue[1]-1500);
  DesiredRatePitch = ManualRatePitch;
  // InputThrottle=ReceiverValue[2];
  InputThrottle = ManualThrottle;
  // DesiredRateYaw=0.15*(ReceiverValue[3]-1500);
  DesiredRateYaw = ManualRateYaw;
}

void resetPidRate(void) {
  PrevErrorRateRoll = 0;
  PrevErrorRatePitch = 0;
  PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0;
  PrevItermRatePitch = 0;
  PrevItermRateYaw = 0;
}


void pidRateEquation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.01 / 2;
  if (Iterm > 400) Iterm = 400;
  else if (Iterm < -400) Iterm = -400;
  float Dterm = D * (Error - PrevError) / 0.01;
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400) PIDOutput = 400;
  else if (PIDOutput < -400) PIDOutput = -400;
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void errorPidRate() {
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;
}
void loopPidRate() {
  errorPidRate();
  pidRateEquation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1];
  PrevItermRateRoll = PIDReturn[2];
  pidRateEquation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];
  PrevItermRatePitch = PIDReturn[2];
  pidRateEquation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw = PIDReturn[0];
  PrevErrorRateYaw = PIDReturn[1];
  PrevItermRateYaw = PIDReturn[2];

  InputThrottle = input_throttle;

  if (InputThrottle > 1800) {
    InputThrottle = 1800;
  }


  MotorInput1 = (InputThrottle - InputRoll);        //
  MotorInput2 = (InputThrottle + InputRoll);        //
  MotorInput3 = (InputThrottle + 0.5 * InputRoll);  //
  MotorInput4 = (InputThrottle - 0.5 * InputRoll);  //
  MotorInput5 = (InputThrottle - 0.5 * InputRoll);  //
  MotorInput6 = (InputThrottle + 0.5 * InputRoll);  //

  if (MotorInput1 > 2000) {
    MotorInput1 = 1999;
  }

  if (MotorInput2 > 2000) {
    MotorInput2 = 1999;
  }

  if (MotorInput3 > 2000) {
    MotorInput3 = 1999;
  }

  if (MotorInput4 > 2000) {
    MotorInput4 = 1999;
  }

  if (MotorInput5 > 2000) {
    MotorInput5 = 1999;
  }

  if (MotorInput6 > 2000) {
    MotorInput6 = 1999;
  }

  int ThrottleIdle = 1100;
  int ThrottleCutOff = 1000;
  if (MotorInput1 < ThrottleIdle) {
    MotorInput1 = ThrottleIdle;
  }
  if (MotorInput2 < ThrottleIdle) {
    MotorInput2 = ThrottleIdle;
  }
  if (MotorInput3 < ThrottleIdle) {
    MotorInput3 = ThrottleIdle;
  }
  if (MotorInput4 < ThrottleIdle) {
    MotorInput4 = ThrottleIdle;
  }
  if (MotorInput5 < ThrottleIdle) {
    MotorInput5 = ThrottleIdle;
  }
  if (MotorInput6 < ThrottleIdle) {
    MotorInput6 = ThrottleIdle;
  }

  // int ThrottleCutOff=1000;
  if (InputThrottle < 1050) {
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;
    MotorInput5 = ThrottleCutOff;
    MotorInput6 = ThrottleCutOff;
    resetPidRate();
  }

#ifdef ENABLE_PID_RATE_PRINT
  Serial.print("PID| ");
  Serial.print("iT:");
  Serial.print(InputThrottle);
  Serial.print(" | ");
  Serial.print("iR:");
  Serial.print(InputRoll);
  Serial.print(" | ");
  Serial.print("iP:");
  Serial.print(InputPitch);
  Serial.print(" | ");
  Serial.print("iY:");
  Serial.println(InputYaw);

  Serial.print("MotorPID| ");
  Serial.print("M1P:");
  Serial.print(MotorInput1);
  Serial.print(" | ");
  Serial.print("M2P:");
  Serial.print(MotorInput2);
  Serial.print(" | ");
  Serial.print("M3P:");
  Serial.print(MotorInput3);
  Serial.print(" | ");
  Serial.print("M4P:");
  Serial.print(MotorInput4);
  Serial.print(" | ");
  Serial.print("M5P:");
  Serial.print(MotorInput5);
  Serial.print(" | ");
  Serial.print("M6P:");
  Serial.println(MotorInput6);

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
float PIDAngleReturn[] = { 0, 0, 0 };
float PAngleRoll = 1;
float PAnglePitch = 0;
float IAngleRoll = 2;
float IAnglePitch = 0;
float DAngleRoll = 0;
float DAnglePitch = 0;

float PYaw = 0;
float IYaw = 0;
float DYaw = 0;
// float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
// SetPoint Controller PID
// float ManualThrottle = input_throttle;
float ManualRoll = refRoll;    // deg
float ManualPitch = refPitch;  // deg
float ManualYaw = refYaw;      // deg

void initPidAngle() {
  // DesiredRateRoll=0.15*(ReceiverValue[0]-1500);
  DesiredRoll = ManualRoll;
  // DesiredRatePitch=0.15*(ReceiverValue[1]-1500);
  DesiredPitch = ManualPitch;
  // InputThrottle=ReceiverValue[2];
  // InputThrottle=ManualThrottle;
  // DesiredRateYaw=0.15*(ReceiverValue[3]-1500);
  DesiredYaw = ManualYaw;
}

void pidAngleEquation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.01 / 2;
  if (Iterm > 400) Iterm = 400;
  else if (Iterm < -400) Iterm = -400;
  float Dterm = D * (Error - PrevError) / 0.004;
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 300) PIDOutput = 300;
  else if (PIDOutput < -300) PIDOutput = -300;
  PIDAngleReturn[0] = PIDOutput;
  PIDAngleReturn[1] = Error;
  PIDAngleReturn[2] = Iterm;
}

void resetPidAngle(void) {
  PrevErrorRoll = 0;
  PrevErrorPitch = 0;
  PrevErrorYaw = 0;
  PrevItermRoll = 0;
  PrevItermPitch = 0;
  PrevItermYaw = 0;
}

void errorPidAngle() {
  ErrorRoll = DesiredRoll - Roll;
  ErrorPitch = DesiredPitch - Pitch;
  ErrorYaw = DesiredYaw - Yaw;
}

#ifdef ENABLE_PID_ANGLE_ONLY

void loopPidAngle() {
  errorPidAngle();
  pidAngleEquation(ErrorRoll, PAngleRoll, IAnglePitch, DAngleRoll, PrevErrorRoll, PrevItermRoll);
  InputRoll = PIDAngleReturn[0];
  PrevErrorRoll = PIDAngleReturn[1];
  PrevItermRoll = PIDAngleReturn[2];
  pidAngleEquation(ErrorPitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorPitch, PrevItermPitch);
  InputPitch = PIDAngleReturn[0];
  PrevErrorPitch = PIDAngleReturn[1];
  PrevItermPitch = PIDAngleReturn[2];
  pidAngleEquation(ErrorYaw, PYaw, IYaw, DYaw, PrevErrorYaw, PrevItermYaw);
  InputYaw = PIDAngleReturn[0];
  PrevErrorYaw = PIDAngleReturn[1];
  PrevItermYaw = PIDAngleReturn[2];

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

  MotorInput1 = 1.024 * (InputThrottle - InputPitch);
  MotorInput2 = 1.024 * (InputThrottle - InputRoll);
  MotorInput3 = 1.024 * (InputThrottle + InputPitch);
  MotorInput4 = 1.024 * (InputThrottle + InputRoll);

  int MaxMotorInput = 1700;
  if (MotorInput1 > MaxMotorInput) MotorInput1 = MaxMotorInput;
  if (MotorInput2 > MaxMotorInput) MotorInput2 = MaxMotorInput;
  if (MotorInput3 > MaxMotorInput) MotorInput3 = MaxMotorInput;
  if (MotorInput4 > MaxMotorInput) MotorInput4 = MaxMotorInput;

  int ThrottleIdle = 1100;
  if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
  if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
  if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
  if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;

  int ThrottleCutOff = 1000;
  if (InputThrottle < 1050) {
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;
    resetPidAngle();
  }


#ifdef ENABLE_PID_ANGLE_PRINT
  Serial.print("PIDAngle| ");
  Serial.print("iT:");
  Serial.print(InputThrottle);
  Serial.print(" | ");
  Serial.print("rR:");
  Serial.print(InputRoll);
  Serial.print(" | ");
  Serial.print("rP:");
  Serial.print(InputPitch);
  Serial.print(" | ");
  Serial.print("rY:");
  Serial.println(InputYaw);

  Serial.print("MotorPID| ");
  Serial.print("M1P:");
  Serial.print(MotorInput1);
  Serial.print(" | ");
  Serial.print("M2P:");
  Serial.print(MotorInput2);
  Serial.print(" | ");
  Serial.print("M3P:");
  Serial.print(MotorInput3);
  Serial.print(" | ");
  Serial.print("M4P:");
  Serial.print(MotorInput4);
  Serial.print(" | ");
  Serial.print("M5P:");
  Serial.print(MotorInput5);
  Serial.print(" | ");
  Serial.print("M6P:");
  Serial.println(MotorInput6);
#endif
}

#elif
void loopPidAngle() {
  errorPidAngle();
  pidAngleEquation(ErrorRoll, PAngleRoll, IAnglePitch, DAngleRoll, PrevErrorRoll, PrevItermRoll);
  RefRateRoll = PIDAngleReturn[0];
  PrevErrorRoll = PIDAngleReturn[1];
  PrevItermRoll = PIDAngleReturn[2];
  pidAngleEquation(ErrorPitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorPitch, PrevItermPitch);
  RefRatePitch = PIDAngleReturn[0];
  PrevErrorPitch = PIDAngleReturn[1];
  PrevItermPitch = PIDAngleReturn[2];
  pidAngleEquation(ErrorYaw, PYaw, IYaw, DYaw, PrevErrorYaw, PrevItermYaw);
  RefRateYaw = PIDAngleReturn[0];
  PrevErrorYaw = PIDAngleReturn[1];
  PrevItermYaw = PIDAngleReturn[2];

#ifdef ENABLE_PID_ANGLE_PRINT
  Serial.print("PIDAngle| ");
  // Serial.print("iT:");Serial.print(InputThrottle); Serial.print(" | ");
  Serial.print("rR:");
  Serial.print(RefRateRoll);
  Serial.print(" | ");
  Serial.print("rP:");
  Serial.print(RefRatePitch);
  Serial.print(" | ");
  Serial.print("rY:");
  Serial.println(RefRateYaw);

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

  // InputThrottle = input_throttle;

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
FuzzySet *RollVeryLow = new FuzzySet(-180, -180, -13.35, -5);
FuzzySet *RollLow = new FuzzySet(-13.35, -5, -5, 0);
FuzzySet *RollNormal = new FuzzySet(-5, 0, 0, 5);
FuzzySet *RollHigh = new FuzzySet(0, 5, 5, 13.35);
FuzzySet *RollVeryHigh = new FuzzySet(5, 13.35, 180, 180);

//INPUT Kecepatan R
FuzzySet *SpeedRVeryLow = new FuzzySet(-30, -30, -3, -1.5);
FuzzySet *SpeedRLow = new FuzzySet(-3, -1.5, -1.5, 0);
FuzzySet *SpeedRNormal = new FuzzySet(-1.5, 0, 0, 1.5);
FuzzySet *SpeedRHigh = new FuzzySet(0, 1.5, 1.5, 3);
FuzzySet *SpeedRVeryHigh = new FuzzySet(1.5, 3, 30, 30);

//OUTPUT PWM
FuzzySet *RPWMVeryLow = new FuzzySet(-0.15, -0.15, -0.1, -0.05);
FuzzySet *RPWMLow = new FuzzySet(-0.1, -0.05, -0.05, 0);
FuzzySet *RPWMNormal = new FuzzySet(-0.05, 0, 0, 0.05);
FuzzySet *RPWMHigh = new FuzzySet(0, 0.05, 0.05, 0.1);
FuzzySet *RPWMVeryHigh = new FuzzySet(0.05, 0.1, 0.15, 0.15);
// FuzzySet *RPWMVeryLow = new FuzzySet(-30, -30, -20, -10);
// FuzzySet *RPWMLow = new FuzzySet(-20, -10, -10, 0);
// FuzzySet *RPWMNormal = new FuzzySet(-10, 0, 0, 10);
// FuzzySet *RPWMHigh = new FuzzySet(0, 10, 10, 20);
// FuzzySet *RPWMVeryHigh = new FuzzySet(10, 20, 30, 30);

float Throttle_Altitude, refAltitude,prev_Altitude, multiplyOutputFuzzyRoll;
float eAltitude, dEAltitude;
bool StartFuzzyRoll = 0;

void initFuzzyRoll() {
  refAltitude = 100;
  Throttle_Altitude = MIN_PULSE_LENGTH;
  prev_Altitude = 0;

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
  FuzzyRuleAntecedent *ifRollVeryLowAndSpeedRVeryLow = new FuzzyRuleAntecedent();
  ifRollVeryLowAndSpeedRVeryLow->joinWithAND(RollVeryLow, SpeedRVeryLow);
  FuzzyRuleConsequent *thenRPWMVeryHigh = new FuzzyRuleConsequent();
  thenRPWMVeryHigh->addOutput(RPWMVeryHigh);
  FuzzyRule *rule1 = new FuzzyRule(1, ifRollVeryLowAndSpeedRVeryLow, thenRPWMVeryHigh);
  fuzzy->addFuzzyRule(rule1);

  FuzzyRuleAntecedent *ifRollVeryLowAndSpeedRLow = new FuzzyRuleAntecedent();
  ifRollVeryLowAndSpeedRLow->joinWithAND(RollVeryLow, SpeedRLow);
  // FuzzyRuleConsequent *thenRPWMVeryHigh = new FuzzyRuleConsequent();
  // thenRPWMVeryHigh->addOutput(RPWMVeryHigh);
  FuzzyRule *rule2 = new FuzzyRule(2, ifRollVeryLowAndSpeedRLow, thenRPWMVeryHigh);
  fuzzy->addFuzzyRule(rule2);

  FuzzyRuleAntecedent *ifRollVeryLowAndSpeedRNormal = new FuzzyRuleAntecedent();
  ifRollVeryLowAndSpeedRNormal->joinWithAND(RollVeryLow, SpeedRNormal);
  // FuzzyRuleConsequent *thenRPWMVeryHigh = new FuzzyRuleConsequent();
  // thenRPWMVeryHigh->addOutput(RPWMVeryHigh);
  FuzzyRule *rule3 = new FuzzyRule(3, ifRollVeryLowAndSpeedRNormal, thenRPWMVeryHigh);
  fuzzy->addFuzzyRule(rule3);

  FuzzyRuleAntecedent *ifRollVeryLowAndSpeedRHigh = new FuzzyRuleAntecedent();
  ifRollVeryLowAndSpeedRHigh->joinWithAND(RollVeryLow, SpeedRHigh);
  FuzzyRuleConsequent *thenRPWMHigh = new FuzzyRuleConsequent();
  thenRPWMHigh->addOutput(RPWMHigh);
  FuzzyRule *rule4 = new FuzzyRule(4, ifRollVeryLowAndSpeedRHigh, thenRPWMHigh);
  fuzzy->addFuzzyRule(rule4);

  FuzzyRuleAntecedent *ifRollVeryLowAndSpeedRVeryHigh = new FuzzyRuleAntecedent();
  ifRollVeryLowAndSpeedRVeryHigh->joinWithAND(RollVeryLow, SpeedRVeryHigh);
  FuzzyRuleConsequent *thenRPWMNormal = new FuzzyRuleConsequent();
  thenRPWMNormal->addOutput(RPWMNormal);
  FuzzyRule *rule5 = new FuzzyRule(5, ifRollVeryLowAndSpeedRVeryHigh, thenRPWMNormal);
  fuzzy->addFuzzyRule(rule5);

  //--Roll Low
  FuzzyRuleAntecedent *ifRollLowAndSpeedRVeryLow = new FuzzyRuleAntecedent();
  ifRollLowAndSpeedRVeryLow->joinWithAND(RollLow, SpeedRVeryLow);
  // FuzzyRuleConsequent *thenRPWMVeryHigh = new FuzzyRuleConsequent();
  // thenRPWMVeryHigh->addOutput(RPWMVeryHigh);
  FuzzyRule *rule6 = new FuzzyRule(6, ifRollLowAndSpeedRVeryLow, thenRPWMVeryHigh);
  fuzzy->addFuzzyRule(rule6);

  FuzzyRuleAntecedent *ifRollLowAndSpeedRLow = new FuzzyRuleAntecedent();
  ifRollLowAndSpeedRLow->joinWithAND(RollLow, SpeedRLow);
  // FuzzyRuleConsequent *thenRPWMVeryHigh = new FuzzyRuleConsequent();
  // thenRPWMVeryHigh->addOutput(RPWMVeryHigh);
  FuzzyRule *rule7 = new FuzzyRule(7, ifRollLowAndSpeedRLow, thenRPWMVeryHigh);
  fuzzy->addFuzzyRule(rule7);

  FuzzyRuleAntecedent *ifRollLowAndSpeedRNormal = new FuzzyRuleAntecedent();
  ifRollLowAndSpeedRNormal->joinWithAND(RollLow, SpeedRNormal);
  // FuzzyRuleConsequent *thenRPWMHigh = new FuzzyRuleConsequent();
  // thenRPWMHigh->addOutput(RPWMHigh);
  FuzzyRule *rule8 = new FuzzyRule(8, ifRollLowAndSpeedRNormal, thenRPWMHigh);
  fuzzy->addFuzzyRule(rule8);

  FuzzyRuleAntecedent *ifRollLowAndSpeedRHigh = new FuzzyRuleAntecedent();
  ifRollLowAndSpeedRHigh->joinWithAND(RollLow, SpeedRHigh);
  // FuzzyRuleConsequent *thenRPWMNormal = new FuzzyRuleConsequent();
  // thenRPWMNormal->addOutput(RPWMNormal);
  FuzzyRule *rule9 = new FuzzyRule(9, ifRollLowAndSpeedRHigh, thenRPWMNormal);
  fuzzy->addFuzzyRule(rule9);

  FuzzyRuleAntecedent *ifRollLowAndSpeedRVeryHigh = new FuzzyRuleAntecedent();
  ifRollLowAndSpeedRVeryHigh->joinWithAND(RollLow, SpeedRVeryHigh);
  FuzzyRuleConsequent *thenRPWMLow = new FuzzyRuleConsequent();
  thenRPWMLow->addOutput(RPWMLow);
  FuzzyRule *rule10 = new FuzzyRule(10, ifRollLowAndSpeedRVeryHigh, thenRPWMLow);
  fuzzy->addFuzzyRule(rule10);

  //--Roll Normal
  FuzzyRuleAntecedent *ifRollNormalAndSpeedRVeryLow = new FuzzyRuleAntecedent();
  ifRollNormalAndSpeedRVeryLow->joinWithAND(RollNormal, SpeedRVeryLow);
  // FuzzyRuleConsequent *thenRPWMVeryHigh = new FuzzyRuleConsequent();
  // thenRPWMVeryHigh->addOutput(RPWMVeryHigh);
  FuzzyRule *rule11 = new FuzzyRule(11, ifRollNormalAndSpeedRVeryLow, thenRPWMVeryHigh);
  fuzzy->addFuzzyRule(rule11);

  FuzzyRuleAntecedent *ifRollNormalAndSpeedRLow = new FuzzyRuleAntecedent();
  ifRollNormalAndSpeedRLow->joinWithAND(RollNormal, SpeedRLow);
  // FuzzyRuleConsequent *thenRPWMVeryHigh = new FuzzyRuleConsequent();
  // thenRPWMVeryHigh->addOutput(RPWMVeryHigh);
  FuzzyRule *rule12 = new FuzzyRule(12, ifRollNormalAndSpeedRLow, thenRPWMHigh);
  fuzzy->addFuzzyRule(rule12);

  FuzzyRuleAntecedent *ifRollNormalAndSpeedRNormal = new FuzzyRuleAntecedent();
  ifRollNormalAndSpeedRNormal->joinWithAND(RollNormal, SpeedRNormal);
  // FuzzyRuleConsequent *thenRPWMVeryHigh = new FuzzyRuleConsequent();
  // thenRPWMVeryHigh->addOutput(RPWMVeryHigh);
  FuzzyRule *rule13 = new FuzzyRule(13, ifRollNormalAndSpeedRNormal, thenRPWMNormal);
  fuzzy->addFuzzyRule(rule13);

  FuzzyRuleAntecedent *ifRollNormalAndSpeedRHigh = new FuzzyRuleAntecedent();
  ifRollNormalAndSpeedRHigh->joinWithAND(RollNormal, SpeedRHigh);
  // FuzzyRuleConsequent *thenRPWMLow = new FuzzyRuleConsequent();
  // thenRPWMLow->addOutput(RPWMLow);
  FuzzyRule *rule14 = new FuzzyRule(14, ifRollNormalAndSpeedRHigh, thenRPWMLow);
  fuzzy->addFuzzyRule(rule14);

  FuzzyRuleAntecedent *ifRollNormalAndSpeedRVeryHigh = new FuzzyRuleAntecedent();
  ifRollNormalAndSpeedRVeryHigh->joinWithAND(RollNormal, SpeedRVeryHigh);
  FuzzyRuleConsequent *thenRPWMVeryLow = new FuzzyRuleConsequent();
  thenRPWMVeryLow->addOutput(RPWMVeryLow);
  FuzzyRule *rule15 = new FuzzyRule(15, ifRollNormalAndSpeedRVeryHigh, thenRPWMVeryLow);
  fuzzy->addFuzzyRule(rule15);

  //--Roll High
  FuzzyRuleAntecedent *ifRollHighAndSpeedRVeryLow = new FuzzyRuleAntecedent();
  ifRollHighAndSpeedRVeryLow->joinWithAND(RollHigh, SpeedRVeryLow);
  FuzzyRule *rule16 = new FuzzyRule(16, ifRollHighAndSpeedRVeryLow, thenRPWMHigh);
  fuzzy->addFuzzyRule(rule16);

  FuzzyRuleAntecedent *ifRollHighAndSpeedRLow = new FuzzyRuleAntecedent();
  ifRollHighAndSpeedRLow->joinWithAND(RollHigh, SpeedRLow);
  FuzzyRule *rule17 = new FuzzyRule(17, ifRollHighAndSpeedRLow, thenRPWMNormal);
  fuzzy->addFuzzyRule(rule17);

  FuzzyRuleAntecedent *ifRollHighAndSpeedRNormal = new FuzzyRuleAntecedent();
  ifRollHighAndSpeedRNormal->joinWithAND(RollHigh, SpeedRNormal);
  FuzzyRule *rule18 = new FuzzyRule(18, ifRollHighAndSpeedRNormal, thenRPWMLow);
  fuzzy->addFuzzyRule(rule18);

  FuzzyRuleAntecedent *ifRollHighAndSpeedRHigh = new FuzzyRuleAntecedent();
  ifRollHighAndSpeedRHigh->joinWithAND(RollHigh, SpeedRHigh);
  FuzzyRule *rule19 = new FuzzyRule(19, ifRollHighAndSpeedRHigh, thenRPWMVeryLow);
  fuzzy->addFuzzyRule(rule19);

  FuzzyRuleAntecedent *ifRollHighAndSpeedRVeryHigh = new FuzzyRuleAntecedent();
  ifRollHighAndSpeedRVeryHigh->joinWithAND(RollHigh, SpeedRVeryHigh);
  FuzzyRule *rule20 = new FuzzyRule(20, ifRollHighAndSpeedRVeryHigh, thenRPWMVeryLow);
  fuzzy->addFuzzyRule(rule20);

  //--Roll Very High
  FuzzyRuleAntecedent *ifRollVeryHighAndSpeedRVeryLow = new FuzzyRuleAntecedent();
  ifRollVeryHighAndSpeedRVeryLow->joinWithAND(RollVeryHigh, SpeedRVeryLow);
  FuzzyRule *rule21 = new FuzzyRule(21, ifRollVeryHighAndSpeedRVeryLow, thenRPWMNormal);
  fuzzy->addFuzzyRule(rule21);

  FuzzyRuleAntecedent *ifRollVeryHighAndSpeedRLow = new FuzzyRuleAntecedent();
  ifRollVeryHighAndSpeedRLow->joinWithAND(RollVeryHigh, SpeedRLow);
  FuzzyRule *rule22 = new FuzzyRule(22, ifRollVeryHighAndSpeedRLow, thenRPWMLow);
  fuzzy->addFuzzyRule(rule22);

  FuzzyRuleAntecedent *ifRollVeryHighAndSpeedRNormal = new FuzzyRuleAntecedent();
  ifRollVeryHighAndSpeedRNormal->joinWithAND(RollVeryHigh, SpeedRNormal);
  FuzzyRule *rule23 = new FuzzyRule(23, ifRollVeryHighAndSpeedRNormal, thenRPWMVeryLow);
  fuzzy->addFuzzyRule(rule23);

  FuzzyRuleAntecedent *ifRollVeryHighAndSpeedRHigh = new FuzzyRuleAntecedent();
  ifRollVeryHighAndSpeedRHigh->joinWithAND(RollVeryHigh, SpeedRHigh);
  FuzzyRule *rule24 = new FuzzyRule(24, ifRollVeryHighAndSpeedRHigh, thenRPWMVeryLow);
  fuzzy->addFuzzyRule(rule24);

  FuzzyRuleAntecedent *ifRollVeryHighAndSpeedRVeryHigh = new FuzzyRuleAntecedent();
  ifRollVeryHighAndSpeedRVeryHigh->joinWithAND(RollVeryHigh, SpeedRVeryHigh);
  FuzzyRule *rule25 = new FuzzyRule(25, ifRollVeryHighAndSpeedRVeryHigh, thenRPWMVeryLow);
  fuzzy->addFuzzyRule(rule25);
}

void userInputFuzzyRoll() {
  // while (!Serial.available())
  //   ;
  // StartFuzzyRoll = Serial.parseInt();
  // if (StartFuzzyRoll) {
  //   while (!Serial.available())
  //     ;
  //   multiplyOutputFuzzyRoll = Serial.parseInt();
  //   Serial.println("Starting Fuzzy Roll");
  // } else if (StartFuzzyRoll == 0) {
  //   Serial.println("Stoping Fuzzy Roll");
  //   resetFuzzyRoll();
  // }


}

void runFuzzyRoll() {
  if (channel_thr_pwm > 1100){
    StartFuzzyRoll = 1;
  }
  else{
    StartFuzzyRoll = 0;
  }
  eAltitude = Altitude - refAltitude;
  dEAltitude = Altitude - prev_Altitude;
  fuzzy->setInput(1, eAltitude);
  fuzzy->setInput(2, dEAltitude);
  
  fuzzy->fuzzify();
  prev_Altitude = Altitude;
  if (StartFuzzyRoll == 1) {
    multiplyOutputFuzzyRoll = 1;
    Throttle_Altitude += (fuzzy->defuzzify(1)) * multiplyOutputFuzzyRoll;
  }
  else{
    resetFuzzyRoll();
  }

  Serial.print("Fuzzy| ");
  // Serial.print("KalmanR");
  // Serial.print(KalmanAngleRoll);
  // Serial.print(" | ");
  // Serial.print("KalmanP");
  // Serial.print(KalmanAnglePitch);
  // Serial.print(" | ");
  Serial.print("rA: ");
  Serial.print(refAltitude);
  Serial.print(" | ");
  Serial.print("A: ");
  Serial.print(Altitude);
  Serial.print(" | ");
  Serial.print("eA: ");
  Serial.print(eAltitude);
  Serial.print(" | ");
  Serial.print("dE: ");
  Serial.print(dEAltitude);
  Serial.print(" | ");
  Serial.print("Thr_Alt: ");
  Serial.println(Throttle_Altitude);

  }





  // Serial.print("IMU DATA | ");
  // Serial.print("Gx:");
  // Serial.print(RateRoll);
  // Serial.print(" | ");
  // Serial.print("Gy:");
  // Serial.print(RatePitch);
  // Serial.print(" | ");
  // Serial.print("Gz:");
  // Serial.print(RateYaw);
  // Serial.print(" | ");
  // Serial.print("R:");
  // Serial.print(Roll);
  // Serial.print(" | ");
  // Serial.print("P:");
  // Serial.print(Pitch);
  // Serial.print(" | ");
  // Serial.print("Y:");
  // Serial.println(Yaw);

  // Serial.print("Fuzzy Roll | ");
  // Serial.print("Error Roll : ");
  // Serial.print(Roll - refRoll);
  // Serial.print(" | ");
  // Serial.print("dError Roll:");
  // Serial.print(Roll - prev_Roll);
  // Serial.print(" | ");
  // Serial.print("Multiplication");
  // Serial.print(multiplyOutputFuzzyRoll);
  // Serial.print(" | ");

  // Serial.print("MotorFuzzy | ");
  // Serial.print("M1P:");
  // Serial.print(MotorInput1);
  // Serial.print(" | ");
  // Serial.print("M2P:");
  // Serial.print(MotorInput2);
  // Serial.print(" | ");
  // Serial.print("M3P:");
  // Serial.print(MotorInput3);
  // Serial.print(" | ");
  // Serial.print("M4P:");
  // Serial.print(MotorInput4);
  // Serial.print(" | ");
  // Serial.print("M5P:");
  // Serial.print(MotorInput5);
  // Serial.print(" | ");
  // Serial.print("M6P:");
  // Serial.print(MotorInput6);
  // Serial.println(" | ");
// }

void resetFuzzyRoll() {
  Throttle_Altitude = MIN_PULSE_LENGTH;
  prev_Altitude = 0;
  // MotorInput1 = InputThrottle;
  // MotorInput2 = InputThrottle;
  // MotorInput3 = InputThrottle;
  // MotorInput4 = InputThrottle;
  // MotorInput5 = InputThrottle;
  // MotorInput6 = InputThrottle;
}
#endif

// 7. RTC BuiltIn
uint8_t currentHour, currentMinute, currentSecond;  // Global Variable for ENABLE_SD_CARD
#ifdef ENABLE_RTC
#include <TimeLib.h>


void initRtc() {
  if (Teensy3Clock.get()) {
    setSyncProvider(Teensy3Clock.get);  // Sinkronkan waktu dengan RTC
    if (timeStatus() == timeSet) {
      Serial.println("RTC time successfully synced!");
    } else {
      Serial.println("RTC time is invalid.");
    }
  } else {
    Serial.println("RTC not initialized, setting default time...");
    setTime(10, 15, 30, 16, 11, 2024);  // Atur waktu default jika RTC belum diatur
    Teensy3Clock.set(now());            // Sinkronkan RTC dengan waktu default
    Serial.println("Default time set to RTC.");
  }
  // Tampilkan waktu saat ini
}

void rtc() {
  currentHour = hour();
  currentMinute = minute();
  currentSecond = second();

#ifdef ENABLE_RTC_PRINT
  Serial.print("RTC| ");
  Serial.print(currentHour);
  Serial.print(":");
  Serial.print(currentMinute);
  Serial.print(":");
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
char buffer[BUFFER_SIZE];                  // Temporary buffer for storing data
size_t bufferIndex = 0;                    // Current position in the buffer

SdFs sd;
FsFile file;

void initSd() {
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
                     currentHour, currentMinute, currentSecond, timestamp,
                     RateRoll, RatePitch, RateYaw, Roll, Pitch, Yaw, Altitude);

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

#define inputRPM_1 24

FASTRUN void pulseHandler_1() {
  unsigned long currentTime_1 = micros();             // Current time in microseconds
  pulseInterval_1 = currentTime_1 - lastPulseTime_1;  // Time between pulses
  lastPulseTime_1 = currentTime_1;
}

void initRPM_1() {
  pinMode(inputRPM_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(inputRPM_1), pulseHandler_1, RISING);  // Detect rising edges
  rpmTimer_1.begin(calculateRPM_1, 10000);                                     // Call calculateRPM every 100ms
}

void calculateRPM_1() {
  if (pulseInterval_1 > 0 && ((unsigned long)micros()) - lastPulseTime_1 <= 20000) {
    float frequency_1 = 1000000.0 / pulseInterval_1;  // Convert µs interval to frequency (Hz)
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

void printRpm_1() {
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
  unsigned long currentTime_2 = micros();             // Current time in microseconds
  pulseInterval_2 = currentTime_2 - lastPulseTime_2;  // Time between pulses
  lastPulseTime_2 = currentTime_2;
}

void initRPM_2() {
  pinMode(inputRPM_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(inputRPM_2), pulseHandler_2, RISING);  // Detect rising edges
  rpmTimer_2.begin(calculateRPM_2, 10000);                                     // Call calculateRPM every 100ms
}

void calculateRPM_2() {
  if (pulseInterval_2 > 0 && ((unsigned long)micros()) - lastPulseTime_2 <= 20000) {
    float frequency_2 = 1000000.0 / pulseInterval_2;  // Convert µs interval to frequency (Hz)
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

void printRpm_2() {
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
  unsigned long currentTime_3 = micros();             // Current time in microseconds
  pulseInterval_3 = currentTime_3 - lastPulseTime_3;  // Time between pulses
  lastPulseTime_3 = currentTime_3;
}

void initRPM_3() {
  pinMode(inputRPM_3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(inputRPM_3), pulseHandler_3, RISING);  // Detect rising edges
  rpmTimer_3.begin(calculateRPM_3, 10000);                                     // Call calculateRPM every 100ms
}

void calculateRPM_3() {
  if (pulseInterval_3 > 0 && ((unsigned long)micros()) - lastPulseTime_3 <= 20000) {
    float frequency_3 = 1000000.0 / pulseInterval_3;  // Convert µs interval to frequency (Hz)
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

void printRpm_3() {
  Serial.print("RPM 3 : ");
  Serial.print(rpm_3);
  Serial.print(" | Interval 3 : ");
  Serial.println(pulseInterval_3);
}
#endif

// 8.4. rpm Sensor 4

float rpm_4 = 0;
volatile unsigned long pulseInterval_4 = 0;
float dt = 0.01;
#ifdef ENABLE_RPM_4
#include <IntervalTimer.h>
IntervalTimer rpmTimer_4;
volatile unsigned long lastPulseTime_4 = 0;

#define inputRPM_4 9

FASTRUN void pulseHandler_4() {
  unsigned long currentTime_4 = micros();             // Current time in microseconds
  pulseInterval_4 = currentTime_4 - lastPulseTime_4;  // Time between pulses
  lastPulseTime_4 = currentTime_4;
}

void initRPM_4() {
  pinMode(inputRPM_4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(inputRPM_4), pulseHandler_4, RISING);  // Detect rising edges
  rpmTimer_4.begin(calculateRPM_4, 10000);                                     // Call calculateRPM every 100ms
}

void calculateRPM_4() {
  if (pulseInterval_4 > 0 && ((unsigned long)micros()) - lastPulseTime_4 <= 20000) {
    float frequency_4 = 1000000.0 / pulseInterval_4;  // Convert µs interval to frequency (Hz)
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

void printRpm_4() {
  Serial.print("RPM 4 : ");
  Serial.print(rpm_4);
  Serial.print(" | Interval 4 : ");
  Serial.println(pulseInterval_4);
}
#endif

#ifdef ENABLE_PID_ANGLE_RATE

float dRefRoll = 30;  // deg/s
float dRefPitch = 30;  // deg/s
float dRefYaw = 30;  // deg/s
float dRefAltitude = 50; // cm /s

float PitchMin = -2;
float PitchMax = 2;
float RollMin = -3;
float RollMax = 3;

// float dt = 0.01;     // 10ms

// volatile float RatePitch, RateRoll, RateYaw;
// float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw,AccXCalibration,AccYCalibration,AccZCalibration;

float PAngleRoll = 2;
float IAngleRoll = 0;
float DAngleRoll = 0;
// float PAngleRoll = 2.4;    // Hasil ZN
// float IAngleRoll = 3.87;
// float DAngleRoll = 0.37;
float PRateRoll = 2;  //2
float IRateRoll = 5.14;   //5.140
float DRateRoll = 0.085;  //0.085
// float PAngleRoll = 0;
// float IAngleRoll = 0;
// float DAngleRoll = 0;
// float PRateRoll = 0;  //0.625
// float IRateRoll = 0;   //0.01
// float DRateRoll = 0;  //0.0088

float PAnglePitch = 2;
float IAnglePitch = 0;
float DAnglePitch = 0;
// float PRatePitch = 3.6;
// float IRatePitch = 8.5;
// float DRatePitch = 0.1;
float PRatePitch = 2;
float IRatePitch = 5.140;
float DRatePitch = 0.085;
// float PAnglePitch = 0;
// float IAnglePitch = 0;
// float DAnglePitch = 0;
// float PRatePitch = 0;
// float IRatePitch = 0;
// float DRatePitch = 0;


// float PAngleYaw = 2;
// float IAngleYaw = 0;
// float DAngleYaw = 0;
// float PRateYaw = 2;
// float IRateYaw = 3;
// float DRateYaw = 0.1;
float PAngleYaw = 0;
float IAngleYaw = 0;
float DAngleYaw = 0;
float PRateYaw = 0;
float IRateYaw = 0;
float DRateYaw = 0;

float PVerticalVelocity = 0;
float IVerticalVelocity = 0;
float DVerticalVelocity = 0;

// float PAltitude = 3;
// float IAltitude = 9;
// float DAltitude = 0.3;
// float PAltitude = 3.03;    // Tuning ZN
// float IAltitude = 1.74;
// float DAltitude = 1.32;
// float PAltitude = 0;
// float IAltitude = 0;
// float DAltitude = 0;

float PPosX = 2;
float IPosX = 0;
float DPosX = 0;

float PPosY = 2;
float IPosY = 0;
float DPosY = 0;


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

volatile float PtermVerticalVelocity;
volatile float ItermVerticalVelocity;
volatile float DtermVerticalVelocity;
volatile float PIDOutputVerticalVelocity;

volatile float PtermAltitude;
volatile float ItermAltitude;
volatile float DtermAltitude;
volatile float PIDOutputAltitude;

volatile float PtermPosX;
volatile float ItermPosX;
volatile float DtermPosX;
volatile float PIDOutputPosX;

volatile float PtermPosY;
volatile float ItermPosY;
volatile float DtermPosY;
volatile float PIDOutputPosY;

volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw, ErrorVerticalVelocity, ErrorAltitude, ErrorPosX, ErrorPosY;
volatile float DErrorAltitude, DErrorPosX, DErrorPosY;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw, InputVerticalVelocity, InputAltitude;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw, PrevErrorVerticalVelocity, PrevErrorAltitude, PrevErrorPosX, PrevErrorPosY;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw, PrevItermVerticalVelocity, PrevItermAltitude, PrevItermPosX, PrevItermPosY;
volatile float PIDReturn[] = { 0, 0, 0 };

float setPointRoll = 0, setPointPitch = 0, setPointYaw = 0;
float setPointAltitude  = 0; 
float DesiredAngleRoll = setPointRoll, DesiredAnglePitch = setPointPitch, DesiredAngleYaw = setPointYaw;
// float DesiredAltitude = setPointAltitude;
volatile float ErrorAngleRoll, ErrorAnglePitch, ErrorAngleYaw;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch, PrevErrorAngleYaw;
volatile float PrevItermAngleRoll, PrevItermAnglePitch, PrevItermAngleYaw;

void setPointAdjust(float& setPoint, float ref, float rateOfChange, float timeStep) {
    if (setPoint < ref) {
        // Increase setpoint towards reference
        setPoint += rateOfChange * timeStep;

        // Ensure setPoint does not exceed ref
        if (setPoint > ref) {
            setPoint = ref;  // Cap at the reference
        }
    } else if (setPoint > ref) {
        // Decrease setpoint towards reference
        setPoint -= rateOfChange * timeStep;

        // Ensure setPoint does not drop below ref
        if (setPoint < ref) {
            setPoint = ref;  // Cap at the reference
        }
    }
}

float mapValue(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// float AccX, AccY, AccZ;
// float AngleRoll, AnglePitch;
// float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
// float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
// float Kalman1DOutput[] = { 0, 0 };
// void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
//   KalmanState = KalmanState + dt * KalmanInput;
//   KalmanUncertainty = KalmanUncertainty + dt * dt * 4 * 4;
//   float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
//   KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
//   KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
//   Kalman1DOutput[0] = KalmanState;
//   Kalman1DOutput[1] = KalmanUncertainty;
// }

// Vertical Velocity
// float AltitudeBarometer, AltitudeBarometerStartUp;
// float AccZInertial;
// #include <BasicLinearAlgebra.h>
// using namespace BLA;
// float AltitudeKalman, VelocityVerticalKalman;
// BLA::Matrix<2,2> F; BLA::Matrix<2,1> G;
// BLA::Matrix<2,2> P; BLA::Matrix<2,2> Q;
// BLA::Matrix<2,1> S; BLA::Matrix<1,2> H;
// BLA::Matrix<2,2> I; BLA::Matrix<1,1> Acc;
// BLA::Matrix<2,1> K; BLA::Matrix<1,1> R;
// BLA::Matrix<1,1> L; BLA::Matrix<1,1> M;
// void kalman_2d(void){
//   Acc = {AccZ};
//   S=F*S+G*Acc;
//   P=F*P*~F+Q;
//   L=H*P*~H+R;
//   K=P*~H*Inverse(L);
//   M = {AltitudeBarometer};
//   S=S+K*(M-H*S);
//   AltitudeKalman=S(0,0); 
//   VelocityVerticalKalman=S(1,0); 
//   P=(I-K*H)*P;
// }


void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + (I * (Error + PrevError) * (dt / 2));
  if (Iterm > 400) {
    Iterm = 400;
  } else if (Iterm < -400) {
    Iterm = -400;
  }
  float Dterm = D * ((Error - PrevError) / dt);
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400) {
    PIDOutput = 400;
  } else if (PIDOutput < -400) {
    PIDOutput = -400;
  }
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void resetPidAngleRate() {
  PrevErrorRateRoll = 0;
  PrevErrorRatePitch = 0;
  PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0;
  PrevItermRatePitch = 0;
  PrevItermRateYaw = 0;
  PrevErrorAngleRoll = 0;
  PrevErrorAnglePitch = 0;
  PrevItermAngleRoll = 0;
  PrevItermAnglePitch = 0;
  PrevErrorAltitude = 0;
  PrevItermAltitude = 0;
  PrevErrorPosX = 0;
  PrevItermPosX = 0;
  PrevErrorPosY = 0;
  PrevItermPosY = 0;
  // xInit = xNow;
  // yInit = yNow;


}

void loopPidAngleRate100Hz() {
  
  // setPointYaw = (channel_yaw_pwm - 1504.0) / 500.0 * (-60);         // when using PID ANGLE
  // DesiredRateYaw = (channel_yaw_pwm - 1522.0) / 500.0 * (-30.0);   // WHEN USING RATE ONLY
  setPointYaw = 0;
  #ifdef ENABLE_PID_ANGLE_RATE_POSITION
  // Target Displacement from initial position 
  dX = (channel_pitch_pwm - 1503.0) / 500.0 * (3);
  dY = (channel_roll_pwm - 1522.0) / 500.0 * (3);

  if (channel_thr_pwm <1100){
    xInit = -1 * x_2;
    yInit = -1 * y_2;
  }
  xNow = -1 * x_2;
  yNow = -1 * y_2;
  xTarget = xInit + dX;
  yTarget = yInit + dY;

  #else
  // DesiredRateRoll = (channel_roll_pwm - 1522.0) / 500.0 * (-30.0);   // WHEN USING RATE ONLY
  // setPointRoll = (channel_roll_pwm - 1522.0) / 500.0 * (30.0);         // when using PID ANGLE
  // setPointRoll = 0;
  
  // DesiredRatePitch = (channel_pitch_pwm - 1522.0) / 500.0 * (-30.0);   // WHEN USING RATE ONLY
  // setPointPitch = (channel_pitch_pwm - 1503.0) / 500.0 * (30.0); 
  // setPointPitch = 0;        // when using PID ANGLE
  #endif

  #ifdef ENABLE_PID_ANGLE_RATE_ALTITUDE
  // setPointAltitude = 50 + (channel_thr_pwm - 1000) / 1000 * 100; // Range ref Alt 110 - 200 cm
  // setPointAltitude = 70;
  #endif

    // setPointAlt - 
  // 
  
    // Pitch = 1

  // kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, Roll);
  // KalmanAngleRoll = Kalman1DOutput[0];
  // KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  // kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, Pitch);
  // KalmanAnglePitch = Kalman1DOutput[0];
  // KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Ramp Signal for reference
  // setPointAdjust(DesiredAngleRoll, setPointRoll, dRefRoll, dt);
  // setPointAdjust(DesiredAnglePitch, setPointPitch, dRefPitch, dt);
  // setPointAdjust(DesiredAngleYaw, setPointYaw, dRefYaw, dt);
  // setPointAdjust(DesiredAltitude, setPointAltitude, dRefAltitude, dt);
  
  DesiredAngleRoll = setPointRoll;      // IF  USING RECEIVER
  DesiredAnglePitch = setPointPitch;
  // DesiredAngleYaw = setPointYaw;  // WHEN USING PID ANGLE
  DesiredAltitude = setPointAltitude;
  

  // Roll Angle PID
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

  // PID Angle Pitch
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

  // Yaw Angle PID
  ErrorAngleYaw = DesiredAngleYaw - Yaw;
  ErrorAngleYaw = fmod(ErrorAngleYaw + 180, 360);
  if (ErrorAngleYaw < 0){
    ErrorAngleYaw = ErrorAngleYaw + 360; 
  }
  ErrorAngleYaw = ErrorAngleYaw - 180;
  
  PtermYaw = PAngleYaw * ErrorAngleYaw;
  ItermYaw = PrevItermAngleYaw + (IAngleYaw * (ErrorAngleYaw + PrevErrorAngleYaw) * (dt / 2));
  ItermYaw = (ItermYaw > 400) ? 400 : ((ItermYaw < -400) ? -400 : ItermYaw);
  DtermYaw = DAngleYaw * ((ErrorAngleYaw - PrevErrorAngleYaw) / dt);
  PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
  PIDOutputYaw = (PIDOutputYaw > 400) ? 400 : ((PIDOutputYaw < -400) ? -400 : PIDOutputYaw);
  DesiredRateYaw = PIDOutputYaw;
  PrevErrorAngleYaw = ErrorAngleYaw;
  PrevItermAngleYaw = ItermYaw;

  // Compute errors
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  // Rate Roll PID
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

  // Pitch Rate PID
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

  // Yaw Rate PID
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

  
}

void loopPidAngleRate20Hz(){
  // DesiredVerticalVelocity =  (channel_thr_pwm - 1300) / 300 * (10);   // WHEN USING RATE ONLY
  // DesiredVerticalVelocity = setPointVertical;
  
  
  #ifdef ENABLE_PID_ANGLE_RATE_ALTITUDE
  if (channel_thr_pwm < 1100){
    // setPointAltitude = Altitude;
    // DesiredAltitude = Altitude;
    setPointAltitude = 5;
  }
  else{
    // setPointAltitude = (channel_thr_pwm - 1300) / 1000 * 200;    // Range 0-200 cm  
    setPointAltitude = 100;
  }
  

  // Altitude PID
  ErrorAltitude = DesiredAltitude - Altitude;
  DErrorAltitude = ErrorAltitude - PrevErrorAltitude;

  PtermAltitude = PAltitude * ErrorAltitude;
  ItermAltitude = PrevItermAltitude + (IAltitude * (ErrorAltitude + PrevErrorAltitude) * (dt / 2));
  ItermAltitude = (ItermAltitude > 400) ? 400 : ((ItermAltitude < -400) ? -400 : ItermAltitude);
  DtermAltitude = DAltitude * ((DErrorAltitude) / dt);
  PIDOutputAltitude = PtermAltitude + ItermAltitude + DtermAltitude;
  PIDOutputAltitude = (PIDOutputAltitude > 600) ? 600 : ((PIDOutputAltitude < -600) ? -600 : PIDOutputAltitude);
  // DesiredRateRoll = PIDOutputRoll;
  PrevErrorAltitude = ErrorAltitude;
  PrevItermAltitude = ItermAltitude;

  InputThrottle = 1000 + PIDOutputAltitude;
  #else
  // InputThrottle = channel_thr_pwm;
  #endif

  

  // // Vertical Velocity PID
  // ErrorVerticalVelocity = DesiredVerticalVelocity - verticalVelocity;
  // PtermVerticalVelocity = PVerticalVelocity * ErrorVerticalVelocity;
  // ItermVerticalVelocity = PrevItermVerticalVelocity + (IVerticalVelocity * (ErrorVerticalVelocity + PrevErrorVerticalVelocity) * (dt / 2));
  // ItermVerticalVelocity = (ItermVerticalVelocity > 400) ? 400 : ((ItermVerticalVelocity < -400) ? -400 : ItermVerticalVelocity);
  // DtermVerticalVelocity = DVerticalVelocity * ((ErrorVerticalVelocity - PrevErrorVerticalVelocity) / dt);
  // PIDOutputVerticalVelocity = PtermVerticalVelocity + ItermVerticalVelocity + DtermVerticalVelocity;
  // PIDOutputVerticalVelocity = (PIDOutputVerticalVelocity > 400) ? 400 : ((PIDOutputVerticalVelocity < -400) ? -400 : PIDOutputVerticalVelocity);

  // InputThrottle = 1300 + PIDOutputVerticalVelocity;

  // PrevErrorVerticalVelocity = ErrorVerticalVelocity;
  // PrevItermVerticalVelocity = ItermVerticalVelocity;

}

void loopPidAngleRate5Hz(){
  #ifdef ENABLE_PID_ANGLE_RATE_POSITION
  if (GPSFix == 1){
    xTarget = xInit + dX;
    yTarget = yInit + dY;
  }

  // Position X PID
  ErrorPosX = 100 * (xTarget - xNow);
  DErrorPosX = ErrorPosX - PrevErrorPosX;

  PtermPosX = PPosX * ErrorPosX;
  ItermPosX = PrevItermPosX + (IPosX * (ErrorPosX + PrevErrorPosX) * (dt / 2));
  ItermPosX = (ItermPosX > 400) ? 400 : ((ItermPosX < -400) ? -400 : ItermPosX);
  DtermPosX = DPosX * ((DErrorPosX) / dt);
  PIDOutputPosX = PtermPosX; /*+ ItermPosX + DtermPosX;*/
  PIDOutputPosX = (PIDOutputPosX > 400) ? 400 : ((PIDOutputPosX < -400) ? -400 : PIDOutputPosX);
  // Ouput PID PosX
  if (GPSFix == 1){
  // setPointPitch = mapValue(PIDOutputPosX, -400, 400, PitchMin,PitchMax);  // -2,2
  }
  else{
    setPointPitch = 0;
  }
  // map(value, fromLow, fromHigh, toLow, toHigh)
  PrevErrorPosX = ErrorPosX;
  PrevItermPosX = ItermPosX;

  // Position Y PID 
  ErrorPosY = 200 * (yTarget - yNow);
  DErrorPosY = ErrorPosY - PrevErrorPosY;

  PtermPosY = PPosY * ErrorPosY;
  ItermPosY = PrevItermPosY + (IPosY * (ErrorPosY + PrevErrorPosY) * (dt / 2));
  ItermPosY = (ItermPosY > 400) ? 400 : ((ItermPosY < -400) ? -400 : ItermPosY);
  DtermPosY = DPosY * ((DErrorPosY) / dt);
  PIDOutputPosY = PtermPosY; /*+ ItermPosY + DtermPosY;*/
  PIDOutputPosY = (PIDOutputPosY > 400) ? 400 : ((PIDOutputPosY < -400) ? -400 : PIDOutputPosY);
  if (GPSFix == 1){
    setPointRoll = -1 * mapValue(PIDOutputPosY, -400, 400, RollMin,RollMax);   // -2,2
  }
  else{
    setPointRoll = 0;
  }
  PrevErrorPosY = ErrorPosY;
  PrevItermPosY = ItermPosY;
  #endif
}

void pidAngleRateLimit(){
  // channel_thr_pwm = 1111;
  if (channel_thr_pwm < 1100){
    InputThrottle = 1000;
    resetPidAngleRate();
  }
  if (InputThrottle < 1100){
    resetPidAngleRate();
  }


  if (InputThrottle > 1800) {
    InputThrottle = 1800;
  }

#ifdef ENABLE_MOTOR_HEXACOPTER
  MotorInput1 = (InputThrottle - InputRoll);        //
  MotorInput2 = (InputThrottle + InputRoll);        //
  MotorInput3 = (InputThrottle + 0.5 * InputRoll);  //
  MotorInput4 = (InputThrottle - 0.5 * InputRoll);  //
  MotorInput5 = (InputThrottle - 0.5 * InputRoll);  //
  MotorInput6 = (InputThrottle + 0.5 * InputRoll);  //
#else
  // Normal Testing
  MotorInput1 = (InputThrottle - InputPitch + InputYaw);  // front - cw
  MotorInput2 = (InputThrottle - InputRoll - InputYaw);   // right - ccw
  MotorInput3 = (InputThrottle + InputPitch + InputYaw);  // rare  - cw
  MotorInput4 = (InputThrottle + InputRoll - InputYaw);   // left  - ccw

  // Roll Testing
  // MotorInput1 = 1000;  // front - cw
  // MotorInput2 = (InputThrottle - InputRoll);   // right - ccw
  // MotorInput3 = 1000;  // rare  - cw
  // MotorInput4 = (InputThrottle + InputRoll);   // left  - ccw

  // Pitch Testing
  // MotorInput1 = (InputThrottle - InputPitch);  // front - cw
  // MotorInput2 = 1000;   // right - ccw
  // MotorInput3 = (InputThrottle + InputPitch);  // rare  - cw
  // MotorInput4 = 1000;   // left  - ccw

  // Yaw Testing
  // MotorInput1 = (InputThrottle + InputYaw);  // front - cw
  // MotorInput2 = (InputThrottle - InputYaw);   // right - ccw
  // MotorInput3 = (InputThrottle + InputYaw);  // rare  - cw
  // MotorInput4 = (InputThrottle - InputYaw);   // left  - ccw

#endif

  if (MotorInput1 > 2000) {
    MotorInput1 = 1999;
  }

  if (MotorInput2 > 2000) {
    MotorInput2 = 1999;
  }

  if (MotorInput3 > 2000) {
    MotorInput3 = 1999;
  }

  if (MotorInput4 > 2000) {
    MotorInput4 = 1999;
  }

  if (MotorInput5 > 2000) {
    MotorInput5 = 1999;
  }

  if (MotorInput6 > 2000) {
    MotorInput6 = 1999;
  }

  int ThrottleIdle = 1100;
  int ThrottleCutOff = 1000;
  if (MotorInput1 < ThrottleIdle) {
    MotorInput1 = ThrottleIdle;
  }
  if (MotorInput2 < ThrottleIdle) {
    MotorInput2 = ThrottleIdle;
  }
  if (MotorInput3 < ThrottleIdle) {
    MotorInput3 = ThrottleIdle;
  }
  if (MotorInput4 < ThrottleIdle) {
    MotorInput4 = ThrottleIdle;
  }
  if (MotorInput5 < ThrottleIdle) {
    MotorInput5 = ThrottleIdle;
  }
  if (MotorInput6 < ThrottleIdle) {
    MotorInput6 = ThrottleIdle;
  }

  if (InputThrottle < 1100) {
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;
    MotorInput5 = ThrottleCutOff;
    MotorInput6 = ThrottleCutOff;
    // resetPidAngleRate();
  }

  #ifdef ENABLE_PID_ANGLE_RATE_PRINT

  // PID Altitude
  // Serial.print("PIDAlt| ");
  // Serial.print("rA: ");
  // Serial.print(DesiredAltitude);
  // Serial.print(" | ");
  // Serial.print("A: ");
  // Serial.print(Altitude);
  // Serial.print(" | ");
  // Serial.print("eA: ");
  // Serial.print(ErrorAltitude);
  // Serial.print(" | ");
  // Serial.print("dE: ");
  // Serial.print(DErrorAltitude);
  // Serial.print(" | ");
  // Serial.print("prevEAlt: ");
  // Serial.print(PrevErrorAltitude);
  // Serial.print(" | ");
  // Serial.print("prevITermAlt: ");
  // Serial.print(PrevItermAltitude);
  // Serial.print(" | ");
  // Serial.print("iT: ");
  // Serial.println(InputThrottle);
  
  Serial.print("PIDRef| ");
  // Serial.print("KalmanR");
  // Serial.print(KalmanAngleRoll);
  // Serial.print(" | ");
  // Serial.print("KalmanP");
  // Serial.print(KalmanAnglePitch);
  // Serial.print(" | ");
  Serial.print("rR");
  Serial.print(DesiredAngleRoll);
  Serial.print(" | ");
  Serial.print("rP");
  Serial.print(DesiredAnglePitch);
  Serial.print(" | ");
  Serial.print("rY");
  Serial.print(DesiredAngleYaw);
  Serial.print(" | ");
  Serial.print("rAlt:");
  Serial.println(DesiredAltitude);
  // Serial.print(" | ");
  // Serial.print("rGx");
  // Serial.print(DesiredRateRoll);
  // Serial.print(" | ");
  // Serial.print("rGy");
  // Serial.print(DesiredRatePitch);
  // Serial.print(" | ");
  // Serial.print("rGz:");
  // Serial.print(DesiredRateYaw);
  // Serial.print(" | ");
  // Serial.print("Thr_Alt:");
  // Serial.println(Throttle_Altitude);
  // Serial.print("PIDOut| ");
  // Serial.print("PTR:");
  // Serial.print(PtermRoll, 3);
  // Serial.print(" | ");
  // Serial.print("ITR:");
  // Serial.print(ItermRoll, 3);
  // Serial.print(" | ");
  // Serial.print("DTR:");
  // Serial.print(DtermRoll, 3);
  // Serial.print("           |       ");
  // Serial.print("PTP:");
  // Serial.print(PtermPitch, 3);
  // Serial.print(" | ");
  // Serial.print("ITP:");
  // Serial.print(ItermPitch, 3);
  // Serial.print(" | ");
  // Serial.print("DTP:");
  // Serial.print(DtermPitch, 3);
  // Serial.print("           |       ");
  // Serial.print("Error Rate Roll");
  // Serial.println(ErrorRateRoll);

  // PID Gain Rate Roll
  // Serial.print("PIDGain| ");
  // Serial.print("PAnglePitch: ");
  // Serial.print(PAnglePitch, 3);
  // Serial.print(" | ");
  // Serial.print("IAnglePitch: ");
  // Serial.print(IAnglePitch, 3);
  // Serial.print(" | ");
  // Serial.print("DAnglePitch: ");
  // Serial.print(DAnglePitch, 3);
  // Serial.print(" | ");
  // Serial.print("PRatePitch: ");
  // Serial.print(PRatePitch, 3);
  // Serial.print(" | ");
  // Serial.print("IRatePitch: ");
  // Serial.print(IRatePitch, 3);
  // Serial.print(" | ");
  // Serial.print("DRatePitch: ");
  // Serial.print(DRatePitch, 3);
  // Serial.print("  | ");
  // Serial.print("ErrorRatePitch: ");
  // Serial.println(ErrorRatePitch, 3);

  // // PID Gain Rate Roll
  // Serial.print("PIDGain| ");
  // Serial.print("PAngleRoll: ");
  // Serial.print(PAngleRoll, 3);
  // Serial.print(" | ");
  // Serial.print("IAngleRoll: ");
  // Serial.print(IAngleRoll, 3);
  // Serial.print(" | ");
  // Serial.print("DAngleRoll: ");
  // Serial.print(DAngleRoll, 3);
  // Serial.print(" | ");
  // Serial.print("PRateRoll: ");
  // Serial.print(PRateRoll, 3);
  // Serial.print(" | ");
  // Serial.print("IRateRoll: ");
  // Serial.print(IRateRoll, 3);
  // Serial.print(" | ");
  // Serial.print("DRateRoll: ");
  // Serial.print(DRateRoll, 3);
  // Serial.print("  | ");
  // Serial.print("ErrorRateRoll: ");
  // Serial.println(ErrorRateRoll, 3);
  
  // // PID Gain Rate Yaw
  // Serial.print("PIDGain| ");
  // Serial.print("PAngleYaw: ");
  // Serial.print(PAngleYaw, 3);
  // Serial.print(" | ");
  // Serial.print("IAngleYaw: ");
  // Serial.print(IAngleYaw, 3);
  // Serial.print(" | ");
  // Serial.print("DAngleYaw: ");
  // Serial.print(DAngleYaw, 3);
  // Serial.print(" | ");
  // Serial.print("PRateYaw: ");
  // Serial.print(PRateYaw, 3);
  // Serial.print(" | ");
  // Serial.print("IRateYaw: ");
  // Serial.print(IRateYaw, 3);
  // Serial.print(" | ");
  // Serial.print("DRateYaw: ");
  // Serial.print(DRateYaw, 3);
  // Serial.print("  | ");
  // Serial.print("ErrorAngleYaw: ");
  // Serial.print(ErrorAngleYaw, 3);
  // Serial.print("  | ");
  // Serial.print("ErrorRateYaw: ");
  // Serial.println(ErrorRateYaw, 3);

  // // PID Vertical Velocity
  // Serial.print("PIDGainV| ");
  // Serial.print("PVV: ");
  // Serial.print(PVerticalVelocity, 3);
  // Serial.print(" | ");
  // Serial.print("IVV: ");
  // Serial.print(IVerticalVelocity, 3);
  // Serial.print(" | ");
  // Serial.print("DVV: ");
  // Serial.print(DVerticalVelocity, 3);
  // Serial.print(" | ");
  // Serial.print("ErrorVV: ");
  // Serial.println(ErrorVerticalVelocity);

  // // PID Altitude
  // Serial.print("PIDGainAlt| ");
  // Serial.print("PAlt: ");
  // Serial.print(PAltitude, 3);
  // Serial.print(" | ");
  // Serial.print("IAlt: ");
  // Serial.print(IAltitude, 3);
  // Serial.print(" | ");
  // Serial.print("DAlt: ");
  // Serial.print(DAltitude, 3);
  // Serial.print(" | ");
  // Serial.print("EAlt: ");
  // Serial.println(ErrorAltitude);
  
  // GPS PRINT
  // float ErrorX = XTarget - XNow;
  // float ErrorY = YTarget - YNow;
  #ifdef ENABLE_GPS
  Serial.print("GPSNow| ");
  Serial.print(GPSFix);
  Serial.print(" | ");
  Serial.print(" ErrorX: ");
  Serial.print(ErrorPosX);
  Serial.print(" | ");
  Serial.print(" ErrorY: ");
  Serial.println(ErrorPosY);


  // PIDGain PosX
  Serial.print("PIDGainPosX| ");
  Serial.print("PX: ");
  Serial.print(PPosX, 3);
  Serial.print(" | ");
  Serial.print("IX: ");
  Serial.print(IPosX, 3);
  Serial.print(" | ");
  Serial.print("DX: ");
  Serial.print(DPosX, 3);
  Serial.print(" | ");
  Serial.print("EX: ");
  Serial.print(ErrorPosX);
  Serial.print(" | ");
  Serial.print("ITX: ");
  Serial.print(ItermPosX);
  Serial.print(" | ");
  Serial.print("DTX: ");
  Serial.println(DtermPosX);

  // PIDGain PosY
  Serial.print("PIDGainPosY| ");
  Serial.print("PY: ");
  Serial.print(PPosY, 3);
  Serial.print(" | ");
  Serial.print("IY: ");
  Serial.print(IPosY, 3);
  Serial.print(" | ");
  Serial.print("DY: ");
  Serial.print(DPosY, 3);
  Serial.print(" | ");
  Serial.print("EY: ");
  Serial.print(ErrorPosY);
  Serial.print(" | ");
  Serial.print("ITY: ");
  Serial.print(ItermPosY);
  Serial.print(" | ");
  Serial.print("DTY: ");
  Serial.println(DtermPosY);

  // Pos X
  Serial.print("PosX| ");
  Serial.print("xI: ");
  Serial.print(xInit, 1);
  Serial.print(" | ");
  Serial.print("xN: ");
  Serial.print(xNow, 1);
  Serial.print(" | ");
  Serial.print("xT: ");
  Serial.print(xTarget, 1);
  Serial.print(" | ");
  Serial.print("dX: ");
  Serial.print(dX, 1);
  Serial.print(" | ");
  Serial.print("delX: ");
  Serial.print(deltaX, 1);
  Serial.print(" | ");
  Serial.print("SP_P: ");
  Serial.println(setPointPitch, 1);
  
  

  // Pos Y
  Serial.print("PosY| ");
  Serial.print("yI: ");
  Serial.print(yInit, 1);
  Serial.print(" | ");
  Serial.print("yN: ");
  Serial.print(yNow, 1);
  Serial.print(" | ");
  Serial.print("yT: ");
  Serial.print(yTarget, 1);
  Serial.print(" | ");
  Serial.print("dY: ");
  Serial.print(dY, 1);
  Serial.print(" | ");
  Serial.print("delY: ");
  Serial.print(deltaY, 1);
  Serial.print(" | ");
  Serial.print("SP_R: ");
  Serial.println(setPointRoll, 1);
  #endif
  // PID PosY

  Serial.print("PIDInput || ");
  Serial.print("iT:");
  Serial.print(InputThrottle);
  Serial.print(" | ");
  Serial.print("iR:");
  Serial.print(InputRoll);
  Serial.print(" | ");
  Serial.print("iP:");
  Serial.print(InputPitch);
  Serial.print(" | ");
  Serial.print("iY:");
  Serial.println(InputYaw);

  // Serial.print("MotorPID| ");
  // Serial.print("M1P:");
  // Serial.print(MotorInput1);
  // Serial.print(" | ");
  // Serial.print("M2P:");
  // Serial.print(MotorInput2);
  // Serial.print(" | ");
  // Serial.print("M3P:");
  // Serial.print(MotorInput3);
  // Serial.print(" | ");
  // Serial.print("M4P:");
  // Serial.print(MotorInput4);
  // Serial.print(" | ");
  // Serial.print("M5P:");
  // Serial.print(MotorInput5);
  // Serial.print(" | ");
  // Serial.print("M6P:");
  // Serial.println(MotorInput6);
#endif

}
#endif

#ifdef ENABLE_CUSTOM
//Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM;

//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
unsigned long channel_1_fs = 1000;  //thro
unsigned long channel_2_fs = 1500;  //ail
unsigned long channel_3_fs = 1500;  //elev
unsigned long channel_4_fs = 1500;  //rudd
unsigned long channel_5_fs = 2000;  //gear, greater than 1500 = throttle cut
unsigned long channel_6_fs = 2000;  //aux1

//IMU:
// float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX, MagY, MagZ;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;

//Controller parameters (take note of defaults before modifying!):
float i_limit = 50;     //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 30.0;   //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxPitch = 30.0;  //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxYaw = 160.0;   //Max yaw rate in deg/sec


float Kp_roll_angle = 0.610;  //Roll P-gain - angle mode
float Ki_roll_angle = 0.115;  //Roll I-gain - angle mode
float Kd_roll_angle = 0.160;  //Roll D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_roll = 0.9;      //Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.0;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.0;   //Pitch D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_pitch = 0.9;     //Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)

float Kp_roll_rate = 0.15;     //Roll P-gain - rate mode
float Ki_roll_rate = 0.2;      //Roll I-gain - rate mode
float Kd_roll_rate = 0.0002;   //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
float Kp_pitch_rate = 0.15;    //Pitch P-gain - rate mode
float Ki_pitch_rate = 0.2;     //Pitch I-gain - rate mode
float Kd_pitch_rate = 0.0002;  //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

float Kp_yaw = 0.0;  //Yaw P-gain
float Ki_yaw = 0.0;  //Yaw I-gain
float Kd_yaw = 0.0;  //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

//Flight status
bool armedFly = false;

void getDesState() {
  //DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
   * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
   * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
   * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
   * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
   */
  channel_thr_pwm = input_throttle;
  thro_des = (channel_thr_pwm - 1000.0) / 1000.0;    //Between 0 and 1
  roll_des = (channel_roll_pwm - 1500.0) / 500.0;    //Between -1 and 1
  pitch_des = (channel_pitch_pwm - 1500.0) / 500.0;  //Between -1 and 1
  yaw_des = (channel_yaw_pwm - 1500.0) / 500.0;      //Between -1 and 1
  roll_passthru = roll_des / 2.0;                    //Between -0.5 and 0.5
  pitch_passthru = pitch_des / 2.0;                  //Between -0.5 and 0.5
  yaw_passthru = yaw_des / 2.0;                      //Between -0.5 and 0.5

  //Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0);                //Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0) * maxRoll;     //Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0) * maxPitch;  //Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0) * maxYaw;        //Between -maxYaw and +maxYaw
  roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);

  if (refRoll <= 30 && refRoll >= -30) {
    roll_des = refRoll;
  }
  if (refPitch <= 30 && refPitch >= -30) {
    pitch_des = refPitch;
  }
}

void controlANGLE() {
  //DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
   * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */

  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll * dt;
  if (channel_thr_pwm < 1060) {  //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit);  //Saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX;
  roll_PID = 0.01 * (Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll - Kd_roll_angle * derivative_roll);  //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch * dt;
  if (channel_thr_pwm < 1060) {  //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit);  //Saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_PID = .01 * (Kp_pitch_angle * error_pitch + Ki_pitch_angle * integral_pitch - Kd_pitch_angle * derivative_pitch);  //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (channel_thr_pwm < 1060) {  //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit);  //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev) / dt;
  yaw_PID = .01 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw);  //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void controlANGLE2() {
  //DESCRIPTION: Computes control commands based on state error (angle) in cascaded scheme
  /*
   * Gives better performance than controlANGLE() but requires much more tuning. Not reccommended for first-time setup.
   * See the documentation for tuning this controller.
   */
  //Outer loop - PID on angle
  float roll_des_ol, pitch_des_ol;
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll_ol = integral_roll_prev_ol + error_roll * dt;
  if (channel_thr_pwm < 1060) {  //Don't let integrator build if throttle is too low
    integral_roll_ol = 0;
  }
  integral_roll_ol = constrain(integral_roll_ol, -i_limit, i_limit);  //Saturate integrator to prevent unsafe buildup
  derivative_roll = (roll_IMU - roll_IMU_prev) / dt;
  roll_des_ol = Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll_ol;  // - Kd_roll_angle*derivative_roll;

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch_ol = integral_pitch_prev_ol + error_pitch * dt;
  if (channel_thr_pwm < 1060) {  //Don't let integrator build if throttle is too low
    integral_pitch_ol = 0;
  }
  integral_pitch_ol = constrain(integral_pitch_ol, -i_limit, i_limit);  //saturate integrator to prevent unsafe buildup
  derivative_pitch = (pitch_IMU - pitch_IMU_prev) / dt;
  pitch_des_ol = Kp_pitch_angle * error_pitch + Ki_pitch_angle * integral_pitch_ol;  // - Kd_pitch_angle*derivative_pitch;

  //Apply loop gain, constrain, and LP filter for artificial damping
  float Kl = 30.0;
  roll_des_ol = Kl * roll_des_ol;
  pitch_des_ol = Kl * pitch_des_ol;
  roll_des_ol = constrain(roll_des_ol, -240.0, 240.0);
  pitch_des_ol = constrain(pitch_des_ol, -240.0, 240.0);
  roll_des_ol = (1.0 - B_loop_roll) * roll_des_prev + B_loop_roll * roll_des_ol;
  pitch_des_ol = (1.0 - B_loop_pitch) * pitch_des_prev + B_loop_pitch * pitch_des_ol;

  //Inner loop - PID on rate
  //Roll
  error_roll = roll_des_ol - GyroX;
  integral_roll_il = integral_roll_prev_il + error_roll * dt;
  if (channel_thr_pwm < 1060) {  //Don't let integrator build if throttle is too low
    integral_roll_il = 0;
  }
  integral_roll_il = constrain(integral_roll_il, -i_limit, i_limit);  //Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev) / dt;
  roll_PID = .01 * (Kp_roll_rate * error_roll + Ki_roll_rate * integral_roll_il + Kd_roll_rate * derivative_roll);  //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des_ol - GyroY;
  integral_pitch_il = integral_pitch_prev_il + error_pitch * dt;
  if (channel_thr_pwm < 1060) {  //Don't let integrator build if throttle is too low
    integral_pitch_il = 0;
  }
  integral_pitch_il = constrain(integral_pitch_il, -i_limit, i_limit);  //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev) / dt;
  pitch_PID = .01 * (Kp_pitch_rate * error_pitch + Ki_pitch_rate * integral_pitch_il + Kd_pitch_rate * derivative_pitch);  //Scaled by .01 to bring within -1 to 1 range

  //Yaw
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (channel_thr_pwm < 1060) {  //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit);  //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev) / dt;
  yaw_PID = .01 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw);  //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev_ol = integral_roll_ol;
  integral_roll_prev_il = integral_roll_il;
  error_roll_prev = error_roll;
  roll_IMU_prev = roll_IMU;
  roll_des_prev = roll_des_ol;
  //Update pitch variables
  integral_pitch_prev_ol = integral_pitch_ol;
  integral_pitch_prev_il = integral_pitch_il;
  error_pitch_prev = error_pitch;
  pitch_IMU_prev = pitch_IMU;
  pitch_des_prev = pitch_des_ol;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void controlRATE() {
  //DESCRIPTION: Computes control commands based on state error (rate)
  /*
   * See explanation for controlANGLE(). Everything is the same here except the error is now the desired rate - raw gyro reading.
   */
  //Roll
  error_roll = roll_des - GyroX;
  integral_roll = integral_roll_prev + error_roll * dt;
  if (channel_thr_pwm < 1060) {  //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit);  //Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev) / dt;
  roll_PID = .01 * (Kp_roll_rate * error_roll + Ki_roll_rate * integral_roll + Kd_roll_rate * derivative_roll);  //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - GyroY;
  integral_pitch = integral_pitch_prev + error_pitch * dt;
  if (channel_thr_pwm < 1060) {  //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit);  //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev) / dt;
  pitch_PID = .01 * (Kp_pitch_rate * error_pitch + Ki_pitch_rate * integral_pitch + Kd_pitch_rate * derivative_pitch);  //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (channel_thr_pwm < 1060) {  //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit);  //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev) / dt;
  yaw_PID = .01 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw);  //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  error_roll_prev = error_roll;
  integral_roll_prev = integral_roll;
  GyroX_prev = GyroX;
  //Update pitch variables
  error_pitch_prev = error_pitch;
  integral_pitch_prev = integral_pitch;
  GyroY_prev = GyroY;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void scaleCommands() {
  //DESCRIPTION: Scale normalized actuator commands to values for ESC/Servo protocol
  /*
   * mX_command_scaled variables from the mixer function are scaled to 125-250us for OneShot125 protocol. sX_command_scaled variables from
   * the mixer function are scaled to 0-180 for the servo library using standard PWM.
   * mX_command_PWM are updated here which are used to command the motors in commandMotors(). sX_command_PWM are updated 
   * which are used to command the servos.
   */

  //Scaled to 1000-2000
  s1_command_PWM = s1_command_scaled * 1000 + 1000;
  s2_command_PWM = s2_command_scaled * 1000 + 1000;
  s3_command_PWM = s3_command_scaled * 1000 + 1000;
  s4_command_PWM = s4_command_scaled * 1000 + 1000;
  s5_command_PWM = s5_command_scaled * 1000 + 1000;
  s6_command_PWM = s6_command_scaled * 1000 + 1000;
  //Constrain commands to servos within servo library bounds
  s1_command_PWM = constrain(s1_command_PWM, 1000, 2000);
  s2_command_PWM = constrain(s2_command_PWM, 1000, 2000);
  s3_command_PWM = constrain(s3_command_PWM, 1000, 2000);
  s4_command_PWM = constrain(s4_command_PWM, 1000, 2000);
  s5_command_PWM = constrain(s5_command_PWM, 1000, 2000);

  MotorInput1 = s1_command_PWM;
  MotorInput2 = s2_command_PWM;
  MotorInput3 = s3_command_PWM;
  MotorInput4 = s4_command_PWM;
}

void controlMixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   * should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
   * normalized (0 to 1) thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with 
   * roll_passthru, pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled variables are used in scaleCommands() 
   * in preparation to be sent to the motor ESCs and servos.
   * 
   *Relevant variables:
   *thro_des - direct thottle control
   *roll_PID, pitch_PID, yaw_PID - stabilized axis variables
   *roll_passthru, pitch_passthru, yaw_passthru - direct unstabilized command passthrough
   *channel_6_pwm - free auxillary channel, can be used to toggle things with an 'if' statement
   */

  //Quad mixing - EXAMPLE
  // m1_command_scaled = thro_des - pitch_PID + roll_PID + yaw_PID; //Front Left
  // m2_command_scaled = thro_des - pitch_PID - roll_PID - yaw_PID; //Front Right
  // m3_command_scaled = thro_des + pitch_PID - roll_PID + yaw_PID; //Back Right
  // m4_command_scaled = thro_des + pitch_PID + roll_PID - yaw_PID; //Back Left
  // m5_command_scaled = 0;
  // m6_command_scaled = 0;

  // m1_command_scaled = thro_des + pitch_PID; //Front
  // m2_command_scaled = thro_des - roll_PID; //Right
  // m3_command_scaled = thro_des - pitch_PID; //Back
  // m4_command_scaled = thro_des + roll_PID; //Left

  //0.5 is centered servo, 0.0 is zero throttle if connecting to ESC for conventional PWM, 1.0 is max throttle
  s1_command_scaled = thro_des + pitch_PID;
  s2_command_scaled = thro_des - roll_PID;
  s3_command_scaled = thro_des - pitch_PID;
  s4_command_scaled = thro_des + roll_PID;
  s5_command_scaled = 0;
  s6_command_scaled = 0;
}

void throttleCut() {
  //DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*
      Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum (120 is
      minimum for oneshot125 protocol, 0 is minimum for standard PWM servo library used) if channel 5 is high. This is the last function
      called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
      the motors to anything other than minimum value. Safety first.

      channel_5_pwm is LOW then throttle cut is OFF and throttle value can change. (ThrottleCut is DEACTIVATED)
      channel_5_pwm is HIGH then throttle cut is ON and throttle value = 120 only. (ThrottleCut is ACTIVATED), (drone is DISARMED)
  */
  if (channel_thr_pwm < 1060) {
    // armedFly = false;
    MotorInput1 = 1000;
    MotorInput2 = 1000;
    MotorInput3 = 1000;
    MotorInput4 = 1000;
    MotorInput5 = 1000;
    MotorInput6 = 1000;

    //Uncomment if using servo PWM variables to control motor ESCs
    //s1_command_PWM = 0;
    //s2_command_PWM = 0;
    //s3_command_PWM = 0;
    //s4_command_PWM = 0;
    //s5_command_PWM = 0;
    //s6_command_PWM = 0;
    //s7_command_PWM = 0;
  }
}

void getImuData() {
  roll_IMU = Roll;
  pitch_IMU = Pitch;
  yaw_IMU = Yaw;
  GyroX = RateRoll;
  GyroY = RatePitch;
  GyroZ = RateYaw;
}


void customPrint() {
  Serial.print("PIDGainPitch| ");
  Serial.print("KpA_P:");
  Serial.print(Kp_pitch_angle, 3);
  Serial.print(" | ");
  Serial.print("KiA_P:");
  Serial.print(Ki_pitch_angle, 3);
  Serial.print(" | ");
  Serial.print("KdA_P:");
  Serial.println(Kd_pitch_angle, 3);

  Serial.print("PIDAnglePitch| ");
  Serial.print("des_P:");
  Serial.print(pitch_des, 3);
  Serial.print(" | ");
  Serial.print("imu_p:");
  Serial.print(pitch_IMU, 3);
  Serial.print(" | ");
  Serial.print("I_R:");
  Serial.print(integral_pitch, 3);
  Serial.print(" | ");
  Serial.print("D_R:");
  Serial.print(derivative_pitch, 3);
  Serial.print(" | ");
  Serial.print("PID_P:");
  Serial.println(pitch_PID, 3);

  //   Serial.print("PIDGainRoll| ");
  // Serial.print("KpA_R:");
  // Serial.print(Kp_roll_angle, 3);
  // Serial.print(" | ");
  // Serial.print("KiA_R:");
  // Serial.print(Ki_roll_angle, 3);
  // Serial.print(" | ");
  // Serial.print("KdA_R:");
  // Serial.println(Kd_roll_angle, 3);

  // Serial.print("PIDAngleRoll| ");
  // Serial.print("des_R:");
  // Serial.print(roll_des, 3);
  // Serial.print(" | ");
  // Serial.print("imu_p:");
  // Serial.print(roll_IMU, 3);
  // Serial.print(" | ");
  // Serial.print("I_R:");
  // Serial.print(integral_roll, 3);
  // Serial.print(" | ");
  // Serial.print("D_R:");
  // Serial.print(derivative_roll, 3);
  // Serial.print(" | ");
  // Serial.print("PID_R:");
  // Serial.println(roll_PID, 3);

  Serial.print("MotorPID| ");
  Serial.print("M1P:");
  Serial.print(MotorInput1);
  Serial.print(" | ");
  Serial.print("M2P:");
  Serial.print(MotorInput2);
  Serial.print(" | ");
  Serial.print("M3P:");
  Serial.print(MotorInput3);
  Serial.print(" | ");
  Serial.print("M4P:");
  Serial.print(MotorInput4);
  Serial.print(" | ");
  Serial.print("M5P:");
  Serial.print(MotorInput5);
  Serial.print(" | ");
  Serial.print("M6P:");
  Serial.println(MotorInput6);
}

void loopCustom() {
  getImuData();
  getDesState();
  //PID Controller - SELECT ONE:
  controlANGLE();  //Stabilize on angle setpoint
  //controlANGLE2(); //Stabilize on angle setpoint using cascaded method. Rate controller must be tuned well first!
  // controlRATE();  //Stabilize on rate setpoint
  controlMixer();
  scaleCommands();
  throttleCut();

#ifdef ENABLE_CUSTOM_PRINT
  customPrint();
#endif
}




#endif




// 10. User input for Throttle
#ifdef ENABLE_USER_INPUT
int data;
int timerCounter;
void displayInstructions() {
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

void userInput() {
  float temp;
  if (Serial.available()) {

    data = Serial.parseInt();
    switch (data) {
      // 0 char  == 48 ascii
      case 18:
        Serial.println("\nSending minimum throttle\n");
        input_throttle = MIN_PULSE_LENGTH;
        Serial.print("input_throttle min: ");
        Serial.println(input_throttle);
        break;

      // 1
      case 1:
        Serial.println("\nSending maximum throttle\n");
        // input_throttle = MAX_PULSE_LENGTH;
        Serial.print("input_throttle max: ");
        Serial.println(input_throttle);
        break;

      // 2
      case 2:
        Serial.println("\nRunning test in 3\n");
        delay(1000);
        Serial.print(" 2");
        delay(1000);
        Serial.println(" 1...");
        delay(1000);
        test();
        break;
      // 3
      case 3:
        Serial.println("\nThrottle : 1100\n");
        input_throttle = 1100;
        Serial.print("input_throttle: ");
        Serial.println(input_throttle);
        break;
      // 4
      case 4:
        Serial.println("\nThrottle : 1200\n");
        input_throttle = 1200;
        Serial.print("input_throttle: ");
        Serial.println(input_throttle);
        break;
      // 5
      case 5:
        Serial.println("\nThrottle : 1300\n");
        input_throttle = 1300;
        Serial.print("input_throttle: ");
        Serial.println(input_throttle);
        break;
      // 6
      case 6:
        Serial.println("\nThrottle Add + 50us\n");
        input_throttle = input_throttle + 50;
        Serial.print("input_throttle: ");
        Serial.println(input_throttle);
        break;
      // 7
      case 7:
        Serial.println("\nThrottle  Subtract - 50us\n");
        input_throttle = input_throttle - 50;
        Serial.print("input_throttle: ");
        Serial.println(input_throttle);
        break;
      // 8
      case 8:
        Serial.println("\nThrottle  Down-3S\n");
        throttle_down_3s();
        break;
      // 9
      case 9:
        Serial.println("\nThrottle Down-2S\n");
        throttle_down_2s();
        break;
      //10
      case 10:
        Serial.println("SP_RPM: 1000");
        setPointRpm = 1000;
        break;
      //11
      case 11:
        Serial.println("SP_RPM: 3000");
        setPointRpm = 3000;
        break;
      //12
      case 12:
        Serial.println("SP_RPM: 5000");
        setPointRpm = 5000;
        break;
      //13
      case 13:
        Serial.println("SP_RPM: 7000");
        setPointRpm = 7000;
        break;
      //14
      case 14:
        Serial.println("SP_RPM: +500");
        setPointRpm = +500;
        break;
      //15
      case 15:
        Serial.println("SP_RPM: -500");
        setPointRpm = -500;
        break;
      //16
      case 16:
        Serial.println("SP_RPM: +1000");
        setPointRpm = +1000;
        break;
      //17
      case 17:
        Serial.println("SP_RPM: -1000");
        setPointRpm = -1000;
        break;
      case 19:
        Serial.println("\nSet Throttle for Motor 1\n");
        Serial.println("Enter value (e.g., 1100):");
        motor1_throttle = Serial.parseInt();
        // Konfirmasi nilai throttle yang dimasukkan
        if (motor1_throttle > 0 && motor1_throttle < 1001) {
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
        motor2_throttle = Serial.parseInt();
        if (motor2_throttle > 0 && motor2_throttle < 1001) {
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
        motor3_throttle = Serial.parseInt();
        if (motor3_throttle > 0 && motor3_throttle < 1001) {
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
        motor4_throttle = Serial.parseInt();
        if (motor4_throttle > 0 && motor4_throttle < 1001) {
          Serial.print("Motor 4 Throttle set to: ");
          Serial.println(motor4_throttle);
        } else {
          Serial.println("Invalid input. Please enter a valid number.");
        }
        // Serial.print("Motor 4 Throttle set to: "); Serial.println(motor4_throttle);
        // runMotor4(motor4_throttle); // Fungsi untuk motor 4
        break;
      case 23:
        refRoll = Serial.parseInt();
        if (refRoll > -46 && refRoll < 46) {
          Serial.print("Ref ROll SET TO: ");
          Serial.println(refRoll);
        } else {
          Serial.println("Invalid input. Please enter a valid number.");
        }
        break;
      case 24:
        #ifdef ENABLE_FUZZY_ROLL
        StartFuzzyRoll = Serial.parseInt();
        #else
        bool StartFuzzyRoll;
        #endif

        StartFuzzyRoll = Serial.parseInt();
        if (StartFuzzyRoll) {
          Serial.println("Starting Fuzzy Roll");
        } else if (StartFuzzyRoll == 0) {
          Serial.println("Stoping Fuzzy Roll");

        #ifdef ENABLE_FUZZY_ROLL
          resetFuzzyRoll();
        #endif
        }
        break;
      case 31:
        temp = Serial.parseFloat();
        if (temp > 8 || temp < 0) {
          Serial.println("Out off Range PAngleRoll");
        } else {
          // PAnglePitch = temp;
        }
        break;
      case 32:
        temp = Serial.parseFloat();
        if (temp > 8 || temp < 0) {
          Serial.println("Out off Range IAnglePitch");
        } else {
          // IAnglePitch = temp;
        }
        break;
      case 33:
        temp = Serial.parseFloat();
        if (temp > 8 || temp < 0) {
          Serial.println("Out off Range DAngleRoll");
        } else {
          // DAnglePitch = temp;
        }
        break;
      case 34:
        temp = Serial.parseFloat();
        if (temp > 8 || temp < 0) {
          Serial.println("Out off Range PRateRoll");
        } else {
          // PRatePitch = temp;
        }
        break;
      case 35:
        temp = Serial.parseFloat();
        if (temp > 8 || temp < 0) {
          Serial.println("Out off Range PRateRoll");
        } else {
          // IRatePitch = temp;
        }
        break;
      case 36:
        temp = Serial.parseFloat();
        if (temp > 8 || temp < 0) {
          Serial.println("Out off Range PRateRoll");
        } else {
          // DRatePitch = temp;
        }
        break;
      case 41:
        iteration = 0;
      break;
    }
  }
}

void test() {
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
  Serial.print("input_throttle: ");
  Serial.println(input_throttle);
}

void throttle_down_3s() {
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

void throttle_down_2s() {
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

void initLed() {
  pinMode(ledTeensy, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  digitalWrite(ledTeensy, HIGH);
  digitalWrite(ledRed, LOW);
  digitalWrite(ledGreen, HIGH);
}

void ledBlink() {
  if (!SafetyState){
    digitalWrite(ledRed, HIGH);
  }
  else{
    digitalWrite(ledRed, LOW);
  }
}
#endif


#ifdef ENABLE_MONITORING_1HZ
void monitoring1Hz() {
#ifdef ENABLE_MONITORING_IMU_1HZ
  Serial.print("IMU| ");
  Serial.print("Gx:");
  Serial.print(RateRoll);
  Serial.print(" | ");
  Serial.print("Gy:");
  Serial.print(RatePitch);
  Serial.print(" | ");
  Serial.print("Gz:");
  Serial.print(RateYaw);
  Serial.print(" | ");
  Serial.print("R:");
  Serial.print(Roll);
  Serial.print(" | ");
  Serial.print("P:");
  Serial.print(Pitch);
  Serial.print(" | ");
  Serial.print("Y:");
  Serial.println(Yaw);
#endif
#ifdef ENABLE_MONITORING_BMP_1HZ
  Serial.print("BMP| ");
  Serial.print("Alt:");
  Serial.print(alti);
  Serial.print(" | ");
  Serial.print("Pressure:");
  Serial.print(press);
  Serial.print(" | ");
  Serial.print("Temp:");
  Serial.println(temp);
#endif
#ifdef ENABLE_MONITORING_LIDAR_1HZ
  Serial.print("Lidar| ");
  Serial.print("Dist: ");
  Serial.print(Altitude);
  Serial.print(" | ");
  Serial.print("Strength: ");
  Serial.println(strength);


#endif
#ifdef ENABLE_MONITORING_VOLTAGE_CURRENT_1HZ
  Serial.print("Analog| ");
  Serial.print("V: ");
  Serial.print(voltage);
  Serial.print(" | ");
  Serial.print("I: ");
  Serial.print(current);
  Serial.print(" | ");
  Serial.print("V_volt: ");
  Serial.print(voltageVolt);
  Serial.print(" | ");
  Serial.print("I_volt: ");
  Serial.println(currentVolt);
#endif
#ifdef ENABLE_MONITORING_MOTOR_1HZ
  Serial.print("Motor| ");
  Serial.print("M1: ");
  Serial.print(MotorInput1);
  Serial.print("| M2: ");
  Serial.print(MotorInput2);
  Serial.print("| M3: ");
  Serial.print(MotorInput3);
  Serial.print("| M4: ");
  Serial.print(MotorInput4);
  Serial.print("| M5: ");
  Serial.print(MotorInput5);
  Serial.print("| M6: ");
  Serial.println(MotorInput6);
#endif
#ifdef ENABLE_MONITORING_PID_1HZ
  Serial.print("PID| ");
  Serial.print("iT:");
  Serial.print(InputThrottle);
  Serial.print(" | ");
  Serial.print("iR:");
  Serial.print(InputRoll);
  Serial.print(" | ");
  Serial.print("iP:");
  Serial.print(InputPitch);
  Serial.print(" | ");
  Serial.print("iY:");
  Serial.print(InputYaw);
  Serial.println(" | ");

  Serial.print("MotorPID| ");
  Serial.print("M1P:");
  Serial.print(MotorInput1);
  Serial.print(" | ");
  Serial.print("M2P:");
  Serial.print(MotorInput2);
  Serial.print(" | ");
  Serial.print("M3P:");
  Serial.print(MotorInput3);
  Serial.print(" | ");
  Serial.print("M4P:");
  Serial.print(MotorInput4);
  Serial.print(" | ");
  Serial.print("M5P:");
  Serial.print(MotorInput5);
  Serial.print(" | ");
  Serial.print("M6P:");
  Serial.println(MotorInput6);
#endif
#ifdef ENABLE_MONITORING_RPM_1HZ
  Serial.print("RPM| ");
  Serial.print("RPM:");
  Serial.println(measuredRPM);
  // Serial.print("RPM_Filtered");Serial.println(current);
#endif
#ifdef ENABLE_MONITORING_RTC_1HZ
  Serial.print("RTC| ");
  Serial.print(currentHour);
  Serial.print(":");
  Serial.print(currentMinute);
  Serial.print(":");
  Serial.println(currentSecond);
#endif
}
#endif

#ifdef ENABLE_MONITORING_10HZ
void monitoring10Hz() {
#ifdef ENABLE_MONITORING_IMU_10HZ
  Serial.print("IMU| ");
  Serial.print("Gx:");
  Serial.print(RateRoll);
  Serial.print(" | ");
  Serial.print("Gy:");
  Serial.print(RatePitch);
  Serial.print(" | ");
  Serial.print("Gz:");
  Serial.print(RateYaw);
  Serial.print(" | ");
  Serial.print("R:");
  Serial.print(Roll);
  Serial.print(" | ");
  Serial.print("P:");
  Serial.print(Pitch);
  Serial.print(" | ");
  Serial.print("Y:");
  Serial.println(Yaw);
#endif
#ifdef ENABLE_MONITORING_BMP_10HZ
  Serial.print("BMP| ");
  Serial.print("Alt:");
  Serial.print(alti);
  Serial.print(" | ");
  Serial.print("Pressure:");
  Serial.print(press);
  Serial.print(" | ");
  Serial.print("Temp:");
  Serial.println(temp);
#endif
#ifdef ENABLE_MONITORING_LIDAR_10HZ
  Serial.print("Lidar| ");
  Serial.print("Dist: ");
  Serial.print(Altitude);
  Serial.print(" | ");
  Serial.print("Strength: ");
  Serial.println(strength);
#endif
#ifdef ENABLE_MONITORING_VOLTAGE_CURRENT_10HZ
  Serial.print("Analog| ");
  Serial.print("V: ");
  Serial.print(voltage);
  Serial.print(" | ");
  Serial.print("I: ");
  Serial.print(current);
  Serial.print(" | ");
  Serial.print("V_volt: ");
  Serial.print(voltageVolt);
  Serial.print(" | ");
  Serial.print("I_volt: ");
  Serial.println(currentVolt);
#endif
#ifdef ENABLE_MONITORING_MOTOR_10HZ
  Serial.print("Motor| ");
  Serial.print("M1: ");
  Serial.print(MotorInput1);
  Serial.print("| M2: ");
  Serial.print(MotorInput2);
  Serial.print("| M3: ");
  Serial.print(MotorInput3);
  Serial.print("| M4: ");
  Serial.print(MotorInput4);
  Serial.print("| M5: ");
  Serial.print(MotorInput5);
  Serial.print("| M6: ");
  Serial.println(MotorInput6);
#endif
#ifdef ENABLE_MONITORING_PID_10HZ
  Serial.print("PID| ");
  Serial.print("iT:");
  Serial.print(InputThrottle);
  Serial.print(" | ");
  Serial.print("iR:");
  Serial.print(InputRoll);
  Serial.print(" | ");
  Serial.print("iP:");
  Serial.print(InputPitch);
  Serial.print(" | ");
  Serial.print("iY:");
  Serial.print(InputYaw);
  Serial.println(" | ");

  Serial.print("MotorPID| ");
  Serial.print("M1P:");
  Serial.print(MotorInput1);
  Serial.print(" | ");
  Serial.print("M2P:");
  Serial.print(MotorInput2);
  Serial.print(" | ");
  Serial.print("M3P:");
  Serial.print(MotorInput3);
  Serial.print(" | ");
  Serial.print("M4P:");
  Serial.print(MotorInput4);
  Serial.print(" | ");
  Serial.print("M5P:");
  Serial.print(MotorInput5);
  Serial.print(" | ");
  Serial.print("M6P:");
  Serial.println(MotorInput6);
#endif
#ifdef ENABLE_MONITORING_FUZZY_ROLL_10HZ
  Serial.print("Fuzzy Roll | ");
  Serial.print("iR:");
  Serial.print(Throttle_Roll);
  Serial.print(" | ");

  Serial.print("MotorFuzzy | ");
  Serial.print("M1P:");
  Serial.print(MotorInput1);
  Serial.print(" | ");
  Serial.print("M2P:");
  Serial.print(MotorInput2);
  Serial.print(" | ");
  Serial.print("M3P:");
  Serial.print(MotorInput3);
  Serial.print(" | ");
  Serial.print("M4P:");
  Serial.print(MotorInput4);
  Serial.print(" | ");
  // Serial.print("M5P:");Serial.print(MotorInput5); Serial.print(" | ");
  // Serial.print("M6P:");Serial.println(MotorInput6);
#endif
#ifdef ENABLE_MONITORING_RPM_10HZ
  Serial.print("RPM| ");
  Serial.print("RPM:");
  Serial.println(rpm_1);
  // Serial.print("RPM_Filtered");Serial.println(current);
#endif
#ifdef ENABLE_MONITORING_RTC_10HZ
  Serial.print("RTC| ");
  Serial.print(currentHour);
  Serial.print(":");
  Serial.print(currentMinute);
  Serial.print(":");
  Serial.println(currentSecond);
#endif
}
#endif

#ifdef ENABLE_SERIAL1
float inputData[20];
char bufferSerial1In[512];
int lenSerial1In;
int channelSerial1 = 0;

void initSerial1() {
  Serial1.begin(115200);
  SerialUSB1.println("SerialUSB1 - ACTIVE");
}
void logSerial1() {
  char bufferSerial1[512];
#ifdef ENABLE_PID_ANGLE_RATE
   
  // Normal Logging
  //                                                                1   2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35
  int lenSerial1 = snprintf(bufferSerial1, sizeof(bufferSerial1), "%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                            cMillis /*1*/, input_throttle /*2*/, MotorInput1 /*3*/, MotorInput2 /*4*/, MotorInput3 /*5*/, MotorInput4 /*6*/, Roll /*7*/, Pitch /*8*/, YawNorm /*9*/, RateRoll /*10*/, RatePitch /*11*/, RateYaw /*12*/, DesiredAngleRoll /*13*/, DesiredRateRoll /*14*/, InputRoll /*15*/, DesiredAnglePitch /*16*/, DesiredRatePitch /*17*/, InputPitch /*18*/, DesiredAngleYaw/*19*/, DesiredRateYaw/*20*/, InputYaw/*21*/,Altitude/*22*/, DesiredAltitude/*23*/, InputThrottle/*24*/);
  
  // RPM Sensor Logging
                                                                //  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20
  // int lenSerial1 = snprintf(bufferSerial1, sizeof(bufferSerial1), "%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
  

  SerialUSB1.write(bufferSerial1, lenSerial1);  // Write the entire buffer at once

#elif ENABLE_CUSTOM
  //                                                                 1   2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20
  int lenSerial1 = snprintf(bufferSerial1, sizeof(bufferSerial1), "%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f   ,%f,%f,%f   ,%f,%f,%f\n",
                            cMillis /*1*/, input_throttle /*2*/, MotorInput1 /*3*/, MotorInput2 /*4*/, MotorInput3 /*5*/, MotorInput4 /*6*/, Roll /*7*/, Pitch /*8*/, Yaw /*9*/, RateRoll /*10*/, RatePitch /*11*/, RateYaw /*12*/, roll_des /*13*/, roll_PID /*14*/, pitch_des /*15*/, pitch_PID /*16*/);
  SerialUSB1.write(bufferSerial1, lenSerial1);  // Write the entire buffer at once

#else
  // //                                                                 1   2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20
  // int lenSerial1 = snprintf(bufferSerial1, sizeof(bufferSerial1), "%lu,%f,%f,%f,%f,%f,%f,%f\n",
  //                           cMillis /*1*/, input_throttle /*2*/,MotorInput1 /*3*/, MotorInput2 /*4*/, MotorInput3 /*5*/, MotorInput4/*6*/,pulseInterval_1 /*7*/, rpm_1 /*8*/);
  // SerialUSB1.write(bufferSerial1, lenSerial1);  // Write the entire buffer at once

#endif

// #ifdef ENABLE_GPS

//   //                                                                1   2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20
//   int lenSerial1 = snprintf(bufferSerial1, sizeof(bufferSerial1), "%lu,%d,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
//                             cMillis /*1*/, GPSFix/*2*/, latitude /*3*/, longitude /*4*/, altitude /*5*/, speed /*6*/, angle /*7*/, satellites /*8*/, y_1 /*9*/, x_1 /*10*/, y_2 /*11*/, x_2 /*12*/, deltaX /*13*/, deltaY /*14*/, lat1/*15*/, lon1/*16*/, lat2/*17*/,lon2/*18*/);
//   SerialUSB1.write(bufferSerial1, lenSerial1);  // Write the entire buffer at once

// #endif
}
void readSerial1() {
  // Serial.println(inputData[0]);
  // Serial.println(inputData[1]);

  if (SerialUSB1.available() > 0) {
    // Serial.println("\n---------INPUT from EXCEL-----------\n");
    for (channelSerial1 = 0; channelSerial1 < 20; channelSerial1++) {
      if (SerialUSB1.available() > 0) {
        inputData[channelSerial1] = SerialUSB1.parseFloat();  // Membaca nilai float dan menyimpannya ke array
        #ifdef ENABLE_CUSTOM
          integral_roll = 0;
          integral_pitch = 0;
        #endif
      }
    }
    #ifdef ENABLE_CUSTOM
        input_throttle = inputData[1];
        Kp_pitch_angle = inputData[2];
        Ki_pitch_angle = inputData[3];
        Kd_pitch_angle = inputData[4];
        Kp_roll_angle = inputData[5];
        Ki_roll_angle = inputData[6];
        Kd_roll_angle = inputData[7];
        refPitch = inputData[8];
        refRoll = inputData[9];
    #endif
    #ifdef ENABLE_PID_ANGLE_RATE
        // input_throttle = inputData[1];

        // Pitch Tuning
        // PRatePitch = inputData[5];
        // IRatePitch = inputData[6];
        // DRatePitch = inputData[7];
        // PAnglePitch = inputData[2];
        // IAnglePitch = inputData[3];
        // DAnglePitch = inputData[4];

        // Yaw Tuning
        // PRateYaw = inputData[5];
        // IRateYaw = inputData[6];
        // DRateYaw = inputData[7];
        // PAngleYaw = inputData[2];
        // IAngleYaw = inputData[3];
        // DAngleYaw = inputData[4];

        // // Velocity Tuning
        // PVerticalVelocity = inputData[5];
        // IVerticalVelocity = inputData[6];
        // DVerticalVelocity = inputData[7];

        // // Altitude Tuning
        // PAltitude = inputData[2];
        // IAltitude = inputData[3];
        // DAltitude = inputData[4];

        // Position X Tuning
        PPosX = inputData[2];
        IPosX = inputData[3];
        DPosX = inputData[4];

        // // Position Y Tuning
        PPosY = inputData[5];
        IPosY = inputData[6];
        DPosY = inputData[7];
        RollMin = inputData[8];
        RollMax = inputData[9];


        // // Position XY
        // dX = inputData[8];
        // dY = inputData[9];

        // IMU Calibration
        // manualCalRoll = inputData[5];
        // manualCalPitch = inputData[6];

    #endif
    // #ifdef ENABLE_RPM_1


    // #endif

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

#ifdef ENABLE_SEQUENCE_EXCECUTION
  void sequenceExcecution(){
    iteration++;
    if (iteration <= 1000) {
      // Increase setPointRoll from 0 to 20
      // setPointPitch = 0;
      InputThrottle = 1000;
      channel_thr_pwm = 1000;
      setPointRoll = 0;
      setPointPitch = 0;
      setPointYaw = 0;
      // setPointAltitude = 5;
      // setPointRoll = 0;
      // input_throttle = 1400;
      // motor4_throttle = 0;
    }
    else if (iteration <= 2000) {
      // Decrease setPointRoll from 20 to 0
      // refRoll = -5;
      InputThrottle = 1300;
      channel_thr_pwm = 1111;
      setPointRoll = 0;
      setPointPitch = 0;
      setPointYaw = 0;
      // setPointAltitude = 100;
      // motor4_throttle = 100;
      // setPointPitch = 5;
      // setPointRoll = 5;
      // input_throttle = 1400;
    }
    else if (iteration <= 3000) {
      // Decrease setPointRoll from 0 to -20
      // refRoll = 0;
      InputThrottle = 1300;
      setPointRoll = 0;
      setPointPitch = 10;
      setPointYaw = 0;
      // motor4_throttle = 200;
      // setPointPitch = 0;
      // setPointRoll = 0;
      // input_throttle = 1400;
      setPointAltitude = 100;
    }
    else if (iteration <= 4000) {
      // Decrease setPointRoll from 0 to -20
      // refRoll = -10;
      InputThrottle = 1300;
      setPointRoll = 0;
      setPointPitch = 10;
      setPointYaw = 0;
      // motor4_throttle = 300;
      // setPointPitch = 10;
      // setPointRoll = 10;
      // input_throttle = 1400;
      setPointAltitude = 100;
    }
    else if (iteration <= 5000) {
      // Decrease setPointRoll from 0 to -20
      // refRoll = 0;
      InputThrottle = 1300;
      setPointRoll = 0;
      setPointPitch = 10;
      setPointYaw = 0;
      // motor4_throttle = 400;
      // setPointPitch = 0;
      // setPointRoll = 0;
      // input_throttle = 1400;
      setPointAltitude = 100;
    }
    else if (iteration <= 6000) {
      // Decrease setPointRoll from 0 to -20
      // refRoll = -15;
      InputThrottle = 1300;
      setPointRoll = 0;
      setPointPitch = 0;
      setPointYaw = 0;
      setPointAltitude = 5;
      // motor4_throttle = 500;
      // setPointPitch = 15;
      // setPointRoll = 15;
      // input_throttle = 1400;
    }
    // else if (iteration <= 7000) {
    //   // Decrease setPointRoll from 0 to -20
    //   // refRoll = 0;
    //   InputThrottle = 1300;
    //   setPointRoll = 0;
    //   setPointPitch = 10;
    //   setPointYaw = 0;
    //   // motor4_throttle = 600;
    //   // setPointPitch = 0;
    //   // setPointRoll = 0;
    //   // input_throttle = 1400;
    // }
    // else if (iteration <= 8000) {
    //   // Decrease setPointRoll from 0 to -20
    //   // refRoll = -20;
    //   InputThrottle = 1300;
    //   setPointRoll = 0;
    //   setPointPitch = 0;
    //   setPointYaw = 0;
    //   // motor4_throttle = 700;
    //   // setPointPitch = 20;
    //   // setPointRoll = 20;
    //   // input_throttle = 1400;
    // }
    // else if (iteration <= 9000) {
    //   // Decrease setPointRoll from 0 to -20
    //   // refRoll = 0;
    //   InputThrottle = 1300;
    //   setPointRoll = 0;
    //   setPointPitch = 20;
    //   setPointYaw = 0;
    //   // motor4_throttle = 800;
    //   // setPointPitch = 0;
    //   // setPointRoll = 0;
    //   // input_throttle = 1400;
    // }
    // else if (iteration <= 10000) {
    //   // Decrease setPointRoll from 0 to -20
    //   // refRoll = -25;
    //   InputThrottle = 1300;
    //   setPointRoll = 0;
    //   setPointPitch = 0;
    //   setPointYaw = 0;
    //   // motor4_throttle = 900;
    //   // setPointPitch = 25;
    //   // setPointRoll = 25;
    //   // input_throttle = 1400;
    // }
    // else if (iteration <= 11000) {
    //   // Decrease setPointRoll from 0 to -20
    //   // refRoll = 0;
    //   InputThrottle = 1300;
    //   setPointRoll = 0;
    //   setPointPitch = 0;
    //   setPointYaw = 10;
    //   // motor4_throttle = 1000;
    //   // setPointPitch = 0;
    //   // setPointRoll = 0;
    //   // input_throttle = 1400;
    // }
    // else if (iteration <= 12000) {
    //   // Decrease setPointRoll from 0 to -20
    //   // refRoll = -30;
    //   InputThrottle = 1300;
    //   setPointRoll = 0;
    //   setPointPitch = 0;
    //   setPointYaw = 0;
    //   // motor4_throttle = 0;
    //   // setPointPitch = 30;
    //   // setPointRoll = 30;
    //   // input_throttle = 1400;
    // }
    // else if (iteration <= 13000) {
    //   // Decrease setPointRoll from 0 to -20
    //   // refRoll = 0;
    //   InputThrottle = 1300;
    //   setPointRoll = 0;
    //   setPointPitch = 0;
    //   setPointYaw = 20;
    //   // motor4_throttle = 500;
    //   // setPointPitch = 0;
    //   // setPointRoll = 0;
    //   // input_throttle = 1400;
    // }
    // else if (iteration <= 14000){
    //   // refRoll = 0;  // Reset iteration to create a loop
    //   InputThrottle = 1300;
    //   setPointRoll = 0;
    //   setPointPitch = 0;
    //   setPointYaw = 0;
    //   // motor4_throttle = 0;
    //   // setPointPitch = 0;
    //   // setPointRoll = 0;
    //   // input_throttle = 1000;
    // }
    // else if (iteration <= 15000){
    //   // refRoll = 0;  // Reset iteration to create a loop
    //   InputThrottle = 1300;
    //   setPointRoll = 10;
    //   setPointPitch = 10;
    //   setPointYaw = 10;
    //   // motor4_throttle = 0;
    //   // setPointPitch = 0;
    //   // setPointRoll = 0;
    //   // input_throttle = 1000;
    // }
    // else if (iteration <= 16000){
    //   // refRoll = 0;  // Reset iteration to create a loop
    //   InputThrottle = 1300;
    //   setPointRoll = 0;
    //   setPointPitch = 0;
    //   setPointYaw = 0;
    //   // motor4_throttle = 0;
    //   // setPointPitch = 0;
    //   // setPointRoll = 0;
    //   // input_throttle = 1000;
    // }

    else{
      // setPointPitch = 0;
      channel_thr_pwm = 1000;
      InputThrottle = 1000;
      setPointRoll = 0;
      setPointPitch = 0;
      setPointYaw = 0;
      setPointAltitude = 5;
      // setPointRoll = 0;
      // input_throttle = 1000;
    }
  }
#endif
//========================================================================================================================//
//                                                      VOID SETUP                                                        //
//========================================================================================================================//


void setup() {
digitalWrite(ledTeensy, HIGH);
// Initialize each module based on preprocessor flags
#ifdef ENABLE_SERIAL1
  Serial.begin(115200);  // Serial initialization
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
  // available @ positionImu();
#endif

#ifdef ENABLE_GPS
  initGPS();
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

#ifdef ENABLE_PID_ANGLE_RATE

#endif

#ifdef ENABLE_CUSTOM

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

#ifdef ENABLE_PID_ANGLE_RATE

#endif

#ifdef ENABLE_LED
  initLed();
  Serial.println("LED START");
// available @ ledBlink();
#endif

#ifdef ENABLE_INTERVAL
// initInterval(); // Tidak perlu
#endif

  // F = {1, 0.01,
  //           0, 1};  
  // G = {0.5*0.01*0.01,
  //      0.01};
  // H = {1, 0};
  // I = {1, 0,
  //      0, 1};
  // Q = G * ~G;
  // Q = Q*100;
  // R = {30*30};
  // P = {0, 0,
  //      0, 0};
  // S = {0,
  //      0};
  SafetyState = true;
}

void loop() {
  cMicros = micros();
  cMillis = millis();

  #ifdef ENABLE_GPS
  loopGPS();
  #endif


  // 500Hz
  if (cMicros - pM500Hz >= interval500Hz) {
// Fuction & Command
#ifdef ENABLE_MOTOR
#ifdef ENABLE_MOTOR_MANUAL_ALL
    input_throttle = ReceiverValue[2];
    MotorInput1 = input_throttle;
    MotorInput2 = input_throttle;
    MotorInput3 = input_throttle;
    MotorInput4 = input_throttle;
    MotorInput5 = input_throttle;
    MotorInput6 = input_throttle;
#elif defined(ENABLE_MOTOR_MANUAL_1) || defined(ENABLE_MOTOR_MANUAL_2) || defined(ENABLE_MOTOR_MANUAL_3) || defined(ENABLE_MOTOR_MANUAL_4) || defined(ENABLE_MOTOR_MANUAL_5) || defined(ENABLE_MOTOR_MANUAL_6)
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
#ifdef ENABLE_MOTOR_MANUAL_5
    MotorInput5 = input_throttle;
#endif
#ifdef ENABLE_MOTOR_MANUAL_6
    MotorInput6 = input_throttle;
#endif
#elif defined(ENABLE_MOTOR_MANUAL_PER_MOTOR)
    MotorInput1 = motor1_throttle + input_throttle;
    MotorInput2 = motor2_throttle + input_throttle;
    MotorInput3 = motor3_throttle + input_throttle;
    MotorInput4 = motor4_throttle + input_throttle;
    MotorInput5 = motor5_throttle + input_throttle;
    MotorInput6 = motor6_throttle + input_throttle;
#endif
    if (SafetyState == false){
      runMotor(MIN_PULSE_LENGTH, MIN_PULSE_LENGTH, MIN_PULSE_LENGTH, MIN_PULSE_LENGTH, MIN_PULSE_LENGTH, MIN_PULSE_LENGTH);
    }
    else{
      runMotor(MotorInput1, MotorInput2, MotorInput3, MotorInput4, MotorInput5, MotorInput6);
    }
    
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
  if (cMillis - pM100Hz >= interval100Hz) {
  // Serial.print("cMillisStart: "); Serial.println(millis());
#ifdef ENABLE_SEQUENCE_EXCECUTION 
  sequenceExcecution();
#endif
    // Fuction & Command
#ifdef ENABLE_IMU
    readImu();
    positionImu();
#endif

#ifdef ENABLE_RECEIVER
    readReceiver();
#endif

#ifdef ENABLE_LIDAR
    readLidar();
#endif

#ifdef ENABLE_USER_INPUT
    userInput();
#endif

#ifdef ENABLE_SERIAL1_INPUT
    readSerial1();
#endif

#ifdef ENABLE_PID_ANGLE
    loopPidAngle();
#endif
#ifdef ENABLE_PID_RATE
    loopPidRate();
#endif


#ifdef ENABLE_FUZZY_ROLL
    runFuzzyRoll();
#endif

#ifdef ENABLE_PID_ANGLE_RATE
    loopPidAngleRate100Hz();
    pidAngleRateLimit();
#endif

#ifdef ENABLE_CUSTOM
    //PID Controller - SELECT ONE:
    loopCustom();
    // controlANGLE(); //Stabilize on angle setpoint
    // controlANGLE2(); //Stabilize on angle setpoint using cascaded method. Rate controller must be tuned well first!
    //controlRATE(); //Stabilize on rate setpoint
#endif


#ifdef ENABLE_SERIAL1_100HZ
    logSerial1();
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



#ifdef ENABLE_MONITORING_100HZ
  #ifdef ENABLE_MOTOR_PRINT_100HZ
    Serial.print("Motor| ");
    Serial.print("M1: ");
    Serial.print(MotorInput1);
    Serial.print("| M2: ");
    Serial.print(MotorInput2);
    Serial.print("| M3: ");
    Serial.print(MotorInput3);
    Serial.print("| M4: ");
    Serial.print(MotorInput4);
    Serial.print("| M5: ");
    Serial.print(MotorInput5);
    Serial.print("| M6: ");
    Serial.println(MotorInput6);
  #endif
  #ifdef ENABLE_LIDAR_PRINT_100HZ
    Serial.print("Lidar| ");
    Serial.print("Dist: ");
    Serial.print(Altitude);
    Serial.print(" | ");
    Serial.print("VVelocity: ");
    Serial.print(verticalVelocity);
    Serial.print(" | ");
    Serial.print("Strength: ");
    Serial.print(strength);
    Serial.print(" | ");
    Serial.print("period: ");
    Serial.println(lidarTimer);
  #endif

#endif


    // End Timer
    pM100Hz = cMillis;
    // Serial.print("cMillisEnd: "); Serial.println(cMillis);
  }

  if (cMillis - pM20Hz >= interval20Hz) {
    
    // #ifdef ENABLE_LIDAR
    // readLidar();
    // #endif

    #ifdef ENABLE_PID_ANGLE_RATE
    loopPidAngleRate20Hz();
    #endif

    // End Timer
    pM20Hz = cMillis;
  }

  if (cMillis - pM5Hz >= interval5Hz) {
    
    // #ifdef ENABLE_LIDAR
    // readLidar();
    // #endif

    #ifdef ENABLE_PID_ANGLE_RATE
    loopPidAngleRate5Hz();
    #endif

    // End Timer
    pM5Hz = cMillis;
  }

  // 10Hz
  if (cMillis - pM10Hz >= interval10Hz) {
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
  if (cMillis - pM1Hz >= interval1Hz) {
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