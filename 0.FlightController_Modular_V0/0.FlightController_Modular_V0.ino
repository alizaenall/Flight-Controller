/*
 * LED
 * BNO055 (Teruji di 500Hz) (perlu dikalibrasi)
 * BMP280 
 * Lidar (Max 100Hz)
 * Current & Votage Sensor (voltage oke)
 * Motor 
 * PID, blm selesai
 * PID RollPitchYawRate & RollPitchYawEuler 250Hz, PID Altitude & Position 100Hz 
 *
 * version  V0.1
 * date  2024-11-8
 * Author : alizaenal
 */



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

unsigned long loopTimer = 0;

// PID 
// float RatePitch, RateRoll, RateYaw;




void setup(){
  // Receiver
  ReceiverInput.begin(receiverPin);
  ReceiverValue[2] = 1040;
    while (ReceiverValue[2] < 1010 ||
    ReceiverValue[2] > 1050) {
      read_receiver();
      Serial.println(ReceiverValue[2]);
      delay(4);
    }

  Serial.println("\n\nEND OF SETUP\n\n");
  digitalWrite(13, LOW);
  loopTimer=micros();
}

void loop(){
  
  oneHzLoop();
  tenHzLoop();
  hundredHzLoop();
  // 250 Hz Loop
  if (micros() - pM4 < 4000){
  }
  else{
  read_receiver();
  // InputThrottle=ReceiverValue[2];
  runMotor();
  // Serial.print("PWM Cyle: ");Serial.println(micros() - pM4);
  pM4=micros();
  }

  loopTime = micros() - pM5;
  pM5= micros();
  // Serial.print("LoopTime: "); Serial.println(loopTime);

}
void oneHzLoop(){
    // 1 Hz Loop 
  cM1 = millis();
  if (cM1 - pM1 >= interval1) {
    digitalWrite(ledRed, !digitalRead(ledRed)); // Indikator Looping
    
    // Time RTC
    Serial.print("Hour: ");
    Serial.print(currentHour);
    Serial.print(", Minute: ");
    Serial.print(currentMinute);
    Serial.print(", Second: ");
    Serial.println(currentSecond);

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
    
    // PWM Signal
    Serial.print("M1: "); Serial.print(MotorInput1);Serial.print(" || M2: "); Serial.print(MotorInput2);Serial.print(" || M3: "); Serial.print(MotorInput3);Serial.print(" || M4: "); Serial.println(MotorInput4);
    Serial.print("IThr: "); Serial.print(InputThrottle);Serial.print(" || IRol: "); Serial.print(InputRoll);Serial.print(" || IPit: ");Serial.print(InputPitch);Serial.print(" || IYaw: ");Serial.println(InputYaw);
    Serial.println();
    pM1 = cM1;
  }
}

void tenHzLoop(){
  // 10 Hz Loop
  cM3 = millis();
  if (cM3 - pM3 >= interval3) {
    

    // Voltage Current Sensor
    voltageCurrentRead();

    pM3 = cM3;
  }
}

void hundredHzLoop(){
  rtc();
  // 100 Hz Loop
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

  pidLoop();
  logData();
  }
}
