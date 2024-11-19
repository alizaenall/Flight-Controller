#define pot A9

// Millis 1 - LED - 1 Hz
unsigned long pM1 = 0;                // PreviousMillis
const long interval1 = 1000;          // Interval to wait (1000 ms)
unsigned long cM1 = 0;                // CurrentMillis
// Millis 2 - IMU - 20 Hz
unsigned long pM2 = 0;                // PreviousMillis
const long interval2 = 50;          // Interval to wait (50 ms)
unsigned long cM2 = 0;                // CurrentMillis
// Millis 3 - BMP - 10 Hz
unsigned long pM3 = 0;                // PreviousMillis
const long interval3 = 50;          // Interval to wait (100 ms)
unsigned long cM3 = 0;                // CurrentMillis


// PWM
float inputThrottle = 0;
float potAdc = 0;

void read_adc(){
  potAdc = analogRead(pot);
  inputThrottle = map(potAdc,0,1023,1000,2000); // 1000us (1ms = 0% throttle) to 2000us (2ms = 100% throttle)
}

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
  
//-----Setup PWM
  analogWriteFrequency(4, 250);
  analogWriteResolution(12);
  delay(250);
    while (inputThrottle < 1020 || inputThrottle > 1050) {
      read_adc();
      Serial.println(inputThrottle);
      delay(4); // 4ms (250Hz)
    }

  Serial.println("\n\nEND OF SETUP\n\n");
}

void loop() 
{
  // 1 Hz Loop 
  cM1 = millis();

  read_adc();
  analogWrite(4,1.024*inputThrottle);

  if (cM1 - pM1 >= interval1) {
    digitalWrite(3, !digitalRead(3)); // Indikator Looping
    Serial.print("ADC: "); Serial.print(potAdc); Serial.print(" bit || thr: "); Serial.print(inputThrottle); Serial.println(" [us]");
    
    // Data IMU
    // Serial.print("X: "); Serial.print(roll_rate, 3); Serial.print(" || "); 
    
    // Serial.print("Y: "); Serial.print(pitch_rate, 3); Serial.print(" || ");
    
    // Serial.print("Z: "); Serial.print(yaw_rate, 3); Serial.print(" || "); 
    
    // // DATA BMP280
    // Serial.print("temp: "); Serial.print(temp); Serial.print(" || ");
    // Serial.print("p(PA): "); Serial.print(press); Serial.print(" || ");
    // Serial.print("alt(m): "); Serial.println(alti);
    
    pM1 = cM1;
  }

  // // 20 Hz Loop
  // cM2 = millis();
  // if (cM2 - pM2 >= interval2) {
  //   mpu.readAngularVelocity();  /* read Angular Velocity */
  //   // Variable IMU
  //   roll_rate = mpu.GyrData.x;  // Roll Rate
  //   pitch_rate = mpu.GyrData.y;  // Pitch Rate
  //   yaw_rate = mpu.GyrData.z;  // Yaw Rate

  //   pM2 = cM2;
  // }

  // // 10 Hz Loop
  //   cM3 = millis();
  // if (cM3 - pM3 >= interval3) {
  //   temp = bmp.getTemperature();
  //   press = bmp.getPressure();
  //   alti = bmp.calAltitude(SEA_LEVEL_PRESSURE, press);

  //   pM3 = cM3;
  // }
}

