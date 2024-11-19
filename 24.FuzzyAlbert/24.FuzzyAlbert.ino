
#include <PulsePosition.h>
#define receiverPin 9
PulsePositionInput ReceiverInput(RISING);
float ReceiverValue[]={0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber=0;
void read_receiver(void){
  ChannelNumber = ReceiverInput.available();
  if (ChannelNumber > 0) {
  for (int i=1; i<=ChannelNumber;i++){
    ReceiverValue[i-1]=ReceiverInput.read(i);
    }
  }
}

#include <PulsePosition.h>
#include <Fuzzy.h> 
#include <Wire.h>

// Lidar
#define LIDAR Serial5  // Define the Serial1 for TF-Luna communication
// LED
#define ledRed 30
#define ledGreen 31
#define ledTeensy 13

//Motor Initial
#define MotorPin1 2 //2
#define MotorPin2 3 //3
#define MotorPin3 4 //4
#define MotorPin4 5 //5

// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs

//Throttle
float InputThrottle = MIN_PULSE_LENGTH;
float Throttle;
float prev_InputThrottle = MIN_PULSE_LENGTH;
char data;
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// Millis 1 - LED - 1 Hz
// unsigned long pM1 = 0;                // PreviousMillis
// const long interval1 = 1000;          // Interval to wait (1000 ms)
// unsigned long cM1 = 0;                // CurrentMillis
// Millis 2 - IMU Lidar - 100 Hz
unsigned long pM2 = 0;                // PreviousMillis
const long interval2 = 10;            // Interval to wait (10 ms)
unsigned long cM2 = 0;                // CurrentMillis
unsigned long cM2_lidar = 0;
// Millis 3 - BMP - 10 Hz
// unsigned long pM3 = 0;                // PreviousMillis
// const long interval3 = 50;            // Interval to wait (100 ms)
// unsigned long cM3 = 0;                // CurrentMillis
// Micros 1 - PWM Motor - 4ms - 250Hz
unsigned long pM4 = 0;                // PreviousMillis
// Micros 2 - Loop Time
// unsigned long loopTime = 0;
// unsigned long pM5 = 0;
// Millis 4 - Fuzzy Motor - 200 Hz
unsigned long pM6 = 0;                // PreviousMillis
const long interval6 = 5;            // Interval to wait (100 ms)
unsigned long cM6 = 0;                // CurrentMillis

unsigned long LoopTimer = 0;

//Variabel Jarak
int setpoint_jarak = 50; 
int delta_jarak; 
int jarak = 0; 
int jarakSebelumnya = 0; 

//LIDAR
uint8_t recvBuffer[9];  // Buffer to store received data
int distance;           // Distance measured by the LiDAR
int strength;           // Signal strength

//Fuzzy Initial
Fuzzy *fuzzy = new Fuzzy();

//INPUT Jarak Z
FuzzySet *AltitudeVeryHigh = new FuzzySet(-100, -100, -80, -40);
FuzzySet *AltitudeHigh = new FuzzySet(-100, -30, -30, 0);
FuzzySet *AltitudeNormal = new FuzzySet(-35, 0, 0, 35);
FuzzySet *AltitudeLow = new FuzzySet(0, 30, 30, 100);
FuzzySet *AltitudeVeryLow = new FuzzySet(40, 80, 100, 100);
// FuzzySet *AltitudeVeryLow = new FuzzySet(1000, 1000, 800, 400);
// FuzzySet *AltitudeLow = new FuzzySet(1000, 300, 300, 0);
// FuzzySet *AltitudeNormal = new FuzzySet(350, 0, 0, -350);
// FuzzySet *AltitudeHigh = new FuzzySet(0, -300, -300, -1000);
// FuzzySet *AltitudeVeryHigh = new FuzzySet(-400, -800, -1000, -1000);
//INPUT Kecepatan Z
FuzzySet *SpeedZVeryLow = new FuzzySet(-10, -10, -8, -4);
FuzzySet *SpeedZLow = new FuzzySet(-10, -3, -3, -0);
FuzzySet *SpeedZNormal = new FuzzySet(-3.5, 0, 0, 3.5);
FuzzySet *SpeedZHigh = new FuzzySet(0, 3, 3, 10);
FuzzySet *SpeedZVeryHigh = new FuzzySet(4, 8, 10, 10);
// FuzzySet *SpeedZVeryLow = new FuzzySet(-100, -100, -80, -40);
// FuzzySet *SpeedZLow = new FuzzySet(-100, -30, -30, -0);
// FuzzySet *SpeedZNormal = new FuzzySet(-35, 0, 0, 35);
// FuzzySet *SpeedZHigh = new FuzzySet(0, 30, 30, 100);
// FuzzySet *SpeedZVeryHigh = new FuzzySet(40, 80, 100, 100);
//OUTPUT PWM
FuzzySet *ZPWMVeryLow = new FuzzySet(-0.25, -0.1625, -0.1625, -0.075);
FuzzySet *ZPWMLow = new FuzzySet(-0.175, -0.075, -0.075, 0.025);
FuzzySet *ZPWMNormal = new FuzzySet(-0.055, 0, 0, 0.055);
FuzzySet *ZPWMHigh = new FuzzySet(-0.025, 0.075, 0.075, 0.175);
FuzzySet *ZPWMVeryHigh = new FuzzySet(0.075, 0.1625, 0.1625, 0.25);
// FuzzySet *ZPWMVeryLow = new FuzzySet(-0.05, -0.0325, -0.0325, -0.015);
// FuzzySet *ZPWMLow = new FuzzySet(-0.035, -0.015, -0.015, 0.005);
// FuzzySet *ZPWMNormal = new FuzzySet(-0.011, 0, 0, 0.011);
// FuzzySet *ZPWMHigh = new FuzzySet(-0.005, 0.015, 0.015, 0.035);
// FuzzySet *ZPWMVeryHigh = new FuzzySet(0.015, 0.0325, 0.0325, 0.05);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  //-----Setup LED 13-BuiltIn 30-RED 31-GREEN
  pinMode(13, OUTPUT); 
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
  digitalWrite(13, HIGH);
  digitalWrite(30, HIGH);
  digitalWrite(31, HIGH);

  //-----Lidar
  LIDAR.begin(115200);        // LiDAR UART
  Serial.println("TF-Luna LiDAR End");

  //PWM Motor
  analogWriteFrequency(MotorPin1, 250);
  analogWriteFrequency(MotorPin2, 250);
  analogWriteFrequency(MotorPin3, 250);
  analogWriteFrequency(MotorPin4, 250);
  analogWriteResolution(12);
  delay(250);

    // Receiver
  ReceiverInput.begin(receiverPin);
    while (ReceiverValue[2] < 1010 ||
    ReceiverValue[2] > 1050) {
      read_receiver();
      Serial.println(ReceiverValue[2]);
      delay(4);
    }
    
  Serial.println("\n\nEND OF SETUP\n\n");
  digitalWrite(13, LOW);
  LoopTimer=micros();

  // FuzzyInput Jarak Z
  FuzzyInput *AltitudeFuzzy = new FuzzyInput(1);
  AltitudeFuzzy->addFuzzySet(AltitudeVeryLow);
  AltitudeFuzzy->addFuzzySet(AltitudeLow);
  AltitudeFuzzy->addFuzzySet(AltitudeNormal);
  AltitudeFuzzy->addFuzzySet(AltitudeHigh);
  AltitudeFuzzy->addFuzzySet(AltitudeVeryHigh);
  fuzzy->addFuzzyInput(AltitudeFuzzy);

  // FuzzyInput Kecepatan Z
  FuzzyInput *SpeedZFuzzy = new FuzzyInput(2);
  SpeedZFuzzy->addFuzzySet(SpeedZVeryLow);
  SpeedZFuzzy->addFuzzySet(SpeedZLow);
  SpeedZFuzzy->addFuzzySet(SpeedZNormal);
  SpeedZFuzzy->addFuzzySet(SpeedZHigh);
  SpeedZFuzzy->addFuzzySet(SpeedZVeryHigh);
  fuzzy->addFuzzyInput(SpeedZFuzzy);

  //FuzzyOutput PWM
  FuzzyOutput *ZPWM = new FuzzyOutput(1);
  ZPWM->addFuzzySet(ZPWMVeryLow);
  ZPWM->addFuzzySet(ZPWMLow);
  ZPWM->addFuzzySet(ZPWMNormal);
  ZPWM->addFuzzySet(ZPWMHigh);
  ZPWM->addFuzzySet(ZPWMVeryHigh);
  fuzzy->addFuzzyOutput(ZPWM);

  //Building FuzzyRule
  //--Altitude Very Low
  FuzzyRuleAntecedent* ifAltitudeVeryLowAndSpeedZVeryLow = new FuzzyRuleAntecedent();
  ifAltitudeVeryLowAndSpeedZVeryLow->joinWithAND(AltitudeVeryLow, SpeedZVeryLow);
  FuzzyRuleConsequent *thenZPWMVeryHigh = new FuzzyRuleConsequent(); 
  thenZPWMVeryHigh->addOutput(ZPWMVeryHigh); 
  FuzzyRule *rule1 = new FuzzyRule(1, ifAltitudeVeryLowAndSpeedZVeryLow, thenZPWMVeryHigh); 
  fuzzy->addFuzzyRule(rule1);

  FuzzyRuleAntecedent* ifAltitudeVeryLowAndSpeedZLow = new FuzzyRuleAntecedent();
  ifAltitudeVeryLowAndSpeedZLow->joinWithAND(AltitudeVeryLow, SpeedZLow);
  // FuzzyRuleConsequent *thenZPWMVeryHigh = new FuzzyRuleConsequent(); 
  // thenZPWMVeryHigh->addOutput(ZPWMVeryHigh); 
  FuzzyRule *rule2 = new FuzzyRule(2, ifAltitudeVeryLowAndSpeedZLow, thenZPWMVeryHigh); 
  fuzzy->addFuzzyRule(rule2);

  FuzzyRuleAntecedent* ifAltitudeVeryLowAndSpeedZNormal = new FuzzyRuleAntecedent();
  ifAltitudeVeryLowAndSpeedZNormal->joinWithAND(AltitudeVeryLow, SpeedZNormal);
  // FuzzyRuleConsequent *thenZPWMVeryHigh = new FuzzyRuleConsequent(); 
  // thenZPWMVeryHigh->addOutput(ZPWMVeryHigh); 
  FuzzyRule *rule3 = new FuzzyRule(3, ifAltitudeVeryLowAndSpeedZNormal, thenZPWMVeryHigh); 
  fuzzy->addFuzzyRule(rule3);

  FuzzyRuleAntecedent* ifAltitudeVeryLowAndSpeedZHigh = new FuzzyRuleAntecedent();
  ifAltitudeVeryLowAndSpeedZHigh->joinWithAND(AltitudeVeryLow, SpeedZHigh);
  FuzzyRuleConsequent *thenZPWMHigh = new FuzzyRuleConsequent(); 
  thenZPWMHigh->addOutput(ZPWMHigh); 
  FuzzyRule *rule4 = new FuzzyRule(4, ifAltitudeVeryLowAndSpeedZHigh, thenZPWMHigh); 
  fuzzy->addFuzzyRule(rule4);
  
  FuzzyRuleAntecedent* ifAltitudeVeryLowAndSpeedZVeryHigh = new FuzzyRuleAntecedent();
  ifAltitudeVeryLowAndSpeedZVeryHigh->joinWithAND(AltitudeVeryLow, SpeedZVeryHigh);
  FuzzyRuleConsequent *thenZPWMNormal = new FuzzyRuleConsequent(); 
  thenZPWMNormal->addOutput(ZPWMNormal); 
  FuzzyRule *rule5 = new FuzzyRule(5, ifAltitudeVeryLowAndSpeedZVeryHigh, thenZPWMNormal); 
  fuzzy->addFuzzyRule(rule5);

  //--Altitude Low
  FuzzyRuleAntecedent* ifAltitudeLowAndSpeedZVeryLow = new FuzzyRuleAntecedent();
  ifAltitudeLowAndSpeedZVeryLow->joinWithAND(AltitudeLow, SpeedZVeryLow);
  // FuzzyRuleConsequent *thenZPWMVeryHigh = new FuzzyRuleConsequent(); 
  // thenZPWMVeryHigh->addOutput(ZPWMVeryHigh); 
  FuzzyRule *rule6 = new FuzzyRule(6, ifAltitudeLowAndSpeedZVeryLow, thenZPWMVeryHigh); 
  fuzzy->addFuzzyRule(rule6);

  FuzzyRuleAntecedent* ifAltitudeLowAndSpeedZLow = new FuzzyRuleAntecedent();
  ifAltitudeLowAndSpeedZLow->joinWithAND(AltitudeLow, SpeedZLow);
  // FuzzyRuleConsequent *thenZPWMVeryHigh = new FuzzyRuleConsequent(); 
  // thenZPWMVeryHigh->addOutput(ZPWMVeryHigh); 
  FuzzyRule *rule7 = new FuzzyRule(7, ifAltitudeLowAndSpeedZLow, thenZPWMVeryHigh); 
  fuzzy->addFuzzyRule(rule7);

  FuzzyRuleAntecedent* ifAltitudeLowAndSpeedZNormal = new FuzzyRuleAntecedent();
  ifAltitudeLowAndSpeedZNormal->joinWithAND(AltitudeLow, SpeedZNormal);
  // FuzzyRuleConsequent *thenZPWMHigh = new FuzzyRuleConsequent(); 
  // thenZPWMHigh->addOutput(ZPWMHigh); 
  FuzzyRule *rule8 = new FuzzyRule(8, ifAltitudeLowAndSpeedZNormal, thenZPWMHigh); 
  fuzzy->addFuzzyRule(rule8);

  FuzzyRuleAntecedent* ifAltitudeLowAndSpeedZHigh = new FuzzyRuleAntecedent();
  ifAltitudeLowAndSpeedZHigh->joinWithAND(AltitudeLow, SpeedZHigh);
  // FuzzyRuleConsequent *thenZPWMNormal = new FuzzyRuleConsequent(); 
  // thenZPWMNormal->addOutput(ZPWMNormal); 
  FuzzyRule *rule9 = new FuzzyRule(9, ifAltitudeLowAndSpeedZHigh, thenZPWMNormal); 
  fuzzy->addFuzzyRule(rule9);
  
  FuzzyRuleAntecedent* ifAltitudeLowAndSpeedZVeryHigh = new FuzzyRuleAntecedent();
  ifAltitudeLowAndSpeedZVeryHigh->joinWithAND(AltitudeLow, SpeedZVeryHigh);
  FuzzyRuleConsequent *thenZPWMLow = new FuzzyRuleConsequent(); 
  thenZPWMLow->addOutput(ZPWMLow); 
  FuzzyRule *rule10 = new FuzzyRule(10, ifAltitudeLowAndSpeedZVeryHigh, thenZPWMLow); 
  fuzzy->addFuzzyRule(rule10);

  //--Altitude Normal
  FuzzyRuleAntecedent* ifAltitudeNormalAndSpeedZVeryLow = new FuzzyRuleAntecedent();
  ifAltitudeNormalAndSpeedZVeryLow->joinWithAND(AltitudeNormal, SpeedZVeryLow);
  // FuzzyRuleConsequent *thenZPWMVeryHigh = new FuzzyRuleConsequent(); 
  // thenZPWMVeryHigh->addOutput(ZPWMVeryHigh); 
  FuzzyRule *rule11 = new FuzzyRule(11, ifAltitudeNormalAndSpeedZVeryLow, thenZPWMVeryHigh); 
  fuzzy->addFuzzyRule(rule11);

  FuzzyRuleAntecedent* ifAltitudeNormalAndSpeedZLow = new FuzzyRuleAntecedent();
  ifAltitudeNormalAndSpeedZLow->joinWithAND(AltitudeNormal, SpeedZLow);
  // FuzzyRuleConsequent *thenZPWMVeryHigh = new FuzzyRuleConsequent(); 
  // thenZPWMVeryHigh->addOutput(ZPWMVeryHigh); 
  FuzzyRule *rule12 = new FuzzyRule(12, ifAltitudeNormalAndSpeedZLow, thenZPWMHigh); 
  fuzzy->addFuzzyRule(rule12);

  FuzzyRuleAntecedent* ifAltitudeNormalAndSpeedZNormal = new FuzzyRuleAntecedent();
  ifAltitudeNormalAndSpeedZNormal->joinWithAND(AltitudeNormal, SpeedZNormal);
  // FuzzyRuleConsequent *thenZPWMVeryHigh = new FuzzyRuleConsequent(); 
  // thenZPWMVeryHigh->addOutput(ZPWMVeryHigh); 
  FuzzyRule *rule13 = new FuzzyRule(13, ifAltitudeNormalAndSpeedZNormal, thenZPWMNormal); 
  fuzzy->addFuzzyRule(rule13);

  FuzzyRuleAntecedent* ifAltitudeNormalAndSpeedZHigh = new FuzzyRuleAntecedent();
  ifAltitudeNormalAndSpeedZHigh->joinWithAND(AltitudeNormal, SpeedZHigh);
  // FuzzyRuleConsequent *thenZPWMLow = new FuzzyRuleConsequent(); 
  // thenZPWMLow->addOutput(ZPWMLow);
  FuzzyRule *rule14 = new FuzzyRule(14, ifAltitudeNormalAndSpeedZHigh, thenZPWMLow); 
  fuzzy->addFuzzyRule(rule14);

  FuzzyRuleAntecedent* ifAltitudeNormalAndSpeedZVeryHigh = new FuzzyRuleAntecedent();
  ifAltitudeNormalAndSpeedZVeryHigh->joinWithAND(AltitudeNormal, SpeedZVeryHigh);
  FuzzyRuleConsequent *thenZPWMVeryLow = new FuzzyRuleConsequent(); 
  thenZPWMVeryLow->addOutput(ZPWMVeryLow); 
  FuzzyRule *rule15 = new FuzzyRule(15, ifAltitudeNormalAndSpeedZVeryHigh, thenZPWMVeryLow); 
  fuzzy->addFuzzyRule(rule15);

  //--Altitude High
  FuzzyRuleAntecedent* ifAltitudeHighAndSpeedZVeryLow = new FuzzyRuleAntecedent();
  ifAltitudeHighAndSpeedZVeryLow->joinWithAND(AltitudeHigh, SpeedZVeryLow);
  FuzzyRule *rule16 = new FuzzyRule(16, ifAltitudeHighAndSpeedZVeryLow, thenZPWMHigh); 
  fuzzy->addFuzzyRule(rule16);

  FuzzyRuleAntecedent* ifAltitudeHighAndSpeedZLow = new FuzzyRuleAntecedent();
  ifAltitudeHighAndSpeedZLow->joinWithAND(AltitudeHigh, SpeedZLow);
  FuzzyRule *rule17 = new FuzzyRule(17, ifAltitudeHighAndSpeedZLow, thenZPWMNormal); 
  fuzzy->addFuzzyRule(rule17);

  FuzzyRuleAntecedent* ifAltitudeHighAndSpeedZNormal = new FuzzyRuleAntecedent();
  ifAltitudeHighAndSpeedZNormal->joinWithAND(AltitudeHigh, SpeedZNormal);
  FuzzyRule *rule18 = new FuzzyRule(18, ifAltitudeHighAndSpeedZNormal, thenZPWMLow); 
  fuzzy->addFuzzyRule(rule18);

  FuzzyRuleAntecedent* ifAltitudeHighAndSpeedZHigh = new FuzzyRuleAntecedent();
  ifAltitudeHighAndSpeedZHigh->joinWithAND(AltitudeHigh, SpeedZHigh);
  FuzzyRule *rule19 = new FuzzyRule(19, ifAltitudeHighAndSpeedZHigh, thenZPWMVeryLow); 
  fuzzy->addFuzzyRule(rule19);

  FuzzyRuleAntecedent* ifAltitudeHighAndSpeedZVeryHigh = new FuzzyRuleAntecedent();
  ifAltitudeHighAndSpeedZVeryHigh->joinWithAND(AltitudeHigh, SpeedZVeryHigh);
  FuzzyRule *rule20 = new FuzzyRule(20, ifAltitudeHighAndSpeedZVeryHigh, thenZPWMVeryLow); 
  fuzzy->addFuzzyRule(rule20);

  //--Altitude Very High
  FuzzyRuleAntecedent* ifAltitudeVeryHighAndSpeedZVeryLow = new FuzzyRuleAntecedent();
  ifAltitudeVeryHighAndSpeedZVeryLow->joinWithAND(AltitudeVeryHigh, SpeedZVeryLow);
  FuzzyRule *rule21 = new FuzzyRule(21, ifAltitudeVeryHighAndSpeedZVeryLow, thenZPWMNormal); 
  fuzzy->addFuzzyRule(rule21);

  FuzzyRuleAntecedent* ifAltitudeVeryHighAndSpeedZLow = new FuzzyRuleAntecedent();
  ifAltitudeVeryHighAndSpeedZLow->joinWithAND(AltitudeVeryHigh, SpeedZLow); 
  FuzzyRule *rule22 = new FuzzyRule(22, ifAltitudeVeryHighAndSpeedZLow, thenZPWMLow); 
  fuzzy->addFuzzyRule(rule22);

  FuzzyRuleAntecedent* ifAltitudeVeryHighAndSpeedZNormal = new FuzzyRuleAntecedent();
  ifAltitudeVeryHighAndSpeedZNormal->joinWithAND(AltitudeVeryHigh, SpeedZNormal);
  FuzzyRule *rule23 = new FuzzyRule(23, ifAltitudeVeryHighAndSpeedZNormal, thenZPWMVeryLow); 
  fuzzy->addFuzzyRule(rule23);

  FuzzyRuleAntecedent* ifAltitudeVeryHighAndSpeedZHigh = new FuzzyRuleAntecedent();
  ifAltitudeVeryHighAndSpeedZHigh->joinWithAND(AltitudeVeryHigh, SpeedZHigh);
  FuzzyRule *rule24 = new FuzzyRule(24, ifAltitudeVeryHighAndSpeedZHigh, thenZPWMVeryLow); 
  fuzzy->addFuzzyRule(rule24);

  FuzzyRuleAntecedent* ifAltitudeVeryHighAndSpeedZVeryHigh = new FuzzyRuleAntecedent();
  ifAltitudeVeryHighAndSpeedZVeryHigh->joinWithAND(AltitudeVeryHigh, SpeedZVeryHigh); 
  FuzzyRule *rule25 = new FuzzyRule(25, ifAltitudeVeryHighAndSpeedZVeryHigh, thenZPWMVeryLow); 
  fuzzy->addFuzzyRule(rule25);
}

void loop() {
  // put your main code here, to run repeatedly:
  hundredHzLoop();
  twoHundredHzLoop();
  runMotor();

  while (micros() - pM4 < 4000);
  pM4 = micros();
}


void hundredHzLoop(){
  // 100 Hz Loop
  cM2 = millis();
  // cM2_lidar = millis();
  if (cM2 - pM2 >= interval2) {
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
    jarak = setpoint_jarak - distance; //(INPUT 1) 
    delta_jarak = jarak - jarakSebelumnya; //(INPUT 2) 
    jarakSebelumnya = jarak;
    Serial.print("Jarak : ");
    Serial.print(jarak);
    Serial.print(" Delta jarak : ");
    Serial.print(delta_jarak);
    Serial.print(" || Lidar: ");
    Serial.print(distance);
    Serial.print(" cm || ");
  }
}

void twoHundredHzLoop(){
  cM6 = millis();
  read_receiver();
  if (cM6 - pM6 >= interval6 && ReceiverValue[2] > 1100) {
    Serial.print("Jarak : ");
    Serial.print(jarak);
    fuzzy->setInput(1, jarak);
    fuzzy->setInput(2, delta_jarak);
    fuzzy->fuzzify();
    Throttle = fuzzy->defuzzify(1);
    Serial.print(" d_Throttle : ");
    Serial.println(Throttle);
    InputThrottle = prev_InputThrottle + Throttle;
    if (InputThrottle > 1600){
      InputThrottle = 1600;
    }

    MotorInput1= 1.024*InputThrottle;
    MotorInput2= 1.024*InputThrottle;
    MotorInput3= 1.024*InputThrottle;
    MotorInput4= 1.024*InputThrottle;
    Serial.print(" Input Throttle : ");
    Serial.println(InputThrottle);
    prev_InputThrottle = InputThrottle;

    pM6 = cM6;
  }
  else if (cM6 - pM6 >= interval6){
    prev_InputThrottle = 1000;
  }
}

void runMotor(){
    // analogWrite(motorTestPin,1.024*thr);
    // Serial.println(thr);
  analogWrite(MotorPin1,MotorInput1);
  analogWrite(MotorPin2,MotorInput2);
  analogWrite(MotorPin3,MotorInput3); 
  analogWrite(MotorPin4,MotorInput4);
}