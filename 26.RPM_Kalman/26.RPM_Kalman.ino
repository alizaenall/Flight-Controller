/*
READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    0 : Send min throttle");
    1 : Send max throttle");
    2 : Run test function\n");
    3 : Send 1100 throttle\n");
    4 : Send 1200 throttle\n");
    5 : Send 1300 throttle\n");
    6 : Send +100 throttle\n");
    7 : Send -100 throttle\n");
*/


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

#define mot1Pin 2 //2
#define mot2Pin 3 //3
#define mot3Pin 4 //4
#define mot4Pin 5 //5

// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs

uint32_t LoopTimer;
float input_throttle = MIN_PULSE_LENGTH;
float input_throttle_sebelumnya = 0;
char data;

#define poles 12
#define rpmPin 7
unsigned long lastSampleTime = 0;   // Waktu terakhir untuk sampling tetap
unsigned long timer = 0;    // Waktu pulsa terakhir
volatile unsigned long counter = 0;
int RPM = 0;

// Variabel Kalman Filter
float x_k = 0;          // Estimasi RPM
float P_k = 1;          // Variansi estimasi
const float Q = 0.1;    // Noise proses
const float R = 10;     // Noise pengukuran

float measuredRPM = 0;    // Nilai RPM dari sensor
float filteredRPM = 0;    // Nilai RPM setelah Kalman Filter
float measuredRPM_Kalman = 0;
float measuredRPMKalman_Before = 0;

void interrupt_rpm(){
  counter++;
  asm("DSB");
}

void setup() {
  Serial.begin(57600);
  //LED 13-BuiltIn 30-RED 31-GREEN
  pinMode(13, OUTPUT); 
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
  digitalWrite(13, HIGH);
  digitalWrite(30, HIGH);
  digitalWrite(31, HIGH);

  analogWriteFrequency(mot1Pin, 500);
  // analogWriteFrequency(mot2Pin, 250);
  // analogWriteFrequency(mot3Pin, 250);
  // analogWriteFrequency(mot4Pin, 250);
  analogWriteResolution(11);
  delay(250);

  LoopTimer=micros();
  timer = micros();

  displayInstructions();

  // Receiver
  ReceiverInput.begin(receiverPin);

  pinMode(rpmPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rpmPin), interrupt_rpm, FALLING);

}

void loop() {
  read_receiver();
  if (Serial.available()) {
      data = Serial.read();

      switch (data) {
          // 0 char  == 48 ascii
          case 48 : Serial.println("Sending minimum throttle");
                    input_throttle = MIN_PULSE_LENGTH;
                    

          break;

          // 1
          case 49 : Serial.println("Sending maximum throttle");
                    input_throttle = MAX_PULSE_LENGTH;
          break;

          // 2
          case 50 : Serial.print("Running test in 3");
                    delay(1000);
                    Serial.print(" 2");
                    delay(1000);
                    Serial.println(" 1...");
                    delay(1000);
                    test();
          break;
          // 3
          case 51 : Serial.print("Throttle : 1100");
                    input_throttle = 1100;
          break;
          // 4
          case 52 : Serial.print("Throttle : 1200");
                    input_throttle = 1200;
          break;
          // 5
          case 53 : Serial.print("Throttle : 1300");
                    input_throttle = 1300;
          break;
          // 6
          case 54 : Serial.println("Add + 50us");
                    input_throttle = input_throttle + 50;
          break;
          // 7
          case 55 : Serial.println("Subtract - 50us");
                    input_throttle = input_throttle - 50;
          break;
      }
  }
  if (ReceiverValue[2] > 1100){
    input_throttle = 1300;
  } else {
    input_throttle = MIN_PULSE_LENGTH;
  }
  // if (input_throttle != input_throttle_sebelumnya){
  //   x_k = 15.4*input_throttle-15400;
  //   input_throttle_sebelumnya = input_throttle;
  // }

  runMotor(input_throttle);

  // Update measuredRPM setiap ada pulsa
  if (counter > 0) {
    float period = 0;
    period = (float)(micros() - timer) / counter;
    measuredRPM = (60000000.0 * counter) / (period * poles / 2);
    // Serial.print("Time : ");
    Serial.print(micros());
    // Serial.print(" Period : ");
    Serial.print(",");
    Serial.print(period);
    // Serial.print(" Measured rpm ");
    Serial.print(",");    
    Serial.println(measuredRPM);
    timer = micros();
    measuredRPM_Kalman = measuredRPM;
    measuredRPMKalman_Before = measuredRPM;
    counter = 0; // Reset counter setelah penghitungan
  }

  LoopTimer=micros();
  
  if (micros() - lastSampleTime >= 2000) {
    lastSampleTime = micros();

    if (micros() - timer > 10000 && input_throttle < 1050){
      measuredRPM_Kalman = 0;
    } else if (micros() - timer > 10000 && x_k > 0 && input_throttle >= 1050){
      measuredRPM_Kalman = 15.4*input_throttle-15400;
    }

    // Kalman Filter
    float x_k_minus = x_k;                   // Prediksi estimasi
    float P_k_minus = P_k + Q;               // Prediksi variansi
    float K_k = P_k_minus / (P_k_minus + R); // Gain Kalman
    x_k = x_k_minus + K_k * (measuredRPM_Kalman - x_k_minus); // Koreksi
    P_k = (1 - K_k) * P_k_minus;             // Update variansi

    if (measuredRPMKalman_Before > 0 && measuredRPM_Kalman == 0){
      x_k = 0;
    }

    filteredRPM = x_k; // Hasil akhir dari Kalman Filter

    if (x_k > 1){
      // Output data
      // Serial.print("Time Filter : ");
      // Serial.print(micros());
      // Serial.print("Measured RPM: ");
      // Serial.print(measuredRPM_Kalman);
      // Serial.print(" | Filtered RPM: ");
      // Serial.println(filteredRPM);    
    } 
  }
}

void runMotor(float thr){
    analogWrite(mot1Pin,thr+12);
    // Serial.println(thr);
    // analogWrite(mot2Pin,1.024*input_throttle);
    // analogWrite(mot3Pin,1.024*input_throttle);
    // analogWrite(mot4Pin,1.024*input_throttle);
}

void test()
{
    for (int i = MIN_PULSE_LENGTH; i <= MAX_PULSE_LENGTH; i += 5) {
        unsigned long Timer = micros();
        Serial.print("Pulse length = ");
        Serial.println(i);
        analogWrite(mot1Pin,i+12);
        // analogWrite(mot2Pin,1.024*i);
        // analogWrite(mot3Pin,1.024*i);
        // analogWrite(mot4Pin,1.024*i);
        Serial.print(Timer);
        Timer = 0;
        delay(100);
    }

    Serial.println("STOP");
    analogWrite(mot1Pin,MIN_PULSE_LENGTH);
    // analogWrite(mot2Pin,MIN_PULSE_LENGTH);
    // analogWrite(mot3Pin,MIN_PULSE_LENGTH);
    // analogWrite(mot4Pin,MIN_PULSE_LENGTH);
    input_throttle = MIN_PULSE_LENGTH;
    
}


/**
 * Displays instructions to user
 */
void displayInstructions()
{  
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("\t0 : Send min throttle");
    Serial.println("\t1 : Send max throttle");
    Serial.println("\t2 : Run test function\n");
    Serial.println("\t3 : Send 1100 throttle\n");
    Serial.println("\t4 : Send 1200 throttle\n");
    Serial.println("\t5 : Send 1300 throttle\n");
    Serial.println("\t6 : Send +50 throttle\n");
    Serial.println("\t7 : Send -50 throttle\n");
}