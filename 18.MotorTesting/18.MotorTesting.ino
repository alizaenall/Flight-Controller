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
    8 : Throttle Down in 3S\n");
    9 : Throttle Down in 2S\n");
*/


#include <PulsePosition.h>
#define receiverPin 9
#define mot1Pin 2 //2
#define mot2Pin 3 //3
#define mot3Pin 4 //4
#define mot4Pin 5 //5

// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs

uint32_t LoopTimer;
float input_throttle = MIN_PULSE_LENGTH;
char data;

int timerCounter = 0;

void setup() {
  Serial.begin(57600);
  //LED 13-BuiltIn 30-RED 31-GREEN
  pinMode(13, OUTPUT); 
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
  digitalWrite(13, HIGH);
  digitalWrite(30, HIGH);
  digitalWrite(31, HIGH);

  // analogWriteFrequency(mot1Pin, 250);
  analogWriteFrequency(mot2Pin, 250);
  // analogWriteFrequency(mot3Pin, 250);
  // analogWriteFrequency(mot4Pin, 250);
  analogWriteResolution(12);
  delay(250);

  LoopTimer=micros();

  displayInstructions();
}
void loop() {
  if (Serial.available()) {
      data = Serial.read();

      switch (data) {
          // 0 char  == 48 ascii
          case 48 : Serial.println("Sending minimum throttle");
                    input_throttle = MIN_PULSE_LENGTH;
                    Serial.println(input_throttle);
          break;

          // 1
          case 49 : Serial.println("Sending maximum throttle");
                    input_throttle = MAX_PULSE_LENGTH;
                    Serial.println(input_throttle);
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
                    Serial.println(input_throttle);
          break;
          // 4
          case 52 : Serial.print("Throttle : 1200");
                    input_throttle = 1200;
                    Serial.println(input_throttle);
          break;
          // 5
          case 53 : Serial.print("Throttle : 1300");
                    input_throttle = 1300;
                    Serial.println(input_throttle);
          break;
          // 6
          case 54 : Serial.print("Add + 100us");
                    input_throttle = input_throttle + 100;
                    Serial.println(input_throttle);
          break;
          // 7
          case 55 : Serial.print("Subtract - 100us");
                    input_throttle = input_throttle - 100;
                    Serial.println(input_throttle);
          break;
          // // 8
          // case 56 : Serial.print("Down-3S");
          //           throttle_down_3s();
          // break;
          // // 9
          // case 57 : Serial.print("Down-2S");
          //           throttle_down_2s();
          // break;
      }
  }

  runMotor(input_throttle);

  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();


}

void runMotor(float thr){
    // analogWrite(mot1Pin,1.024*thr);
    analogWrite(mot2Pin,1.024*thr);
    // analogWrite(mot3Pin,1.024*thr);
    // analogWrite(mot4Pin,1.024*thr);
    Serial.println(thr);
}

void test()
{
    for (int i = MIN_PULSE_LENGTH; i <= MAX_PULSE_LENGTH; i += 10) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        // analogWrite(mot1Pin,1.024*i);
        analogWrite(mot2Pin,1.024*i);
        // analogWrite(mot3Pin,1.024*i);
        // analogWrite(mot4Pin,1.024*i);

        delay(40);
    }

    Serial.println("STOP");
    // analogWrite(mot1Pin,MIN_PULSE_LENGTH);
    analogWrite(mot2Pin,MIN_PULSE_LENGTH);
    // analogWrite(mot3Pin,MIN_PULSE_LENGTH);
    // analogWrite(mot4Pin,MIN_PULSE_LENGTH);
    input_throttle = MIN_PULSE_LENGTH;
    Serial.print("input_throttle: ");Serial.println(input_throttle);
    
}
void throttle_down_3s(){
    for (int i = MAX_PULSE_LENGTH; i >= MIN_PULSE_LENGTH; i -= 20) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        // analogWrite(mot1Pin,1.024*i);
        analogWrite(mot2Pin,1.024*i);
        // analogWrite(mot3Pin,1.024*i);
        // analogWrite(mot4Pin,1.024*i);

        timerCounter += 60;
        Serial.println(timerCounter);
        // formula delay : t*1000 / ((max_length - min_length)/i_step)
        delay(60);
    }

    Serial.println("STOP");
    // analogWrite(mot1Pin,MIN_PULSE_LENGTH);
    analogWrite(mot2Pin,MIN_PULSE_LENGTH);
    // analogWrite(mot3Pin,MIN_PULSE_LENGTH);
    // analogWrite(mot4Pin,MIN_PULSE_LENGTH);
    input_throttle = MIN_PULSE_LENGTH;

    timerCounter = 0;
}

void throttle_down_2s(){
    for (int i = MAX_PULSE_LENGTH; i >= MIN_PULSE_LENGTH; i -= 20) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        // analogWrite(mot1Pin,1.024*i);
        analogWrite(mot2Pin,1.024*i);
        // analogWrite(mot3Pin,1.024*i);
        // analogWrite(mot4Pin,1.024*i);

        timerCounter += 40;
        Serial.println(timerCounter);
        // formula delay : t*1000 / ((max_length - min_length)/i_step)
        delay(40);
    }

    Serial.println("STOP");
    // analogWrite(mot1Pin,MIN_PULSE_LENGTH);
    analogWrite(mot2Pin,MIN_PULSE_LENGTH);
    // analogWrite(mot3Pin,MIN_PULSE_LENGTH);
    // analogWrite(mot4Pin,MIN_PULSE_LENGTH);
    input_throttle = MIN_PULSE_LENGTH;

    timerCounter = 0;
}


/**
 * Displays instructions to user
 */
void displayInstructions()
{  
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("\t0 : Send min throttle");
    Serial.println("\t1 : Send max throttle");
    Serial.println("\t2 : Run test function");
    Serial.println("\t3 : Send 1100 throttle");
    Serial.println("\t4 : Send 1200 throttle");
    Serial.println("\t5 : Send 1300 throttle");
    Serial.println("\t6 : Send +50 throttle");
    Serial.println("\t7 : Send -50 throttle");
    Serial.println("\t8 : Throttle Down in 3S");
    Serial.println("\t9 : Throttle Down in 2S");
}