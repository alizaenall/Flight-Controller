/***************************************************************************
 * Example sketch for the Teensy-DShot library
 *
 * This example initiates the ESCs, increases the throttle to maximum speed,
 * and then decreases the throttle to zero.
 *
 ***************************************************************************/
#include <Arduino.h>

#include "DShot.h"

constexpr uint16_t LOOP_HZ = 2000;

DShot motor0(&Serial2, DShotType::DShot600); // Teensy4.X Pin 8
DShot motor1(&Serial3, DShotType::DShot600); // Teensy4.X Pin 14
DShot motor2(&Serial4, DShotType::DShot600); // Teensy4.X Pin 17
DShot motor3(&Serial5, DShotType::DShot600); // Teensy4.X Pin 20

uint64_t counter = 0;
int16_t throttle = 0;
// int8_t throttleChange = 1;
uint8_t ledState = false;
#define MIN_PULSE_LENGTH 0
#define MAX_PULSE_LENGTH 2047
float input_throttle = 0;

  float motor1_throttle = 0;
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
    Serial.println("   17  \tSP_RPM: -1000");
    Serial.println("   19 (4 digit PWM) \tSend Throttle to Motor 1 Manual");
    Serial.println("   20 (4 digit PWM) \tSend Throttle to Motor 2 Manual");
    Serial.println("   21 (4 digit PWM) \tSend Throttle to Motor 3 Manual");
    Serial.println("   22 (4 digit PWM) \tSend Throttle to Motor 4 Manual\n\n");
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
            // // 8
            // case 8 : Serial.println("\nThrottle  Down-3S\n");
            //           throttle_down_3s();
            // break;
            // // 9
            // case 9 : Serial.println("\nThrottle Down-2S\n");
            //           throttle_down_2s();
            // break;
            //10
            // case 10 : Serial.println("SP_RPM: 1000");
            //           setPointRpm = 1000;
            // break;
            // //11
            // case 11 : Serial.println("SP_RPM: 3000");
            //           setPointRpm = 3000;
            // break;
            // //12
            // case 12 : Serial.println("SP_RPM: 5000");
            //           setPointRpm = 5000;
            // break;
            // //13
            // case 13 : Serial.println("SP_RPM: 7000");
            //           setPointRpm = 7000;
            // break;
            // //14
            // case 14 : Serial.println("SP_RPM: +500");
            //           setPointRpm =+ 500;
            // break;
            // //15
            // case 15 : Serial.println("SP_RPM: -500");
            //           setPointRpm =- 500;
            // break;
            // //16
            // case 16 : Serial.println("SP_RPM: +1000");
            //           setPointRpm =+ 1000;
            // break;
            // //17
            // case 17 : Serial.println("SP_RPM: -1000");
            //           setPointRpm =- 1000;
            // break;
            // case 19:
            //     Serial.println("\nSet Throttle for Motor 1\n");
            //     Serial.println("Enter value (e.g., 1100):");
            //     while (!Serial.available());
            //     motor1_throttle = Serial.parseInt();
            //     // Konfirmasi nilai throttle yang dimasukkan
            //     if (motor1_throttle > 999 && motor1_throttle < 2001) {
            //       Serial.print("Motor 1 Throttle set to: "); 
            //       Serial.println(motor1_throttle);
            //     } else {
            //       Serial.println("Invalid input. Please enter a valid number.");
            //     }
            //     // Serial.print("Motor 1 Throttle set to: "); Serial.println(motor1_throttle);
            //     // runMotor1(motor1_throttle); // Fungsi untuk motor 1
            //     break;

            // case 20:
            //     Serial.println("\nSet Throttle for Motor 2\n");
            //     Serial.println("Enter value (e.g., 1200):");
            //     while (!Serial.available());
            //     motor2_throttle = Serial.parseInt();
            //     if (motor2_throttle > 999 && motor2_throttle < 2001) {
            //       Serial.print("Motor 2 Throttle set to: "); 
            //       Serial.println(motor2_throttle);
            //     } else {
            //       Serial.println("Invalid input. Please enter a valid number.");
            //     }
            //     // Serial.print("Motor 2 Throttle set to: "); Serial.println(motor2_throttle);
            //     // runMotor2(motor2_throttle); // Fungsi untuk motor 2
            //     break;

            // case 21:
            //     Serial.println("\nSet Throttle for Motor 3\n");
            //     Serial.println("Enter value (e.g., 1300):");
            //     while (!Serial.available());
            //     motor3_throttle = Serial.parseInt();
            //     if (motor3_throttle > 999 && motor3_throttle < 2001) {
            //       Serial.print("Motor 3 Throttle set to: "); 
            //       Serial.println(motor3_throttle);
            //     } else {
            //       Serial.println("Invalid input. Please enter a valid number.");
            //     }
            //     // Serial.print("Motor 3 Throttle set to: "); Serial.println(motor3_throttle);
            //     // runMotor3(motor3_throttle); // Fungsi untuk motor 3
            //     break;

            // case 22:
            //     Serial.println("\nSet Throttle for Motor 4\n");
            //     Serial.println("Enter value (e.g., 1400):");
            //     while (!Serial.available());
            //     motor4_throttle = Serial.parseInt();
            //     if (motor4_throttle > 999 && motor4_throttle < 2001) {
            //       Serial.print("Motor 4 Throttle set to: "); 
            //       Serial.println(motor4_throttle);
            //     } else {
            //       Serial.println("Invalid input. Please enter a valid number.");
            //     }
            //     // Serial.print("Motor 4 Throttle set to: "); Serial.println(motor4_throttle);
            //     // runMotor4(motor4_throttle); // Fungsi untuk motor 4
            //     break;
        }
    }
  }




void setup()
{
    // Setup blink led
    pinMode(LED_BUILTIN, OUTPUT);

  userInput();
  displayInstructions();

    // Wait for motor init
    for (size_t i = 0; i < 4000; i++) {
        motor0.sendCommand(0, false);
        motor1.sendCommand(0, false);
        motor2.sendCommand(0, false);
        motor3.sendCommand(0, false);
        delayMicroseconds(1'000);
    }
}

void loop()
{
    uint32_t loopCycleStart = micros();
    userInput();

    // Set throttle
    motor0.sendThrottle(input_throttle, false);
    motor1.sendThrottle(throttle, false);
    motor2.sendThrottle(throttle, false);
    motor3.sendThrottle(throttle, false);

    // // Decrease throttle if max reached
    // if (throttle >= 1999) {
    //     throttleChange = -1;
    // }

    // Increase or decrease throttle
    // if (counter % 20 == 0 && !(throttle == 0 && throttleChange == -1)) {
    //     throttle += throttleChange;
        
    // }

    // Blink the LED
    if (counter % LOOP_HZ == 0) {
        digitalWrite(LED_BUILTIN, ledState ^= 1);
        displayInstructions();
        Serial.println(input_throttle);

    }

    ++counter;
    delayMicroseconds((1'000'000 / LOOP_HZ) - (micros() - loopCycleStart));
}


