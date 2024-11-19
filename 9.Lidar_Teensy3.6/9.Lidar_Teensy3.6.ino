// Teensy 3.6 with TF-Luna
// Copyright (C) 2021 https://www.roboticboat.uk
// 6a3dbdfa-410a-4b05-bd61-4ea40bdf0f83
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
// These Terms shall be governed and construed in accordance with the laws of 
// England and Wales, without regard to its conflict of law provisions.


// TF-Luna (i2c connections) 
// Pin 1 - 5v  - Red    (on supplied cable)   Teensy 3.6 pin Vin
// Pin 2 - SDA - Blue   (on supplied cable)   Teensy 3.6 pin 18
// Pin 3 - SCL - Yellow (on supplied cable)   Teensy 3.6 pin 19
// Pin 4 - GND - Green  (on supplied cable)   Teensy 3.6 pin GND
// Pin 5 - GND - White  (on supplied cable)   Teensy 3.6 pin GND
// Pin 6 - Int - Black  (on supplied cable)   no used (yet)

 
#include <Wire.h>

// Address of the TF-Luna on i2C (default address)
#define _i2cAddress 0x10

// TF-Luna registers
#define DISTANCE_Register   0x00
#define VERSION_Register    0x0A
#define SAVE_Register       0x20
#define REBOOT_Register     0x21
#define ADDRESS_Register    0x22
#define MODE_Register       0x23

#define ONE_BYTE    1
#define THREE_BYTES 3
#define SIX_BYTES   6

byte byteHigh;
byte byteLow;
uint8_t versionRevision;
uint8_t versionMinor;
uint8_t versionMajor;
uint8_t TriggerContinuous;

uint16_t distance = 0;
uint16_t strength = 0;
uint16_t celsius = 0;
uint16_t nReceived = 0;

void setup()
{
  // Initialize the serial port to the User
  // Set this up early in the code, so the User sees all messages
  Serial.begin(9600);

  // Start the i2c network
  Wire1.begin();

  // Wait for i2c to setup
  delay(100);

  // Get TF-Luna version
  if (getVersion(versionMajor, versionMinor, versionRevision))
  {
    Serial.print("i2c address 0x");
    if (_i2cAddress < 16) {Serial.print("0");}
    Serial.println(_i2cAddress,HEX);
    Serial.println("TF-Luna LiDAR Module");
    Serial.print("Version\t");
    Serial.print(versionMajor, HEX);
    Serial.print(".");
    Serial.print(versionMinor, HEX);
    Serial.print(".");
    Serial.println(versionRevision, HEX);
  }

  // Continous ranging mode or trigger mode?
  if (getMode(TriggerContinuous))
  {
    switch (TriggerContinuous){
      case 0:
        Serial.println("Continuous ranging mode");
        break;
      case 1:
        Serial.println("Trigger mode");
        break;
      default:
        Serial.println("Unknown mode response");
    }
  }

  // Uncomment code to change address, then comment again
  // Only required if having more than one TF-Luna on network
  // Change the i2c address of the module
  //if (!changeAddress(0x08))
  //{
  //  Serial.print("Script stopped");
  //  while(1);
  //}

}

void loop()
{
  // Read the TF-Luna
  if (getMeasuredData(distance, strength, celsius))
  {
    Serial.print("Distance ");
    Serial.print(distance);
    Serial.print(" cm,\tStrength ");
    Serial.print(strength);
    Serial.print(",\tCelsius ");
    Serial.print(celsius);
    Serial.println(" ºC");
  }
  else
  {
    Serial.println("Error getting data");
    Scani2cNetwork();
  }

  // Wait 100ms
  delay(100);
}

bool getMeasuredData(uint16_t &dist, uint16_t &stren, uint16_t &cels)
{
  // NOTE that pin 6 is not checked. This means the distance measurement
  // might have a read/write conflict and thus give errornous answer.
  // At the moment I have not experience this. But ideally the code should check.
  
  // Begin communication with TF-Luna
  Wire1.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire1.write(DISTANCE_Register);

  // End transmission
  Wire1.endTransmission();
  
  // Request 6 bytes from TF-Luna
  nReceived = Wire1.requestFrom(_i2cAddress , SIX_BYTES);

  // Something has gone wrong
  if (nReceived != SIX_BYTES) return false;

  // Distance in centimeters (cm)
  byteLow = Wire1.read();
  byteHigh = Wire1.read(); 
  dist = ((uint16_t)byteHigh <<8) + (uint16_t)byteLow;

  // Signal Strength 0 - &FFFF
  byteLow = Wire1.read();
  byteHigh = Wire1.read(); 
  stren = ((uint16_t)byteHigh <<8) + (uint16_t)byteLow;

  // Read the temperature of the unit
  // Start around room temperature and goes to around 50ºC
  byteLow = Wire1.read();
  byteHigh = Wire1.read(); 
  cels = ((uint16_t)byteHigh <<8) + (uint16_t)byteLow;

  // Rescale celsius
  cels = cels/100;

  // Is all OK?
  return true;
}

bool getVersion(uint8_t &versionMajor, uint8_t versionMinor, uint8_t versionRevision)
{
  // Begin communication with TF-Luna
  Wire1.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire1.write(VERSION_Register);

  // End transmission
  Wire1.endTransmission();
  
  // Request 6 bytes from TF-Luna
  nReceived = Wire1.requestFrom(_i2cAddress , THREE_BYTES);

  // Something has gone wrong
  if (nReceived != THREE_BYTES) return false;

  // Read the values
  versionRevision = Wire1.read();
  versionMinor = Wire1.read(); 
  versionMajor  = Wire1.read(); 

  // Is all OK?
  return true;  
}

bool getMode(uint8_t TriggerContinuous)
{
  // Begin communication with TF-Luna
  Wire1.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire1.write(MODE_Register);

  // End transmission
  Wire1.endTransmission();
  
  // Request 1 byte from TF-Luna
  nReceived = Wire1.requestFrom(_i2cAddress , ONE_BYTE);

  // Something has gone wrong
  if (nReceived != ONE_BYTE) return false;

  // Read the values
  TriggerContinuous = Wire1.read();

  // Is all OK?
  return true;  
}

bool changeAddress(uint8_t newi2cAddress)
{
  // Set the i2c address of the Module
  // The 7 bit i2c address must end with a 0. (even numbers please)
  // Range from 0x08, 0x10, .... 0x76

  Serial.print("Request to change i2c address from 0x");
  Serial.print(_i2cAddress, HEX);
  Serial.print(" to 0x");
  Serial.println(newi2cAddress, HEX);

  // Check the last bit is zero
  if (newi2cAddress & (byte)1)
  {
    // If this is true the last a 1.
    Serial.println("NOT a even number");
    return false;  
  }
  
  // Check the new address is different
  if (newi2cAddress == _i2cAddress)
  {
    Serial.println("SAME ADDRESS!");
    return false;
  }

  // Check the new address is allowed
  if (newi2cAddress < 0x08 || newi2cAddress > 0x76)
  {
    Serial.println("NOT WITHIN RANGE 0x08 to 0x76");
    return false;
  }

  // Begin communication with TF-Luna
  Wire1.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire1.write(ADDRESS_Register);

  // Write the new i2c address
  Wire1.write(newi2cAddress);

  // End transmission
  Wire1.endTransmission();

  // Give some time to update
  delay(100);
  
  // Now we save the configuation 
  // So that if we unplug, TF-Luna will remember the new address
  // Remember to change _i2cAddress in code
  saveConfiguation(newi2cAddress);
  
  return true;
}

void saveConfiguation(uint8_t myAddress)
{
  // Begin communication with TF-Luna
  Wire1.beginTransmission(myAddress);

  // Tell register you want some data
  Wire1.write(SAVE_Register);

  // Write 0x01 to save configuation
  Wire1.write(0x01);

  // End transmission
  Wire1.endTransmission(); 
}

void reboot()
{
  // Begin communication with TF-Luna
  Wire1.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire1.write(REBOOT_Register);

  // Write 0x02 to reboot
  Wire1.write(0x02);

  // End transmission
  Wire1.endTransmission(); 
}

void Scani2cNetwork()
{  
  // Allocate memory
  uint8_t tryAddress;
  uint8_t response;
  
  // Loop over the I2C devices
  for(tryAddress = 1; tryAddress < 127; tryAddress++ ) 
  {
    // Did the I2C device acknowledge a transmission 
    Wire1.beginTransmission(tryAddress);
     
    // Ends a transmission to a slave device that was begun by beginTransmission()
    response = Wire1.endTransmission();

    // response = 0: success
    if (response == 0)
    {
      // Have found an I2C device
      Serial.print("Device found at 0x");
        
      // Print a number as an ASCII-encoded hexadecimal
      if (tryAddress < 16) {Serial.print("0");}
      Serial.println(tryAddress, HEX);
    }    
  }

  // 3 second wait
  delay(3000);
}