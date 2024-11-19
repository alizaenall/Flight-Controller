#include <Wire.h> //I2C Arduino Library
#define addr 0x1E //I2C Address for The HMC5883

void setup()
{
Serial.begin(9600);
Wire.begin();
Wire.beginTransmission(addr);
Wire.write(0x02);
Wire.write(0x00); //Continuously Measure
Wire.endTransmission();
}

void loop()
{
int x,y,z; //triple axis data
Wire.beginTransmission(addr);
Wire.write(0x03);
Wire.endTransmission();
//Read the data
Wire.requestFrom(addr, 6);
if(6<=Wire.available())
{
  x = Wire.read()<<8; //MSB  x 
  x |= Wire.read(); //LSB  x
  z = Wire.read()<<8; //MSB  z
  z |= Wire.read(); //LSB z
  y = Wire.read()<<8; //MSB y
  y |= Wire.read(); //LSB y
}
// Show Values
Serial.print("X Value: ");
Serial.println(x);
Serial.print("Y Value: ");
Serial.println(y);
Serial.print("Z Value: ");
Serial.println(z);
Serial.println();
delay(500);
}