#include <SoftwareSerial.h>
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
SoftwareSerial gps_port(RXPin, TXPin);

void setup()
{
  Serial.begin( 115200 );
  gps_port.begin( GPSBaud );
}

void loop()
{
  if (gps_port.available())
    Serial.write( gps_port.read() );
}