#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Define GPS pins (change RX and TX pins according to your wiring)
SoftwareSerial gpsSerial(10, 11); // RX, TX

TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);    // Serial Monitor communication
  gpsSerial.begin(9600); // GPS module communication

  Serial.println("GPS Module is initializing...");
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());

    if (gps.location.isUpdated()) {
      Serial.print("Latitude: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 6);
      Serial.print("Altitude (meters): ");
      Serial.println(gps.altitude.meters());

      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value());
      Serial.print("HDOP: ");
      Serial.println(gps.hdop.value());

      Serial.println();
    }
  }
}
