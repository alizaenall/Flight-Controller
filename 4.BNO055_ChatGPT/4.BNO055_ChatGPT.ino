// WORKED - JANGAN DIUBAH-UBAH !!!

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Use Wire1 for I2C2 (pins 16: SCL, 17: SDA)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

void setup() {
  
  Serial.begin(9600);
  
  // Initialize the Wire1 object without specifying pins (Teensy automatically assigns the correct ones for I2C2)
  Wire1.begin();

  // Initialize the BNO055 sensor
  if (!bno.begin()) {
    Serial.print("Couldn't find sensor BNO055 ... Check connection!");
    while (1);

  }

  // Optional: Enable external crystal for better precision
  bno.setExtCrystalUse(true);
  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delay(4000);
  digitalWrite(2, LOW);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);

  Serial.println("Sensor ready!");
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);

  // Output orientation data (Euler angles)
  Serial.print("Orientation X: "); Serial.print(event.orientation.x);
  Serial.print(" Y: "); Serial.print(event.orientation.y);
  Serial.print(" Z: "); Serial.println(event.orientation.z);

  delay(100);
}
