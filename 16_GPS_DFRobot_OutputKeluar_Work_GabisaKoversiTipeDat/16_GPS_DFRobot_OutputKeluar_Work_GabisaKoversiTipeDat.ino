 /*!
  * @file  getGNSS.ino
  * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @license The MIT License (MIT)
  * @author ZhixinLiu(zhixin.liu@dfrobot.com)
  * @version V0.1
  * @date 2022-08-15
  * @url https://github.com/dfrobot/DFRobot_GNSS
  */

#include "DFRobot_GNSS.h"

//DFRobot_GNSS_I2C gnss(&Wire ,GNSS_DEVICE_ADDR);

#ifdef ESP_PLATFORM
  // ESP32 user hardware uart
  // RX io16
  // TX io17
  // DFRobot_GNSS_UART gnss(&Serial8 ,9600);
#else
  // Arduino user software uart
  // RX io10
  // TX io11
  SoftwareSerial mySerial(35 ,34);
  DFRobot_GNSS_UART gnss(&mySerial ,9600);
  // DFRobot_GNSS_UART gnss(&Serial8 ,9600);
#endif

// Titik referensi (misalnya, titik asal)
const float latRef = 7.283996;  // Latitude referensi
const float lonRef = 112.796340; // Longitude referensi
const float k = 111320.0;      // Faktor konversi untuk derajat ke meter
float lat = 7.283996;
float lon = 112.796340;
float x = 0;
float y = 0;

// Fungsi konversi lat, lon ke x, y dalam meter
void latLonToXY(float lat, float lon, float x, float y) {
  float latMid = (lat + latRef) / 2.0;  // Rata-rata latitude
  x = (lon - lonRef) * k * cos(radians(latMid));
  y = (lat - latRef) * k;
}


void setup() 
{
  Serial.begin(115200);
  while(!gnss.begin()){
    Serial.println("NO Deivces !");
    delay(1000);
  }

  gnss.enablePower();      

/** Set the galaxy to be used
 *   eGPS              USE gps
 *   eBeiDou           USE beidou
 *   eGPS_BeiDou       USE gps + beidou
 *   eGLONASS          USE glonass
 *   eGPS_GLONASS      USE gps + glonass
 *   eBeiDou_GLONASS   USE beidou +glonass
 *   eGPS_BeiDou_GLONASS USE gps + beidou + glonass
 */
  gnss.setGnss(eGPS_BeiDou_GLONASS);


  // gnss.setRgbOff();
  gnss.setRgbOn();
  // gnss.disablePower();      
}

void loop()
{
  sTim_t utc = gnss.getUTC();
  sTim_t date = gnss.getDate();
  sLonLat_t latGPS = gnss.getLat();
  sLonLat_t lonGPS = gnss.getLon();
  // lat = (float)(latGPS);
  // lon = (float)(lonGPS);

  
  double high = gnss.getAlt();
  uint8_t starUserd = gnss.getNumSatUsed();
  double sog = gnss.getSog();
  double cog = gnss.getCog();

  Serial.println("");
  Serial.print(date.year);
  Serial.print("/");
  Serial.print(date.month);
  Serial.print("/");
  Serial.print(date.date);
  Serial.print("/");
  Serial.print(utc.hour);
  Serial.print(":");
  Serial.print(utc.minute);
  Serial.print(":");
  Serial.print(utc.second);
  Serial.println();
  // Serial.println((char)lat.latDirection);
  // Serial.println((char)lon.lonDirection);

  // Serial.print("lat DDMM.MMMMM = ");
  // Serial.println(lat.latitude, 5);
  // Serial.print(" lon DDDMM.MMMMM = ");
  // Serial.println(lon.lonitude, 5);
  // Serial.print("lat degree = ");
  // Serial.println(lat.latitudeDegree,6);
  // Serial.print("lon degree = ");
  // Serial.println(lon.lonitudeDegree,6);

  Serial.print("star userd = ");
  Serial.println(starUserd);
  Serial.print("alt high = ");
  Serial.println(high);
  Serial.print("sog =  ");
  Serial.println(sog);
  Serial.print("cog = ");
  Serial.println(cog);
  Serial.print("gnss mode =  ");
  Serial.println(gnss.getGnssMode());

  latLonToXY(lat, lon, x, y);
    // Tampilkan hasil
  Serial.print("Koordinat x: ");
  Serial.println(x);
  Serial.print("Koordinat y: ");
  Serial.println(y);

  delay(1000);
}