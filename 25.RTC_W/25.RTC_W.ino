#include <TimeLib.h>

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    // Tunggu hingga Serial Monitor terbuka (khusus untuk Teensy)
  }

  // Periksa apakah RTC memiliki waktu valid
  if (Teensy3Clock.get()) {
    setSyncProvider(Teensy3Clock.get); // Sinkronkan waktu dengan RTC
    if (timeStatus() == timeSet) {
      Serial.println("RTC time successfully synced!");
    } else {
      Serial.println("RTC time is invalid.");
    }
  } else {
    Serial.println("RTC not initialized, setting default time...");
    setTime(10, 15, 30, 16, 11, 2024); // Atur waktu default jika RTC belum diatur
    Teensy3Clock.set(now());           // Sinkronkan RTC dengan waktu default
    Serial.println("Default time set to RTC.");
  }

  // Tampilkan waktu saat ini
  printCurrentTime();
}

void loop() {
  // Perbarui dan tampilkan waktu setiap detik
  printCurrentTime();
  delay(1000);
}

void printCurrentTime() {
  Serial.print("Current Time: ");
  Serial.print(hour());
  Serial.print(":");
  Serial.print(minute());
  Serial.print(":");
  Serial.print(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print("/");
  Serial.print(month());
  Serial.print("/");
  Serial.println(year());
}
