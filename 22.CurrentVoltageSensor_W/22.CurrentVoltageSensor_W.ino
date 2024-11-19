#define voltageSensorPin 22
#define currentSensorPin 23

// Current Voltage Sensor
int voltageBit,currentBit = 0;
float voltageVolt, currentVolt, voltage, current = 0;

void setup() {
    Serial.begin(115200); // Memulai komunikasi serial dengan baud rate 115200
    delay(1000);          // Menunggu serial siap
    pinMode(voltageSensorPin, INPUT);
    pinMode(currentSensorPin, INPUT);
}

void loop() {
    voltageBit = analogRead(voltageSensorPin); // Membaca nilai analog dari pin A0
    currentBit = analogRead(currentSensorPin); // Membaca nilai analog dari pin A0
    voltageVolt = (float)voltageBit/1023.0*3.3;
    currentVolt = (float)currentBit/1023.0*3.3;
    voltage = voltageVolt*35.34/3.3;  //maximum 30V, calibration value: 35.34
    current = currentVolt*90;      //maximum 90A, not yet calibrated
    // Menampilkan hasil pembacaan di Serial Monitor
    Serial.print("currentBit: "); Serial.println(currentBit); Serial.print(" || voltagebit: "); Serial.println(voltageBit);
    Serial.print("currentVolt: "); Serial.println(currentVolt); Serial.print(" || voltageVoltt: "); Serial.println(voltageVolt);
    
    delay(1000); // Menunggu 1 detik sebelum membaca ulang
}
