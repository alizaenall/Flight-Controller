#define LIDAR Serial5  // Define the Serial1 for TF-Luna communication

uint8_t recvBuffer[9];  // Buffer to store received data
int distance;           // Distance measured by the LiDAR
int strength;           // Signal strength

void setup() {
  // Initialize serial communication
  Serial.begin(115200);       // Serial monitor
  LIDAR.begin(115200);        // LiDAR UART
  Serial.println("TF-Luna LiDAR Test");
}

void loop() {
  // Check if data is available from LiDAR
  if (LIDAR.available()) {
    // Read one byte at a time into the buffer
    if (LIDAR.readBytes(recvBuffer, 9) == 9) {  // TF-Luna sends data in packets of 9 bytes
      // Verify if the packet is valid (check the frame header)
      if (recvBuffer[0] == 0x59 && recvBuffer[1] == 0x59) {
        distance = recvBuffer[2] + (recvBuffer[3] << 8);  // Calculate distance (low byte + high byte)
        strength = recvBuffer[4] + (recvBuffer[5] << 8);  // Calculate signal strength

        // Print the distance and signal strength
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.print(" cm\t");
        Serial.print("Signal Strength: ");
        Serial.println(strength);
      }
    }
  }
}
