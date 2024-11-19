// HEXACOPTER with Remote Control

#include <PulsePosition.h>
#define receiverPin 9
#define mot1Pin 2 //2
#define mot2Pin 3 //3
#define mot3Pin 4 //4
#define mot4Pin 5 //5
#define mot4Pin 5 //5
#define mot4Pin 5 //5


uint32_t LoopTimer;

PulsePositionInput ReceiverInput(RISING);
float ReceiverValue[]={0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber=0;
float InputThrottle;
void read_receiver(void){
  ChannelNumber = ReceiverInput.available();
  if (ChannelNumber > 0) {
  for (int i=1; i<=ChannelNumber;i++){
    ReceiverValue[i-1]=ReceiverInput.read(i);
    }
  }
}
void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT); 
  digitalWrite(13, HIGH);
  ReceiverInput.begin(receiverPin);
  analogWriteFrequency(mot1Pin, 250);
  analogWriteFrequency(mot2Pin, 250);
  analogWriteFrequency(mot3Pin, 250);
  analogWriteFrequency(mot4Pin, 250);
  analogWriteResolution(12);
  delay(250);
  while (ReceiverValue[2] < 1005 ||
    ReceiverValue[2] > 1050) {
      read_receiver();
      Serial.println(ReceiverValue[2]);
      delay(4);
    }
  LoopTimer=micros();
}
void loop() {
  read_receiver();
  InputThrottle=ReceiverValue[2];
  // analogWrite(mot1Pin,1.024*InputThrottle);
  analogWrite(mot2Pin,1.024*InputThrottle);
  // analogWrite(mot3Pin,1.024*InputThrottle);
  // analogWrite(mot4Pin,1.024*InputThrottle);
  Serial.println(InputThrottle);
  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
}