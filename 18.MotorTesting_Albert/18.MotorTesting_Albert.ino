#define poles 12
#define rpmPin 7
unsigned long timer = 0;
volatile unsigned long counter = 0;
int RPM = 0;

void setup() {
  Serial.begin(57600);

  pinMode(rpmPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rpmPin), interrupt_rpm, RISING);
}

void loop() {
  if (counter > 99){
    float period = 0;
    period = ((float)(millis()-timer) / counter);
    RPM = 60000 / (period * poles / 2);
    Serial.print("prd ");
    Serial.print(period);
    Serial.print("ms  --  RPM =");
    Serial.println(RPM);
    timer = millis();
    counter = 0;
  }
  
}

FASTRUN void interrupt_rpm(){
  counter++;
  asm("DSB");
}