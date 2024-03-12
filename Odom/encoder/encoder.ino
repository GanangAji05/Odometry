
#define encoder1 2  // Motor 1 Interrupt Pin - INT 0
#define encoder2 3  // Motor 2 Interrupt Pin - INT 1

unsigned long currentMillis;
unsigned long previousArmMillis;
unsigned long previousMillis;

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

void setup() {
  Serial.begin(9600);
  pinMode(encoder1, INPUT_PULLUP);    // encoder pins
  pinMode(encoder2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder1), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2), doEncoderB, CHANGE); 
}

void doEncoderA() {
    encoder0Pos++;
}

void doEncoderB() {
    encoder1Pos++;
}

void loop() {
  currentMillis = millis();   // bookmark the time 
  if (currentMillis - previousMillis >= 10) {  // start timed loop for everything else
         previousMillis = currentMillis;
         Serial.print(encoder0Pos);
         Serial.print(",");
         Serial.println(encoder1Pos);
  }
}
