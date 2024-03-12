#include <PID_v1.h>

//encoder
#define encoder1 2  // Motor 1 Interrupt Pin - INT 0
#define encoder2 3  // Motor 2 Interrupt Pin - INT 1

//motor
#define motor1A 4
#define motor1B 5
#define motor2A 6
#define motor2B 7

double Pk1 = 1;
double Ik1 = 0;
double Dk1 = 0.01;

double Setpoint1, Output1, Input1, Output1a;
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1, Dk1, DIRECT);

double Pk2 = 1;
double Ik2 = 0;
double Dk2 = 0.01;

double Setpoint2, Output2, Input2, Output2a;
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2, Dk2, DIRECT);

float demand1;
float demand2;

unsigned long currentMillis;
unsigned long previousMillis;

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

void setup() {
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);

  Serial.begin(9600);
  pinMode(encoder1, INPUT_PULLUP);    // encoder pins
  pinMode(encoder2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder1), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2), doEncoderB, CHANGE); 

  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-100, 100);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);
  PID2.SetOutputLimits(-100, 100);
  PID2.SetSampleTime(10);
  
}

void loop() {
  currentMillis = millis();

  if (currentMillis - previousMillis >= 10 ){
    previousMillis = currentMillis;

    if (Serial.available()>0){
      char c = Serial.read();

      if (c == 'a'){
        demand1 = 1000;
        demand2 = 1000;
      }
      else if (c == 'z') {
        demand1 = 0;
        demand2 = 0;
      }
    }

    Serial.print(encoder0Pos);
    Serial.print(" , ");
    Serial.print(encoder1Pos);
    Serial.print(" , ");
    Serial.print(Output1a);
    Serial.print(" , ");
    Serial.println(Output2a);

    Setpoint1 = demand1;
    Input1 = encoder0Pos;
    PID1.Compute();

    Setpoint2 = demand2;
    Input2 = encoder1Pos;
    PID2.Compute();

    //motor
    if (Output1 > 0){
      Output1a = abs(Output1);
      analogWrite(motor2A, Output1a);
      analogWrite(motor2B, 0);
    }
    else if (Output1 < 0) {
      Output1a = abs(Output1);
      analogWrite(motor2B, Output1a);
      analogWrite(motor2A, 0);
    }
    else{
      analogWrite(motor2B, 0);
      analogWrite(motor2A, 0);
    }

    //motor 2
    if (Output2 > 0){
      Output2a = abs(Output2);
      analogWrite(motor1B, Output2a);
      analogWrite(motor1A, 0);
    }
    else if (Output2 < 0) {
      Output2a = abs(Output2);
      analogWrite(motor1A, Output2a);
      analogWrite(motor1B, 0);
    }
    else{
      analogWrite(motor1A, 0);
      analogWrite(motor1B, 0);
    }
  }

}

void doEncoderA() {
     encoder0Pos = encoder0Pos + 1;

}

void doEncoderB() {
     encoder1Pos = encoder1Pos + 1;

}
