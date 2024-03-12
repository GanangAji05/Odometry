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

float encoder0Diff;
float encoder1Diff;

float encoder0Error;
float encoder1Error;

float encoder0Prev;
float encoder1Prev;

void setup() {
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);

  Serial.begin(115200);
  pinMode(encoder1, INPUT_PULLUP);    // encoder pins
  pinMode(encoder2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder1), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2), doEncoderB, CHANGE); 

  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-200, 200);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);
  PID2.SetOutputLimits(-200, 200);
  PID2.SetSampleTime(10);
  
}

void loop() {
  currentMillis = millis();

  if (currentMillis - previousMillis >= 10 ){
    previousMillis = currentMillis;

    if (Serial.available()>0){
      char c = Serial.read();

      if (c == 'a'){
        demand1 = 10;
        demand2 = 10;
      }
      else if (c == 'b') {
        demand1 = 0.25;
        demand2 = 0.25;
      }
      else if (c == 'c') {
        demand1 = 0.1;
        demand2 = 0.1;
      }
      else if (c == 'd') {
        demand1 = 0.145;
        demand2 = -0.145;
      }
      else if (c == 'e') {
        demand1 = -0.145;
        demand2 = 0.145;
      }
      else if (c == 'z') {
        demand1 = 0;
        demand2 = 0;
      }
    }
/*    rata rata jumlah encoder per 1 meter 800 tick
      8 per 10 millisecond atau 1 meter/deteik 

      Jarak antara roda 60mm, setengahnya 30mm
      Circumference dari 60mm (Pi x D) adalah 190mm(dibulatkan), Setiap roda butuh penggerak setengah dari Circumference, yang nilainya 95mm 1 belok radian,
      Setiap roda butuh penggerak 95/Pi = 30mm (per detik untuk 1 rad/s)
*/

    encoder0Diff = encoder0Pos - encoder0Prev;
    encoder1Diff = encoder1Pos - encoder1Prev;

    encoder0Error = (demand1*10) - encoder0Diff;
    encoder1Error = (demand2*10) - encoder1Diff;

    encoder0Prev = encoder0Pos;
    encoder1Prev = encoder1Pos;

    Setpoint1 = demand1*10;
    Input1 = encoder0Diff;
    PID1.Compute();

    Setpoint2 = demand2*10;
    Input2 = encoder1Diff;
    PID2.Compute();

    Serial.print(Setpoint2);
    Serial.print(" , ");
    Serial.print(encoder1Diff);
    Serial.print(" , ");
    Serial.print(encoder1Pos);
    Serial.print(" , ");
    Serial.println(Output2);

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
    // Baca status arah putaran motor
    int motor1Direction = digitalRead(motor1A) ^ digitalRead(motor1B);
    
    // Sesuaikan penambahan atau pengurangan nilai encoder sesuai dengan arah putaran motor
    if (motor1Direction == HIGH) {
        encoder0Pos = encoder0Pos - 1;
    } else {
        encoder0Pos = encoder0Pos + 1;
    }
}

void doEncoderB() {
    // Baca status arah putaran motor
    int motor2Direction = digitalRead(motor2A) ^ digitalRead(motor2B);
    
    // Sesuaikan penambahan atau pengurangan nilai encoder sesuai dengan arah putaran motor
    if (motor2Direction == HIGH) {
        encoder1Pos = encoder1Pos - 1;
    } else {
        encoder1Pos = encoder1Pos + 1;
    }
}
