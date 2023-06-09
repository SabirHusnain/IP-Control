#include <Arduino.h>

// Macros
#define __clock_Wise 0
#define __anti_Clock_Wise 1
#define __square_Time_Period 2
#define __stop_Time 10
#define __encoder_Resolution 1.8
#define __dps_2_rpm 0.16667

// Encoder
#define enc_1 12
#define enc_2 14

// Motor Control
#define pwm 32
#define in_1 27
#define in_2 26

// Variables
volatile long pulses = 0;
volatile long prevPos = 0, delPos = 0;
volatile double vel = 0;
double vel_1 = 0;
bool motorDir, inputDir = __clock_Wise;
volatile double currTime = 0.0, prevTime = 0.0, delTime = 0.0;
double squareTime = 0.0, setupTime;
String data;

// Functions
void encPulse();
void squareInput(bool, float);

void setup()
{
  pinMode(enc_1, INPUT_PULLUP);
  pinMode(enc_2, INPUT_PULLUP);
  pinMode(pwm, OUTPUT);
  pinMode(in_1, OUTPUT);
  pinMode(in_2, OUTPUT);
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(enc_1), encPulse, RISING);
  setupTime = micros();
}

void loop()
{
  vel_1 = (vel * 1e6) * __encoder_Resolution * __dps_2_rpm;
  data = String(prevTime) + ',' + String(vel_1);
  Serial.println(data);
  // squareInput(inputDir, 1);

  // if ((millis() - (setupTime / 1000) - squareTime) / 1000 >= (0.5 * __square_Time_Period))
  // {
  //   inputDir = !inputDir;
  //   squareTime = millis() - setupTime / 1000;
  // }

  // if ((millis() - setupTime / 1000) / 1000 >= __stop_Time)
  // {
  //   digitalWrite(in_1, LOW);
  //   digitalWrite(in_2, LOW);
  //   while (1)
  //     ;
  // }
}

void encPulse()
{
  int temp = 0;
  if (digitalRead(enc_2) == LOW)
    temp = 1;
  else
    temp = -1;

  pulses += temp;
  currTime = micros();
  delTime = currTime - prevTime;
  delPos = prevPos - pulses;
  vel = delPos / delTime;
  prevPos = pulses;
  prevTime = currTime;
}

void squareInput(bool dir, float dutyCycle)
{
  digitalWrite(in_1, dir);
  digitalWrite(in_2, !dir);
  analogWrite(pwm, dutyCycle);
}