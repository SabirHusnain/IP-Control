#include <Arduino.h>

// Macros
#define __clock_Wise 0
#define __anti_Clock_Wise 1
#define __square_Time_Period 2
#define __stop_Time 10
#define __encoder_Resolution 1.8
#define __dps_2_rpm 0.166667
#define __pwm_Frequency 4000
#define __pwm_Resolution 12

// Encoder
#define enc_1 25
#define enc_2 27

// Motor Control
#define pwm 12
#define pwmChannel 0
#define in_1 3
#define in_2 13

// Variables
volatile long pulses = 0;
long pos = 0, prevPos = 0, delPos = 0;
bool motorDir, inputDir = __clock_Wise;
double currTime = 0.0, prevTime = 0.0, delTime = 0.0;
double squareTime = 0.0, setupTime;
String data;

// Functions
void encPulse();
void squareInput(bool, int);

void setup()
{
  pinMode(enc_1, INPUT_PULLUP);
  pinMode(enc_2, INPUT_PULLUP);
  pinMode(in_1, OUTPUT);
  pinMode(in_2, OUTPUT);
  ledcSetup(pwmChannel, __pwm_Frequency, __pwm_Resolution);
  ledcAttachPin(pwm, pwmChannel);
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(enc_1), encPulse, RISING);
  setupTime = micros();
}

void loop()
{
  currTime = (double)(micros() - setupTime);
  delTime = (double)(currTime - prevTime);
  prevTime = (double)currTime;
  pos = pulses;
  delPos = pos - prevPos;
  prevPos = pos;
  data = String(prevTime) + ',' + String(delPos) + ',' + String(delTime) + ',' + String(inputDir);
  Serial.println(data);
  squareInput(inputDir, 4095);

  if ((millis() - (setupTime / 1000) - squareTime) / 1000 >= (0.5 * __square_Time_Period))
  {
    inputDir = !inputDir;
    squareTime = millis() - setupTime / 1000;
  }

  if ((millis() - setupTime / 1000) / 1000 >= __stop_Time)
  {
    digitalWrite(in_1, LOW);
    digitalWrite(in_2, LOW);
    // Serial.println(millis() - setupTime * 0.001);
    while (1)
      ;
  }
  delay(5);
}

void encPulse()
{
  if (digitalRead(enc_2) == LOW)
    pulses++;
  else
    pulses--;
}

void squareInput(bool dir, int dutyCycle)
{
  digitalWrite(in_1, dir);
  digitalWrite(in_2, !dir);
  ledcWrite(pwm, dutyCycle);
}