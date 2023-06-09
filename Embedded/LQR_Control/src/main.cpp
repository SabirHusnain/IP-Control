#include <Arduino.h>

// Macros
#define __clock_Wise 0                // Motor Direction --> Clockwise --> 0
#define __anti_Clock_Wise 1           // Motor Direction --> Anti Clockwise --> 1
#define __motor_Voltage 15.0          // Nominal Motor Voltage --> 24V
#define __encoder_Resolution 0.031416 // Encoder Resolution --> 1.8 degrees --> 200/360
#define __pwm_Resolution 12           // PWM Channel Resolution --> 12-bit
#define __pwm_Frequency 4000          // PWM Channel Frequency --> 4KHz

// Encoder Pins
#define enc_1 12 // Encoder Output A
#define enc_2 14 // Encoder Output B

// Motor Control Pins
#define pwm 32  // PWM Pin
#define pwmCh 0 // PWM Channel
#define in_1 27 // Motor Direction Pin 1
#define in_2 26 // Motor Direction Pin 2

// Variables
double currTime = 0.0, prevTime = 0.0, delTime = 0.0; // Time Variables
double setupTime = 0.0;                               // Time Required by Setup Function

// Control Parameters
volatile long pulses = 0;                          // Total Pulses
double currPos = 0.0, prevPos = 0.0, delPos = 0.0; // Position Variables (Angular)
double vel = 0.0;
double currVel = 0; // Velocity Variables (Angular)

bool dir = __clock_Wise;
float V = 0.0;
int myPwm = 0;                 // Inputs (Direction, VOltage, PWM)
double u = 0.0;                // Control Signal)
float K[] = {13.0931, 1.6435}; // Controller Gains

// Functions Declerations
void encPulse();            // ISR
void motorInput(bool, int); // Motor Input

void setup()
{
  pinMode(enc_1, INPUT_PULLUP);
  pinMode(enc_2, INPUT_PULLUP);

  // Configuring PWM Channel
  ledcAttachPin(pwm, pwmCh);
  ledcSetup(pwmCh, __pwm_Frequency, __pwm_Resolution);

  pinMode(in_1, OUTPUT);
  pinMode(in_2, OUTPUT);

  Serial.begin(115200);

  // Interrupt Attach
  attachInterrupt(digitalPinToInterrupt(enc_1), encPulse, RISING);

  // Logging Setup Time
  motorInput(0, 4095);
  delay(100);
  setupTime = micros();
}

void loop()
{
  // currTime = micros() - setupTime;
  // delTime = currTime - prevTime;
  // prevTime = currTime;
  // currPos = pulses * __encoder_Resolution;
  // delPos = currPos - prevPos;
  // prevPos = currPos;
  // currVel = ((delPos / delTime) * 1e6);
  // // Serial.print(currPos);
  // // Serial.print(" , ");
  // // Serial.println(currVel);
  // // Serial.print(K[0]);
  // u = -K[0] * currPos - K[1] * currVel; // u=-Kx
  // if (abs(u) > __motor_Voltage)
  //   V = __motor_Voltage; // Voltage Limiter
  // else
  //   V = abs(u);

  // myPwm = map(V, 0, __motor_Voltage, 0, pow(2, __pwm_Resolution) - 1); // Voltage to PWM Conversion

  // if (u < 0)
  //   dir = __anti_Clock_Wise;
  // else
  //   dir = __clock_Wise;

  // motorInput(dir, myPwm); // Start Motorٖ

  // // Data Logging
  // Serial.print(currPos);
  // Serial.print(",");
  // Serial.print(currVel);
  // Serial.print(",");
  // Serial.print(u);
  // Serial.print(",");
  // Serial.print(V);
  // Serial.print(",");
  // Serial.print(myPwm);
  // Serial.print(",");
  // Serial.println(dir);
}

// ISR
void encPulse()
{
  if (digitalRead(enc_2) == LOW)
    pulses++;
  else
    pulses--;
}

// Motor Control Function
void motorInput(bool dir, int dutyCycle)
{
  digitalWrite(in_1, dir);
  digitalWrite(in_2, !dir);
  // analogWrite(25,duTy)
  ledcWrite(pwmCh, dutyCycle);
}