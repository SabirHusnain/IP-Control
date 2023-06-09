#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include "LQR.h"

// Macros
#define motors 2
#define __clock_Wise 0                 // Motor Direction --> Clockwise --> 0
#define __anti_Clock_Wise 1            // Motor Direction --> Anti Clockwise --> 1
#define __max_Motor_Voltage 24         // Nominal Motor Voltage --> 24V
#define __encoder_Resolution 0.031416F // Encoder Resolution --> 1.8 degrees --> 360/200
#define __pwm_Resolution 12            // PWM Channel Resolution --> 12-bit
#define __pwm_Frequency 4000           // PWM Channel Frequency --> 4KHz
#define __motorAngle_2_cartPos 0.01F   // Motor Angle to Cart Position Conversion

// Encoder Pins
#define enc_1_A 12 // Encoder Output A
#define enc_2_A 14 // Encoder Output B
#define enc_1_B 9  // Encoder Output A
#define enc_2_B 10 // Encoder Output B

// Motor Control Pins
#define pwm_Channel_A 0         // PWM Channel - Motor 1
#define pin_Pwm_A 1             // PWM Motor 1
#define pin_Clockwise_A 26      // Motor Pin 1 - Motor 1
#define pin_Anti_Clockwise_A 27 // Motor Pin 2 - Motor 1

#define pwm_Channel_B 1         // PWM Channel - Motor 2
#define pin_Pwm_B 3             // PWM Motor 2
#define pin_Clockwise_B 25      // Motor Pin 1 - Motor 2
#define pin_Anti_Clockwise_B 33 // Motor Pin 2 - Motor 2

const int enc_1[] = {enc_1_A, enc_1_B};
const int enc_2[] = {enc_2_A, enc_2_B};
const int pin_PWM[] = {pin_Pwm_A, pin_Pwm_B};
const int pwm_Channel[] = {pwm_Channel_A, pwm_Channel_B};
const int pin_Clockwise[] = {pin_Clockwise_A, pin_Clockwise_B};
const int pin_Anti_Clockwise[] = {pin_Anti_Clockwise_A, pin_Anti_Clockwise_B};

// Variables
double currTime = 0.0, prevTime = 0.0, delTime = 0.0; // Time Variables
double setupTime = 0.0;                               // Time Required by Setup Function

// Control Parameters
volatile long pulses[motors] = {0, 0};                                                          // Total Pulses
double currPos[motors] = {0.0, 0.0}, prevPos[motors] = {0.0, 0.0}, delPos[motors] = {0.0, 0.0}; // Tranlational Position Variables
double currTheta = 0.0, prevTheta = 0.0, delTheta = 0.0;                                        // Angular Position Variables

// float K[] = {-79.9803, -11.5174, -22.3607, -24.3234};
float K[] = {-59.6934, -8.7730, -8.9443, -17.5633};

#define __alpha 0.95F
bool first_Itr = true;
float accAngle, gyroAngle;

LQR myMotor[motors];

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_accel, *mpu_gyro;
sensors_event_t a, g;

void encPulse_A();
void encPulse_B();

void setup()
{
  for (int i = 0; i < motors; i++)
  {
    myMotor[i].configLQR(__max_Motor_Voltage,
                         pin_PWM[i], pwm_Channel[i],
                         __pwm_Resolution, __pwm_Frequency,
                         enc_1[i], enc_2[i],
                         pin_Clockwise[i], pin_Anti_Clockwise[i],
                         K);
    myMotor[i].beginLQR();
  }
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  attachInterrupt(digitalPinToInterrupt(enc_1[0]), encPulse_A, RISING);
  attachInterrupt(digitalPinToInterrupt(enc_1[1]), encPulse_B, RISING);
  mpu_accel = mpu.getAccelerometerSensor();
  mpu_gyro = mpu.getGyroSensor();
  setupTime = micros();
}

void loop()
{
  currTime = micros() - setupTime;
  delTime = currTime - prevTime;
  prevTime = currTime;

  for (int i = 0; i < motors; i++)
  {
    currPos[i] = pulses[i] * __encoder_Resolution * __motorAngle_2_cartPos;
    delPos[i] = currPos[i] - prevPos[i];
    prevPos[i] = currPos[i];
  }

  mpu_accel->getEvent(&a);
  mpu_gyro->getEvent(&g);

  accAngle = -atan2(a.acceleration.y, a.acceleration.x);
  gyroAngle += g.gyro.z * (delTime * 1e-3);

  currTheta = ((1 - __alpha) * (accAngle)) + (__alpha * gyroAngle); // Complementory Filter >> sensor fusion

  if (first_Itr)
  {
    currTheta = accAngle;
    first_Itr = false;
  }

  delTheta = currTheta - prevTheta;
  prevTheta = currTheta;

  for (int i = 0; i < motors; i++)
    myMotor[i].evaluateLQR(delTime, currPos[i], delPos[i], currTheta, delTheta);

  for (int i = 0; i < motors; i++)
    myMotor[i].runLQR();
}

void encPulse_A()
{
  if (digitalRead(enc_2[0]) == LOW)
    pulses[0]++;
  else
    pulses[0]--;
}

void encPulse_B()
{
  if (digitalRead(enc_2[1]) == LOW)
    pulses[1]++;
  else
    pulses[1]--;
}