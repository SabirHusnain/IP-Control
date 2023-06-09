/*
======================================================
=============     Inverted Pendudlum     =============
======================================================
*/

#define __run_Control
// #define __calibrate_Sensor

// #define __lqr_Control
#define __pid_Control

#define __single_Motor_Control
// #define __dual_Motor_Control

// #define __Output_Readable_YPR
// #define __Output_Readable_Real_Acc
// #define __Output_Readable_World_Acc

#define __Output_Control_Cycle_Time

/*





=====================================================
=============          Control          =============
=====================================================
*/
#ifdef __run_Control

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#ifdef __pid_Control
#include "PID.h"
#endif

#ifdef __lqr_Control
#include "LQR.h"
#endif

// Macros
#define motors 2
#define __clock_Wise 0                 // Motor Direction --> Clockwise --> 0
#define __anti_Clock_Wise 1            // Motor Direction --> Anti Clockwise --> 1
#define __max_Motor_Voltage 17         // Nominal Motor Voltage --> 17V
#define __encoder_Resolution 0.031416F // Encoder Resolution --> 1.8 degrees --> 360/200
#define __pwm_Resolution 12            // PWM Channel Resolution --> 12-bit
#define __pwm_Frequency 4000           // PWM Channel Frequency --> 4KHz
#define __motorAngle_2_cartPos 0.005F  // Motor Angle to Cart Position Conversion

// Pins
#define mpu_Interrupt_Pin 19

// Encoder Pins
#define enc_1_A 13 // Encoder Output A
#define enc_2_A 12 // Encoder Output B
#define enc_1_B 27 // Encoder Output A
#define enc_2_B 14 // Encoder Output B

// Motor Control Pins
#define pwm_Channel_A 0        // PWM Channel - Motor 1
#define pin_Pwm_A 26           // PWM Motor 1
#define pin_Clockwise_A 2      // Motor Pin 1 - Motor 1
#define pin_Anti_Clockwise_A 4 // Motor Pin 2 - Motor 1

#define pwm_Channel_B 1         // PWM Channel - Motor 2
#define pin_Pwm_B 25            // PWM Motor 2
#define pin_Clockwise_B 16      // Motor Pin 1 - Motor 2
#define pin_Anti_Clockwise_B 17 // Motor Pin 2 - Motor 2

const int enc_1[] = {enc_1_A, enc_1_B};
const int enc_2[] = {enc_2_A, enc_2_B};
const int pin_PWM[] = {pin_Pwm_A, pin_Pwm_B};
const int pwm_Channel[] = {pwm_Channel_A, pwm_Channel_B};
const int pin_Clockwise[] = {pin_Clockwise_A, pin_Clockwise_B};
const int pin_Anti_Clockwise[] = {pin_Anti_Clockwise_A, pin_Anti_Clockwise_B};

// Variables

// MPU control/status vars
bool dmpReady = false;              // set true if DMP init was successful
uint8_t mpuIntStatus;               // holds actual interrupt status byte from MPU
uint8_t devStatus;                  // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;                 // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];             // FIFO storage buffer
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Time Variables
double currTime = 0.0, prevTime = 0.0, delTime = 0.0; // Time Variables
double setupTime = 0.0;                               // Time Required by Setup Function

// Control Variables
volatile long pulses[motors] = {0};                                              // Total Pulses
double currPos[motors] = {0.0}, prevPos[motors] = {0.0}, delPos[motors] = {0.0}; // Tranlational Position Variables
double currTheta = 0.0, prevTheta = 0.0, delTheta = 0.0;                         // Angular Position Variables

#ifdef __pid_Control
// PID Gains
double K[] = {151.6684, 1.5978e3, 6.1356}; // {K_p  K_i  K_d}
PID myMotor[motors];
#endif

#ifdef __lqr_Control
// LQR Gains
double K[] = {1, 1, 1, 1};
LQR myMotor[motors];
#endif

MPU6050 mpu;

void encPulse_A();

#ifdef __dual_Motor_Control
void encPulse_B();
#endif

void dmpDataReady();

void motorInputA(bool dir, int dutyCycle);
void motorInputB(bool dir, int dutyCycle);

void setup()
{
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  while (!Serial)
    ;

// join I2C bus
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(mpu_Interrupt_Pin, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* ----> Factory Offsets <----*/
  // mpu.setXGyroOffset(220);
  // mpu.setYGyroOffset(76);
  // mpu.setZGyroOffset(-85);
  // mpu.setZAccelOffset(1788);

  /* ----> My Offsets <----*/
  mpu.setXGyroOffset(-69);
  mpu.setYGyroOffset(50);
  mpu.setZGyroOffset(-103);
  mpu.setZAccelOffset(2158);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable ESP32 interrupt detection
    Serial.print(F("Enabling interrupt detection (ESP32 external interrupt "));
    Serial.print(digitalPinToInterrupt(mpu_Interrupt_Pin));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(mpu_Interrupt_Pin), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  attachInterrupt(digitalPinToInterrupt(enc_1[0]), encPulse_A, RISING);

#ifdef __dual_Motor_Control
  attachInterrupt(digitalPinToInterrupt(enc_1[1]), encPulse_B, RISING);
#endif

#ifdef __pid_Control
  for (int i = 0; i < motors; i++)
  {
    myMotor[i].configPID(__max_Motor_Voltage,
                         pin_PWM[i], pwm_Channel[i],
                         __pwm_Resolution, __pwm_Frequency,
                         enc_1[i], enc_2[i],
                         pin_Clockwise[i], pin_Anti_Clockwise[i],
                         K);
    myMotor[i].beginPID();
  }
#endif

#ifdef __lqr_Control
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
#endif

  setupTime = micros();
}

void loop()
{
  currTime = micros() - setupTime;
  delTime = currTime - prevTime;
  prevTime = currTime;

  // if programming failed, parse old values
  if (!dmpReady)
    ypr[3] = prevTheta;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#if defined(__Output_Readable_YPR) || defined(__Output_Readable_Real_Acc) || defined(__Output_Readable_World_Acc)
    printMPU();
#endif
  }

#ifdef __dual_Motor_Control
  for (int i = 0; i < motors; i++)
  {
    currPos[i] = pulses[i] * __encoder_Resolution * __motorAngle_2_cartPos;
    delPos[i] = currPos[i] - prevPos[i];
    prevPos[i] = currPos[i];
  }
#elif defined(__single_Motor_Control)
  currPos[0] = pulses[0] * __encoder_Resolution * __motorAngle_2_cartPos;
  delPos[0] = currPos[0] - prevPos[0];
  prevPos[0] = currPos[0];
#endif

  currTheta = ypr[3];
  delTheta = currTheta - prevTheta;
  prevTheta = currTheta;

#ifdef __pid_Control
#ifdef __dual_Motor_Control
  for (int i = 0; i < motors; i++)
    myMotor[i].evaluatePID(delTime, currTheta, 0.0, 1);

  for (int i = 0; i < motors; i++)
    myMotor[i].runPID();
#endif

#ifdef __single_Motor_Control
  myMotor[0].evaluatePID(delTime, currTheta, 0.0, 1);
  myMotor[0].runPID(pwm_Channel_B, pin_Clockwise_B, pin_Anti_Clockwise_B);
#endif
#endif

#ifdef __lqr_Control
#ifdef __dual_Motor_Control
  for (int i = 0; i < motors; i++)
    myMotor[i].evaluateLQR(delTime, currPos[i], delPos[i], currTheta, delTheta);

  for (int i = 0; i < motors; i++)
    myMotor[i].runLQR();
#endif

#ifdef __single_Motor_Control
  myMotor[0].evaluateLQR(delTime, currPos[0], delPos[0], currTheta, delTheta);
  myMotor[0].runLQR(pwm_Channel_B, pin_Clockwise_B, pin_Anti_Clockwise_B);
#endif
#endif

#ifdef __Output_Control_Cycle_Time
  Serial.println(delTime, 4);
#endif
}

void encPulse_A()
{
  if (digitalRead(enc_2[0]) == LOW)
    pulses[0]++;
  else
    pulses[0]--;
}

#ifdef __dual_Motor_Control
void encPulse_B()
{
  if (digitalRead(enc_2[1]) == LOW)
    pulses[1]++;
  else
    pulses[1]--;
}
#endif

void dmpDataReady()
{
  mpuInterrupt = true;
}

void motorInputA(bool dir, int dutyCycle)
{
  digitalWrite(pin_Clockwise_A, dir);
  digitalWrite(pin_Anti_Clockwise_A, !dir);
  ledcWrite(pwm_Channel_A, dutyCycle);
}

void motorInputB(bool dir, int dutyCycle)
{
  digitalWrite(pin_Clockwise_B, dir);
  digitalWrite(pin_Anti_Clockwise_B, !dir);
  ledcWrite(pwm_Channel_B, dutyCycle);
}

void printMPU()
{
#ifdef __Output_Readable_YPR
  // display Roll, Pitch, Yaw angles in degrees
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180 / M_PI);
#endif

#ifdef __Output_Readable_Real_Acc
  // display real acceleration, adjusted to remove gravity
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  Serial.print("areal\t");
  Serial.print(aaReal.x);
  Serial.print("\t");
  Serial.print(aaReal.y);
  Serial.print("\t");
  Serial.println(aaReal.z);
#endif

#ifdef __Output_Readable_World_Acc
  // display initial world-frame acceleration, adjusted to remove gravity
  // and rotated based on known orientation from quaternion
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  Serial.print("aworld\t");
  Serial.print(aaWorld.x);
  Serial.print("\t");
  Serial.print(aaWorld.y);
  Serial.print("\t");
  Serial.println(aaWorld.z);
#endif
}
#endif

/*





=====================================================
=============        Calibration        =============
=====================================================
*/

#ifdef __calibrate_Sensor
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 accelgyro;
// MPU6050 accelgyro(0x69); // <-- use for AD0 high

const char LBRACKET = '[';
const char RBRACKET = ']';
const char COMMA = ',';
const char BLANK = ' ';
const char PERIOD = '.';

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

const int usDelay = 3150; // empirical, to hold sampling to 200 Hz
const int NFast = 1000;   // the bigger, the better (but slower)
const int NSlow = 10000;  // ..
const int LinesBetweenHeaders = 5;
int LowValue[6];
int HighValue[6];
int Smoothed[6];
int LowOffset[6];
int HighOffset[6];
int Target[6];
int LinesOut;
int N;

void SetAveraging(int NewN);

void ForceHeader()
{
  LinesOut = 99;
}

void GetSmoothed()
{
  int16_t RawValue[6];
  int i;
  long Sums[6];
  for (i = iAx; i <= iGz; i++)
  {
    Sums[i] = 0;
  }
  //    unsigned long Start = micros();

  for (i = 1; i <= N; i++)
  { // get sums
    accelgyro.getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz],
                         &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
    if ((i % 500) == 0)
      Serial.print(PERIOD);
    delayMicroseconds(usDelay);
    for (int j = iAx; j <= iGz; j++)
      Sums[j] = Sums[j] + RawValue[j];
  } // get sums
    //    unsigned long usForN = micros() - Start;
    //    Serial.print(" reading at ");
    //    Serial.print(1000000/((usForN+N/2)/N));
    //    Serial.println(" Hz");
  for (i = iAx; i <= iGz; i++)
  {
    Smoothed[i] = (Sums[i] + N / 2) / N;
  }
} // GetSmoothed

void Initialize()
{
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  Serial.println("PID tuning Each Dot = 100 readings");

  accelgyro.CalibrateAccel(6);
  accelgyro.CalibrateGyro(6);
  Serial.println("\nat 600 Readings");
  accelgyro.PrintActiveOffsets();
  Serial.println();
  accelgyro.CalibrateAccel(1);
  accelgyro.CalibrateGyro(1);
  Serial.println("700 Total Readings");
  accelgyro.PrintActiveOffsets();
  Serial.println();
  accelgyro.CalibrateAccel(1);
  accelgyro.CalibrateGyro(1);
  Serial.println("800 Total Readings");
  accelgyro.PrintActiveOffsets();
  Serial.println();
  accelgyro.CalibrateAccel(1);
  accelgyro.CalibrateGyro(1);
  Serial.println("900 Total Readings");
  accelgyro.PrintActiveOffsets();
  Serial.println();
  accelgyro.CalibrateAccel(1);
  accelgyro.CalibrateGyro(1);
  Serial.println("1000 Total Readings");
  accelgyro.PrintActiveOffsets();
  Serial.println("\n\n Any of the above offsets will work nice \n\n Lets proof the PID tuning using another method:");
}

void SetOffsets(int TheOffsets[6])
{
  accelgyro.setXAccelOffset(TheOffsets[iAx]);
  accelgyro.setYAccelOffset(TheOffsets[iAy]);
  accelgyro.setZAccelOffset(TheOffsets[iAz]);
  accelgyro.setXGyroOffset(TheOffsets[iGx]);
  accelgyro.setYGyroOffset(TheOffsets[iGy]);
  accelgyro.setZGyroOffset(TheOffsets[iGz]);
}

void ShowProgress()
{
  if (LinesOut >= LinesBetweenHeaders)
  {
    Serial.println("\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
    LinesOut = 0;
  }
  Serial.print(BLANK);
  for (int i = iAx; i <= iGz; i++)
  {
    Serial.print(LBRACKET);
    Serial.print(LowOffset[i]),
        Serial.print(COMMA);
    Serial.print(HighOffset[i]);
    Serial.print("] --> [");
    Serial.print(LowValue[i]);
    Serial.print(COMMA);
    Serial.print(HighValue[i]);
    if (i == iGz)
    {
      Serial.println(RBRACKET);
    }
    else
    {
      Serial.print("]\t");
    }
  }
  LinesOut++;
}

void PullBracketsIn()
{
  boolean AllBracketsNarrow;
  boolean StillWorking;
  int NewOffset[6];

  Serial.println("\nclosing in:");
  AllBracketsNarrow = false;
  ForceHeader();
  StillWorking = true;
  while (StillWorking)
  {
    StillWorking = false;
    if (AllBracketsNarrow && (N == NFast))
    {
      SetAveraging(NSlow);
    }
    else
    {
      AllBracketsNarrow = true;
    }
    for (int i = iAx; i <= iGz; i++)
    {
      if (HighOffset[i] <= (LowOffset[i] + 1))
      {
        NewOffset[i] = LowOffset[i];
      }
      else
      {
        StillWorking = true;
        NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
        if (HighOffset[i] > (LowOffset[i] + 10))
        {
          AllBracketsNarrow = false;
        }
      }
    }
    SetOffsets(NewOffset);
    GetSmoothed();
    for (int i = iAx; i <= iGz; i++)
    {
      if (Smoothed[i] > Target[i])
      {
        HighOffset[i] = NewOffset[i];
        HighValue[i] = Smoothed[i];
      }
      else
      {
        LowOffset[i] = NewOffset[i];
        LowValue[i] = Smoothed[i];
      }
    }
    ShowProgress();
  }
}

void PullBracketsOut()
{
  boolean Done = false;
  int NextLowOffset[6];
  int NextHighOffset[6];

  Serial.println("expanding:");
  ForceHeader();

  while (!Done)
  {
    Done = true;
    SetOffsets(LowOffset);
    GetSmoothed();
    for (int i = iAx; i <= iGz; i++)
    {
      LowValue[i] = Smoothed[i];
      if (LowValue[i] >= Target[i])
      {
        Done = false;
        NextLowOffset[i] = LowOffset[i] - 1000;
      }
      else
      {
        NextLowOffset[i] = LowOffset[i];
      }
    }

    SetOffsets(HighOffset);
    GetSmoothed();
    for (int i = iAx; i <= iGz; i++)
    {
      HighValue[i] = Smoothed[i];
      if (HighValue[i] <= Target[i])
      {
        Done = false;
        NextHighOffset[i] = HighOffset[i] + 1000;
      }
      else
      {
        NextHighOffset[i] = HighOffset[i];
      }
    }
    ShowProgress();
    for (int i = iAx; i <= iGz; i++)
    {
      LowOffset[i] = NextLowOffset[i];
      HighOffset[i] = NextHighOffset[i];
    }
  }
}

void SetAveraging(int NewN)
{
  N = NewN;
  Serial.print("averaging ");
  Serial.print(N);
  Serial.println(" readings each time");
}

void setup()
{
  Initialize();
  for (int i = iAx; i <= iGz; i++)
  {                // set targets and initial guesses
    Target[i] = 0; // must fix for ZAccel
    HighOffset[i] = 0;
    LowOffset[i] = 0;
  }
  Target[iAz] = 16384;
  SetAveraging(NFast);

  PullBracketsOut();
  PullBracketsIn();

  Serial.println("-------------- done --------------");
}

void loop()
{
}
#endif