#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// MPU6050 sensor Data
#define adrs 0x68
#define accMaxRange 8.0
#define accSensitivity 4096.0
#define gyroMaxRange 2000.0
#define gyroSensitivity 16.4
#define earthGravity 9.80665
#define offsetGravity 10.32

#define __rad_2_deg 57.29578

double accMultiple = offsetGravity / accSensitivity;

// Variables
double accX = 0, accY = 0;
int16_t accX_V = 0, accY_V = 0;
int16_t gyroZ = 0;
int64_t gyroErrorZ = 0;
double gyroTheta = 0.0, accTheta = 0.0, accTheta_1 = 0.0, accTheta_2 = 0.0, finalTheta = 0.0, gyroVel = 0.0;
bool first_Itr = true;
double cTime = 0.0, pTime = 0.0, dTime = 0.0, sTime = 0.0, printTime = 0.0;

float gx = 0.0, gy = 0.0, g = offsetGravity, h = 0.0;

// Functions
void config6050();
void readGyro();
void readAcc();
void calibrateGyro();

void setup()
{
  Wire.begin();
  Serial.begin(115200);

  config6050();
  calibrateGyro();

  Serial.println("Gyro Error: " + String(gyroErrorZ));

  delay(2000);

  sTime = micros();
}

void loop()
{
  cTime = micros() - sTime;
  dTime = cTime - pTime;
  pTime = cTime;

  // readAcc();
  readGyro();

  gyroZ = gyroZ - gyroErrorZ;
  gyroVel = gyroZ / gyroSensitivity;
  gyroTheta += gyroVel * (dTime * 1e-6); // + (6.866815e-6 * dTime / 1000);
  if (millis() - printTime >= 100)
  {
    Serial.println(gyroVel);
    printTime = millis();
  }
  /*
  accX = accX_V * accMultiple;
  accY = accY_V * accMultiple;
  accZ = accZ_V * accMultiple;

  if (first_Itr) {
    g = sqrt(pow(accX, 2) + pow(accY, 2));
    // gyroTheta = atan2(accY, accX);
    finalTheta = atan2(accY, accX);
    // first_Itr = false;
  } else {
    h = sqrt(pow(accX, 2) + pow(accY, 2)) - g;

    float gy_1 = (accX * (2 * g * h) + pow(accX, 2) * accY + accY * pow(g, 2) - accY * pow(h, 2) + pow(accY, 3)) / (2 * (pow(accX, 2) + pow(accY, 2)));

    float gy_2 = (pow(accX, 2) * accY - accX * (2 * g * h) + accY * pow(g, 2) - accY * pow(h, 2) + pow(accY, 3)) / (2 * (pow(accX, 2) + pow(accY, 2)));

    float gx_1 = sqrt(pow(g, 2) - pow(gy_1, 2));
    float gx_2 = sqrt(pow(g, 2) - pow(gy_2, 2));

    accTheta_1 = atan2(gy_1, gx_1);
    accTheta_2 = atan2(gy_2, gx_2);

    gyroTheta += gyroVel * (dTime * 1e-6);  // + (6.866815e-6 * dTime / 1000);

    float diff_1 = abs(gyroTheta - accTheta_1);
    float diff_2 = abs(gyroTheta - accTheta_2);

    if (diff_1 < diff_2)
      accTheta = accTheta_1;
    else
      accTheta = accTheta_2;

    finalTheta = 0.65 * (gyroTheta) + 0.35 * (accTheta);
  }

  if (millis() - printTime >= 100) {
    Serial.print("Rotation Gyro: ");
    Serial.print(gyroTheta * __rad_2_deg, 6);
    Serial.println(" deg");

    // Serial.print("Rotation Acc: ");
    // Serial.print(gyroTheta * __rad_2_deg, 6);
    // Serial.println(" deg");

    // Serial.print("Rotation Final: ");
    // Serial.print(finalTheta * __rad_2_deg, 6);
    // Serial.println(" deg");

    Serial.println("");
    printTime = millis();
  }
  */
}

void config6050()
{
  Wire.beginTransmission(adrs);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();

  Wire.beginTransmission(adrs);
  Wire.write(0x1A);
  Wire.write(0b00000011);
  Wire.endTransmission();

  Wire.beginTransmission(adrs);
  Wire.write(0x1C);
  Wire.write(0b00010000);
  Wire.endTransmission();

  Wire.beginTransmission(adrs);
  Wire.write(0x1B);
  Wire.write(0b00011000);
  Wire.endTransmission();
}

void readGyro()
{
  Wire.beginTransmission(adrs);
  Wire.write(0x47);
  Wire.endTransmission();
  Wire.requestFrom(adrs, 2);
  while (Wire.available() < 2)
    ;
  gyroZ = (Wire.read() << 8) | Wire.read();
}

void readAcc()
{
  Wire.beginTransmission(adrs);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(adrs, 4);
  while (Wire.available() < 4)
    ;
  accX_V = (Wire.read() << 8) | Wire.read();
  accY_V = (Wire.read() << 8) | Wire.read();
}

void calibrateGyro()
{
  for (int i = 0; i < 2000; i++)
  {
    readGyro();
    gyroErrorZ = gyroErrorZ + gyroZ;
  }
  gyroErrorZ = gyroErrorZ / 2000;
}