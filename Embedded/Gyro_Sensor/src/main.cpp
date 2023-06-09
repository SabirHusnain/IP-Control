#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

#define __rad_2_deg 57.29578

double currTime = 0, prevTime = 0, delTime = 0, setupTime = 0, printTime = 0;
bool first_Itr = true;

struct Angle
{
   double x = 0, y = 0, z = 0;
} gyroAngle, accAngle, finalAngle;

float gx = 0.0, gy = 0.0, g = 10.2838, h = 0.0;

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_accel, *mpu_gyro;
sensors_event_t acc, gyro;

double myFunc(double num);

void setup(void)
{
   Serial.begin(115200);
   Serial.println("Adafruit MPU6050 test!");

   if (!mpu.begin())
   {
      Serial.println("Failed to find MPU6050 chip");
      while (1)
      {
         delay(250);
      }
   }
   Serial.println("MPU6050 Found!");

   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
   mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
   mpu.setFilterBandwidth(MPU6050_BAND_44_HZ); // Cut of freq

   mpu_accel = mpu.getAccelerometerSensor();
   mpu_gyro = mpu.getGyroSensor();

   Serial.println("");
   setupTime = micros();
}

void loop()
{
   currTime = micros() - setupTime;
   delTime = currTime - prevTime;
   prevTime = currTime;

   mpu_accel->getEvent(&acc);
   mpu_gyro->getEvent(&gyro);

   h = sqrt(pow(acc.acceleration.x, 2) + pow(acc.acceleration.y, 2)) - g;

   float gy_1 = (acc.acceleration.x * myFunc(sqrt((2.0 * g * h - pow(acc.acceleration.x, 2) - pow(acc.acceleration.y, 2) + pow(g, 2) + pow(h, 2)) * (2.0 * g * h + pow(acc.acceleration.x, 2) + pow(acc.acceleration.y, 2) - pow(g, 2) - pow(h, 2)))) + pow(acc.acceleration.x, 2) * acc.acceleration.y + acc.acceleration.y * pow(g, 2) - acc.acceleration.y * pow(h, 2) + pow(acc.acceleration.y, 3)) / (pow(acc.acceleration.x, 2) * 2.0 + pow(acc.acceleration.y, 2) * 2.0);

   float gy_2 = (-acc.acceleration.x * myFunc(sqrt((g * h * 2.0 - pow(acc.acceleration.x, 2) - pow(acc.acceleration.y, 2) + pow(g, 2) + pow(h, 2)) * (g * h * 2.0 + pow(acc.acceleration.x, 2) + pow(acc.acceleration.y, 2) - pow(g, 2) - pow(h, 2)))) + pow(acc.acceleration.x, 2) * acc.acceleration.y + acc.acceleration.y * pow(g, 2) - acc.acceleration.y * pow(h, 2) + pow(acc.acceleration.y, 3)) / (pow(acc.acceleration.x, 2) * 2.0 + pow(acc.acceleration.y, 2) * 2.0);

   float gx_1 = sqrt(pow(g, 2) - pow(gy_1, 2));
   float gx_2 = sqrt(pow(g, 2) - pow(gy_2, 2));

   float diff_1 = abs(g - sqrt(abs(pow(gx_1, 2) + pow(gy_1, 2))));
   float diff_2 = abs(g - sqrt(abs(pow(gx_2, 2) + pow(gy_2, 2))));

   if (diff_1 < diff_2)
   {
      gx = gx_1;
      gy = gy_1;
   }
   else
   {
      gx = gx_2;
      gy = gy_2;
   }

   accAngle.z = (-atan2(gy, gx)); // - 0.0873;
   gyroAngle.z += gyro.gyro.z * (delTime * 1e-6) + (6.866815e-9 * delTime);

   finalAngle.z = 0.02 * (gyroAngle.z) + 0.98 * (accAngle.z);

   if (first_Itr)
   {
      gyroAngle.z = accAngle.z;
      finalAngle.z = accAngle.z;
      first_Itr = false;
   }

   if (millis() - printTime >= 100)
   {
      Serial.print("Rotation Gyro: ");
      Serial.print(gyroAngle.z * __rad_2_deg, 6);
      Serial.println(" deg");

      Serial.print("Rotation Acc: ");
      Serial.print(accAngle.z * __rad_2_deg, 6);
      Serial.println(" deg");

      Serial.print("Rotation Final: ");
      Serial.print(finalAngle.z * __rad_2_deg, 6);
      Serial.println(" deg");

      Serial.println("");

      printTime = millis();
   }

   /* Print out the values */
   // Serial.print("Acceleration X: ");
   // Serial.print(a.acceleration.x);
   // Serial.print(", Y: ");
   // Serial.print(a.acceleration.y);
   // Serial.print(", Z: ");
   // Serial.print(a.acceleration.z);
   // Serial.println(" m/s^2");

   // Serial.print("Rotation X: ");
   // Serial.print(g.gyro.x);
   // Serial.print(", Y: ");
   // Serial.print(g.gyro.y);
   // Serial.print(", Z: ");
   // Serial.print(g.gyro.z);
   // Serial.println(" rad/s");

   // Serial.println("");
   // delacc.acceleration.y(500);
}

double myFunc(double num)
{
   if (num > 0)
      return num;
   else
      return 0.0;
}