#ifndef _PID_H_
#define _PID_H_

#include <Arduino.h>

#define __clockwise 0
#define __anti_Clockwise 1

class PID
{
private:
   uint8_t max_Motor_Voltage;
   uint8_t pwm_Channel;
   int pwm_Resolution, pwm_Frequency;
   uint8_t enc_1, enc_2;
   uint8_t pin_Clockwise, pin_Anti_Clockwise, pin_Pwm;
   double K[3];

   float err, e_Int, de_dt, prev_Err;
   int myPwm;
   bool dir;

   double pre_Filter_Inp[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   double pre_Filter_Opt[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

   double evaluatePreFilter(double inp);

public:
   void configPID(int max_Motor_Voltage, int pin_Pwm, int pwm_Channel, int pwm_Resolution, int pwm_Frequency, int enc_1, int enc_2, int pin_Clockwise, int pin_Anti_Clockwise, double *K);
   void beginPID();
   void evaluatePID(double dTime, double output, double reference, bool preFilter);
   void runPID();
   void runPID(uint8_t pwmPinB, uint8_t dirCPinB, uint8_t dirACPinB);
};

#endif