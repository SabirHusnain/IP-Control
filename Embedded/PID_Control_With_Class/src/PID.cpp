#include "PID.h"

void PID::configPID(int max_Motor_Voltage, int pin_Pwm, int pwm_Channel, int pwm_Resolution, int pwm_Frequency, int enc_1, int enc_2, int pin_Clockwise, int pin_Anti_Clockwise, double *K)
{
   this->max_Motor_Voltage = (uint8_t)max_Motor_Voltage;
   this->pin_Pwm = pin_Pwm;
   this->pwm_Channel = (uint8_t)pwm_Channel;
   this->pwm_Resolution = pwm_Resolution;
   this->pwm_Frequency = pwm_Frequency;
   this->enc_1 = (uint8_t)enc_1;
   this->enc_2 = (uint8_t)enc_2;
   this->pin_Clockwise = (uint8_t)pin_Clockwise;
   this->pin_Anti_Clockwise = (uint8_t)pin_Anti_Clockwise;
   for (int i = 0; i < 3; i++)
      this->K[i] = *(K++);
   this->err = 0;
   this->e_Int = 0;
   this->de_dt = 0;
   this->prev_Err = 0;
}

void PID::beginPID()
{
   pinMode(this->enc_1, INPUT_PULLUP);
   pinMode(this->enc_2, INPUT_PULLUP);
   pinMode(this->pin_Clockwise, OUTPUT);
   pinMode(this->pin_Anti_Clockwise, OUTPUT);
   ledcSetup(this->pwm_Channel, this->pwm_Frequency, this->pwm_Resolution);
   ledcAttachPin(this->pin_Pwm, this->pwm_Channel);
}

void PID::evaluatePID(double dTime, double output, double reference, bool preFilter)
{
   float U, V;
   if (preFilter)
      this->err = evaluatePreFilter(reference) - output;
   else
      this->err = reference - output;

   this->e_Int += err * (dTime / 1.0e6);
   this->de_dt = ((1.0e6) * (this->err - this->prev_Err)) / dTime;
   U = this->K[0] * this->err + this->K[1] * this->e_Int + this->K[2] * de_dt;
   this->prev_Err = err;

   if (abs(U) > this->max_Motor_Voltage)
      V = max_Motor_Voltage;
   else
      V = abs(U);

   // Integral Windup
   if ((abs(U) != V) && (((this->err > 0) && (U > 0)) || ((this->err < 0) && (U < 0))))
      this->e_Int = 0;

   this->myPwm = map(V, 0, max_Motor_Voltage, 0, pow(2, pwm_Resolution) - 1); // Voltage to PWM Conversion

   if (U < 0)
      this->dir = __anti_Clockwise;
   else
      this->dir = __clockwise;
}

void PID::runPID()
{
   digitalWrite(this->pin_Clockwise, this->dir);
   digitalWrite(this->pin_Anti_Clockwise, !(this->dir));
   ledcWrite(this->pwm_Channel, this->myPwm);
}

void PID::runPID(uint8_t pwmChannelB, uint8_t dirCPinB, uint8_t dirACPinB)
{
   digitalWrite(this->pin_Clockwise, this->dir);
   digitalWrite(this->pin_Anti_Clockwise, !(this->dir));
   digitalWrite(dirCPinB, this->dir);
   digitalWrite(dirACPinB, !(this->dir));
   ledcWrite(pwmChannelB, this->myPwm);
}

double PID::evaluatePreFilter(double inp)
{
   double opt = (8.189e-5) * inp - (2.06e-5) * this->pre_Filter_Inp[0] - (3.024e-4) * this->pre_Filter_Inp[1] + (1.124e-4) * this->pre_Filter_Inp[2] + (4.257e-4) * this->pre_Filter_Inp[3] - (2.038e-4) * this->pre_Filter_Inp[4] - (2.717e-4) * this->pre_Filter_Inp[5] + (1.529e-4) * this->pre_Filter_Inp[6] + (6.655e-5) * this->pre_Filter_Inp[7] - (4.083e-5) * this->pre_Filter_Inp[8] + 7.669 * this->pre_Filter_Opt[0] - 26.23 * this->pre_Filter_Opt[1] + 52.51 * this->pre_Filter_Opt[2] - 67.83 * this->pre_Filter_Opt[3] + 58.63 * this->pre_Filter_Opt[4] - 33.92 * this->pre_Filter_Opt[5] + 12.66 * this->pre_Filter_Opt[6] - 2.77 * this->pre_Filter_Opt[7] + 0.2704 * this->pre_Filter_Opt[8];

   for (int i = 8; i > 0; i--)
      this->pre_Filter_Inp[i] = this->pre_Filter_Inp[i - 1];
   this->pre_Filter_Inp[0] = inp;

   for (int i = 8; i > 0; i--)
      this->pre_Filter_Opt[i] = this->pre_Filter_Opt[i - 1];
   this->pre_Filter_Opt[0] = opt;

   return opt;
}