#include "main.h"
#include "pros/api_legacy.h"
#include <array>
#include "pid.hpp"

PID::PID(long double *kp, long double *ki, long double *kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  resetError();
}

void PID::resetError()
{
  error_sum_ = 0.0;
  last_error_ = 0.0;
}

long double PID::update(long double setpoint, long double current_value, long double integral_active_zone)
{
  long double error = setpoint - current_value;
  long double errorDifference = error - last_error_;

  last_error_ = error;
  error_sum_ += (abs((int)error) < integral_active_zone ? error : 0.0);

  long double proportionalError = *kp_ * error;
  long double integralError = (abs((int)error) < integral_active_zone ? *ki_ * error_sum_ : 0);
  long double derivativeError = *kd_ * errorDifference;

  long double total = proportionalError + integralError + derivativeError;


  return total;
}
