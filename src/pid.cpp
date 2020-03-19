#include "main.h"
#include "pros/api_legacy.h"
#include <array>
#include "pid.hpp"

PID::PID(double *kp, double *ki, double *kd)
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

double PID::update(double setpoint, double current_value)
{
  double error = setpoint - current_value;
  double errorDifference = error - last_error_;

  last_error_ = error;
  error_sum_ += error;

  double proportionalError = *kp_ * error;
  double integralError = *ki_ * error_sum_;
  double derivateError = *kd_ * errorDifference;

  return proportionalError + integralError + derivateError;
}

