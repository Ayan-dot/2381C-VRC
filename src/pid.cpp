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

  double total = proportionalError + integralError + derivateError;

  // set a maximum threshold for the motor voltage
  if (abs((int)total) > 8000) {
    if (total < 0) {
      total = -7500;
    } else {
      total = 7500;
    }
  }

  // // set a minimum threshold too
  // if (abs((int)total) < 3000) {
  //   if (total < 0) {
  //     total = -2000;
  //   } else {
  //     total = 2000;
  //   }
  // }

  // if(total > 8000) {
  //   total = 6500;
  // }
  //
  // if(total < -6500) {
  //   total = -6500;
  // }
  //
  // if(total > -3000 && total < 0) {
  //   total = -3000;
  // }

  return total;
}
