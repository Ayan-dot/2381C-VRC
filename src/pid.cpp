/*
  ___  ____   ___  __  _____
 |__ \|___ \ / _ \/_ |/ ____|
    ) | __) | (_) || | |
   / / |__ < > _ < | | |
  / /_ ___) | (_) || | |____
 |____|____/ \___/ |_|\_____|

All code is the property of 2381C, Kernel Bye. ANY UNAUTHORIZED REPRODUCTION
OR DISTRIBUTION OF THIS CODE IS STRICTLY FORBIDDEN. Please contact team 2381C
directly with any questions, concerns or suggestions you may have.

pid.cpp [contains]:
  * Defines our PID class from which we create PID objects for different subsystems like the drivetrain

NOTE: All relevant mathematical calculations (odometry and motion) are documented in extensive detail in our paper regarding robot motion
  * https://drive.google.com/file/d/1zBMroM90nDU6iHqsbI_qOgd120M7x-rd/view

*/

// Necessary imports
#include "main.h"
#include "pros/api_legacy.h"
#include <array>
#include "pid.hpp"

PID::PID(long double *kp, long double *ki, long double *kd)
{
  // Initialization
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  resetError();
}

void PID::resetError()
{
  // Reset values to zero
  error_sum_ = 0.0;
  last_error_ = 0.0;
}

long double PID::update(long double setpoint, long double current_value, long double integral_active_zone)
{
  // PID calculations
  long double error = setpoint - current_value;
  long double errorDifference = error - last_error_;

  last_error_ = error;
  error_sum_ += (abs((int)error) < integral_active_zone ? error : 0.0);

  long double proportionalError = *kp_ * error;
  long double integralError = (abs((int)error) < integral_active_zone ? *ki_ * error_sum_ : 0); // integral active zone is a certain threshold of error at which integral becomes active
  long double derivativeError = *kd_ * errorDifference;

  long double total = proportionalError + integralError + derivativeError;

  // // set a maximum threshold for the motor voltage
  // if (abs((int)total) > 9000) {
  //   if (total < 0) {
  //     total = -9000;
  //   } else {
  //     total = 9000;
  //   }
  // }

  return total;
}
