/*
  ___  ____   ___  __  _____
 |__ \|___ \ / _ \/_ |/ ____|
    ) | __) | (_) || | |
   / / |__ < > _ < | | |
  / /_ ___) | (_) || | |____
 |____|____/ \___/ |_|\_____|

2381C <Team Captain: allentao7@gmail.com>

This file is part of 2381C's codebase for 2020-21 VEX Robotics VRC Change
Up Competition.

This file can not be copied, modified, or distributed without the express
permission of 2381C.

All relevant mathematical calculations for odometry and motion profiling are
documented and have been explained in extensive detail in our paper about
robot motion. The paper is located in the docs (documentation) folder.

pid.cpp [contains]:
  - Defines our PID class from which we create PID objects for different
    subsystems like the drive train
*/

#include "main.h"
#include "pros/api_legacy.h"
#include <array>
#include "pid.hpp"

/**
 * Constructor for the PID, using kp, ki and kd values to initialize as the
 * parameters for the loop.
 */
PID::PID(long double *kp, long double *ki, long double *kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  resetError();
}

/**
 * Reset PID errors to 0.
 */
void PID::resetError()
{
  error_sum_ = 0.0;
  last_error_ = 0.0;
}

/**
 * PID state functions
 * 
 * @param setpoint the target point
 * @param current_value the current point
 * @param integral_active_zone the point at which the integral is activated
 * 
 * @return the sum of the PID errors
*/
long double PID::update(long double setpoint, long double current_value, long double integral_active_zone)
{
  // calculate errors
  long double error = setpoint - current_value;
  long double errorDifference = error - last_error_;

  last_error_ = error;
  error_sum_ += (abs((int)error) < integral_active_zone ? error : 0.0);

  long double proportionalError = *kp_ * error;
  // integral active zone is a certain threshold of error at which integral
  // becomes active
  long double integralError = (abs((int)error) < integral_active_zone ? *ki_ * error_sum_ : 0);
  long double derivativeError = *kd_ * errorDifference;

  long double total = proportionalError + integralError + derivativeError;
  return total;
}