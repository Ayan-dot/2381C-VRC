#ifndef PID_H_
#define PID_H_

class PID
{
public:
  /**
   * Constructor
   * Accepts the kp, ki, and kd to determine the correction value
   * Defaults to (0,0,0)
   */
  PID(long double *kp = 0, long double *ki = 0, long double *kd = 0);
  /**
   * Resets the error counts. It should be called when the PID loop is not
   * active to prevent integral windup.
   */
  void resetError();

  // Summation of errors used in the integral term
  long double error_sum_;

  // The last error value to find the difference with the current error value
  // for the derivative term.
  long double last_error_;

  /**
   * Returns the output of the PID controller correcting the input.
   * @param setpoint The current setpoint value
   * @param current_value The current value that will be compared with the setpoint
   */
  long double update(long double setpoint, long double current_value, long double integral_active_zone);

private:
  // PID constants (Proportional (P), Integral (I), Derivative (D))
  long double *kp_;
  long double *ki_;
  long double *kd_;
};

#endif // !PID_H_
