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

globals.cpp [contains]:
  - Controller, motor, and sensor definitions (ports, reversals)
  - Tunable constants used throughout the code, like PID constants, and
    lighting constants
  - Calibration of sensors in initialize()
*/
#include "main.h"
#include <array>
#include "globals.hpp"
#include "pid.hpp"

// V5 controller definition
pros::Controller master(pros::E_CONTROLLER_MASTER);

// V5 motor definitions
pros::Motor leftBack(1);
pros::Motor leftFront(20);
pros::Motor rightBack(10);
pros::Motor rightFront(11);
pros::Motor leftIntake(15);
pros::Motor rightIntake(16);
pros::Motor indexer(17);
pros::Motor shooter(14);

// inertial sensor definition
pros::Imu inertial(19);

// optical shaft encoder (for tracking wheels) definitions
pros::ADIEncoder verticalEncoder1('A', 'B', false);
pros::ADIEncoder verticalEncoder2('C', 'D', true);
pros::ADIEncoder horizontalEncoder('E', 'F', true);

// line trackers definitions (for auto indexing)
pros::ADIAnalogIn line_tracker1('G');
pros::ADIAnalogIn line_tracker2('H');

// kp, ki, and kd constants respectively for creating PID objects from PID class (in PID.cpp and PID.hpp)
std::array<long double, 3> pointTurnPIDParams = {12000, 0, 8000};
std::array<long double, 3> drivebasePIDParams = {52, 2, 180};
std::array<long double, 3> turningPID = {14000, 2, 180};
std::array<long double, 3> strafePIDParams = {52, 2, 180};

// time spent indexing each ball
int indTime = 185;

// max voltage sent to motors
int maxSpeed = 12000;

// line sensor threshold, depending on ambient lighting conditions
int INDEX_THRESHOLD = 2750;

// epsilon, near zero value, to avoid zero division error
const long double EPS = 1e-8;

// time in ms given to accelerate for square root acceleration in translationPID
long double accelerationTime = 550;

// tracking wheel offsets for odometry
long double verticalOffset1 = 5.905;
long double verticalOffset2 = 5.905;
long double horizontalOffset = 6.660;

// ticks to inches conversions
long double horiToInch = (pi * 2.75) / 360.0;
long double vertToInch = (pi * 2.75) / 360.0;

// degrees to radians conversion
long double imuToRad = pi / 180.0;

// global x and y values for odometry
long double globalX = 0, globalY = 0;

// mathematical constant pi
// note: defined using trignometry instead of decimals to maximize accuracy
const long double pi = asin(1) * 2.0;

void disabled() {}

void competition_initialize()
{
  initialize();
}

void initialize()
{
  // initialize a timer
  int time = pros::millis();
  int iter = 0;

  // initialize V5 brain screen for debugging
  pros::lcd::initialize();

  // calibrate inertial sensor
  inertial.reset();
  while (inertial.is_calibrating())
  {
    pros::lcd::set_text(7, "IMU calibrating ...");
    iter += 20;
    pros::delay(20);
  }
  pros::lcd::set_text(7, "IMU is done calibrating (took %d ms)\n" + std::to_string(iter - time));

  // calibrate optical shaft encoders
  verticalEncoder1.reset();
  verticalEncoder2.reset();
  horizontalEncoder.reset();
}
