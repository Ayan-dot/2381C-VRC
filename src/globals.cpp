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

globals.cpp [contains]:
  * Controller, motor, and sensor definitions (ports, reversals)
  * Tunable constants used throughout the code, like PID constants, and lighting constants
  * Calibration of sensors in initialize()

NOTE: All relevant mathematical calculations (odometry and motion) are documented in extensive detail in our paper regarding robot motion
  * https://drive.google.com/file/d/1zBMroM90nDU6iHqsbI_qOgd120M7x-rd/view

*/

// Necessary imports from C++ libraries, and other header files, and cpp files within this directory
#include "main.h"
#include <array>
#include "globals.hpp"
#include "pid.hpp"

// V5 controller definition
pros::Controller master(pros::E_CONTROLLER_MASTER);

// V5 motor definitions
pros::Motor leftBack(2);
pros::Motor leftFront(12);
pros::Motor rightBack(9);
pros::Motor rightFront(19);
pros::Motor leftIntake(15);
pros::Motor rightIntake(16);
pros::Motor indexer(3);
pros::Motor shooter(17);

// Inertial sensor definition
pros::Imu inertial(1);

// Optical shaft encoder (for tracking wheels) definitions
pros::ADIEncoder verticalEncoder1('E','F', false);
pros::ADIEncoder verticalEncoder2('A','B', false);
pros::ADIEncoder horizontalEncoder('C','D', true);

// Line trackers definitions (for auto indexing)
pros::ADIAnalogIn line_tracker1('G');
pros::ADIAnalogIn line_tracker2('H');

// Kp, Ki, and Kd constants respectively for creating PID objects from PID class (in PID.cpp and PID.hpp)
std::array<long double, 3> pointTurnPIDParams = {8000, 0, 8000};
std::array<long double, 3> drivebasePIDParams = {52, 2, 180};
std::array<long double, 3> turningPID = {10000, 0, 0};
std::array<long double, 3> strafePIDParams = {52, 2, 180};

// Max voltage sent to motors
int maxSpeed = 12000;

// Line sensor threshold, depending on lighting conditions
int INDEX_THRESHOLD = 2700; // needs to be changed depending on ambient lighting conditions

// Epsilon, near zero value, for applications to avoid zero division error etc by dividing by a very small value
const long double EPS = 1e-8;

// Time in milliseconds given to accelerate for square root acceleration in translationPID
long double accelerationTime = 750;

// Tracking wheel offsets for odometry
long double verticalOffset1 = 6.1675; // needs to be changed depending on vertical tracking wheel placement
long double verticalOffset2 = 6.1675;
long double horizontalOffset = 6.8075;

// Ticks to inches conversions
long double horiToInch = (pi*2.75)/360.0;
long double vertToInch = (pi*2.75)/360.0;

// Degrees to radians conversion
long double imuToRad = pi/180.0;

// Global x and y values for odometry
long double globalX = 0, globalY = 0;

// Mathematical constant pi, defined using trignometry instead of hard coded decimals for maximal accuracy
const long double pi = asin(1) * 2.0;

void disabled() {}

void competition_initialize() {
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
    while (inertial.is_calibrating()) {
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
