#ifndef GLOBAL_H_
#define GLOBAL_H_

#include "api.h"

extern pros::Controller master;
extern pros::Motor leftBack, leftFront, rightBack, rightFront;
extern pros::Motor leftIntake, rightIntake;
extern pros::Motor lift;
extern pros::Imu inertial;

extern std::array<double, 3> anglerPIDParams;
extern std::array<double, 3> drivebasePIDParams;
extern std::array<double, 3> turningPID;

extern int maxSpeed;

extern double verticalOffset;
extern double horizontalOffset;
extern double horiToInch;
extern double vertToInch;
extern double imuToRad;

extern const double pi;

extern pros::ADIEncoder verticalEncoder;
extern pros::ADIEncoder horizontalEncoder;


#endif

