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
*/

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include "api.h"

extern pros::Controller master;
extern pros::Motor leftBack, leftFront, rightBack, rightFront;
extern pros::Motor leftIntake, rightIntake;
extern pros::Motor shooter, indexer;
extern pros::Motor lift;
extern pros::Imu inertial;

extern std::array<double, 3> anglerPIDParams;
extern std::array<double, 3> drivebasePIDParams;

extern int maxSpeed;

extern double verticalOffset;
extern double horizontalOffset;
extern double horiToInch;
extern double vertToInch;
extern double imuToRad;

extern const double pi;

extern pros::ADIEncoder verticalEncoder;
extern pros::ADIEncoder horizontalEncoder;
extern pros::ADIAnalogIn line_tracker1, line_tracker2;

#endif
