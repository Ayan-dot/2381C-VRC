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
extern std::array<double, 3> adjustmentPIDParams;

extern int maxSpeed;

extern double verticalOffset1;
extern double verticalOffset2;
extern double horizontalOffset;
extern double horiToInch;
extern double vertToInch;
extern double imuToRad;
extern double globalX, globalY;

extern const double pi;

extern lv_obj_t *blue;
extern lv_obj_t *red;

extern lv_obj_t *tabview;

extern pros::ADIEncoder verticalEncoder1;
extern pros::ADIEncoder verticalEncoder2;
extern pros::ADIEncoder horizontalEncoder;


#endif
