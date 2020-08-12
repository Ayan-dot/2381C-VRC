#ifndef GLOBAL_H_
#define GLOBAL_H_

#include "api.h"

extern pros::Controller master;
extern pros::Motor leftBack, leftFront, rightBack, rightFront;
extern pros::Motor leftIntake, rightIntake, indexer, shooter;
extern pros::Motor lift;
extern pros::Imu inertial;

extern std::array<long double, 3> pointTurnPIDParams;
extern std::array<long double, 3> drivebasePIDParams;
extern std::array<long double, 3> turningPID;
extern std::array<long double, 3> strafePIDParams;

extern int maxSpeed;
extern int INDEX_THRESHOLD;
extern long double ANGLE_Kp;
extern long double STRAFE_Kp;
extern long double accelerationTime;
extern const long double EPS;
//extern int coastVoltage;

extern long double verticalOffset1;
extern long double verticalOffset2;
extern long double horizontalOffset;
extern long double horiToInch;
extern long double vertToInch;
extern long double imuToRad;
extern long double globalX, globalY;

extern const long double pi;

extern pros::ADIEncoder verticalEncoder1;
extern pros::ADIEncoder verticalEncoder2;
extern pros::ADIEncoder horizontalEncoder;
extern pros::ADIAnalogIn line_tracker;


#endif
