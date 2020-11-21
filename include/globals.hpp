#ifndef GLOBAL_H_
#define GLOBAL_H_

#include "api.h"

extern pros::Controller master; // defines controller
extern pros::Motor leftBack, leftFront, rightBack, rightFront; // defines drive motors
extern pros::Motor leftIntake, rightIntake, indexer, shooter; // defines alternate function motors
extern pros::Imu inertial; // defines inertial sensor

extern std::array<long double, 3> pointTurnPIDParams; // defines PID constant array for point turning
extern std::array<long double, 3> drivebasePIDParams; // defines PID constant array for regular driving
extern std::array<long double, 3> turningPID; // defines PID constant array for strafe turning
extern std::array<long double, 3> strafePIDParams; // defines PID constant array for strafe driving

extern int maxSpeed; // defining motion constants
extern int indTime;
extern int INDEX_THRESHOLD;
extern long double ANGLE_Kp;
extern long double STRAFE_Kp;
extern long double accelerationTime;
extern const long double EPS;
//extern int coastVoltage;

extern long double verticalOffset1; // offset from center for vertical tracking wheel 1
extern long double verticalOffset2; // offset from center for vertical tracking wheel 2
extern long double horizontalOffset; // offset from center for horizontal tracking wheel
extern long double horiToInch; // conversion from horizontal encoder ticks to inches
extern long double vertToInch; // conversion from vertical encoder ticks to inches
extern long double imuToRad; // conversions from inertial sensor degrees to radians
extern long double globalX, globalY; // global X and Y coordinates for the absolute position tracking suite

extern const long double pi; // defines the pi constant

extern pros::ADIEncoder verticalEncoder1; // defining encoders and line tracking sensors
extern pros::ADIEncoder verticalEncoder2;
extern pros::ADIEncoder horizontalEncoder;
extern pros::ADIAnalogIn line_tracker1;
extern pros::ADIAnalogIn line_tracker2;

#endif
