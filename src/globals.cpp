#include "main.h"
#include <array>
#include "globals.hpp"
#include "pid.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor leftBack(20);
pros::Motor leftFront(1);
pros::Motor rightBack(11);
pros::Motor rightFront(2);
pros::Imu inertial(9);
pros::ADIEncoder verticalEncoder1('A','B', true);
pros::ADIEncoder verticalEncoder2('C','D', true);
pros::ADIEncoder horizontalEncoder('E','F', true);

std::array<double, 3> anglerPIDParams = {0.07, 0, 0};
std::array<double, 3> drivebasePIDParams = {23, 0, 0};
std::array<double, 3> turningPID = {160, 0, 170};
std::array<double, 3> adjustmentPIDParams = {2,0,0};

int maxSpeed = 12000;
double verticalOffset1 = 7.185; // needs to be changed depending on vertical tracking wheel placement
double verticalOffset2 = 7.185;
double horizontalOffset = 7.25;
double horiToInch = (pi*2.75)/360.0;
double vertToInch = (pi*2.75)/360.0;
double imuToRad = pi/180.0;
double globalX = 0, globalY = 0;

const double pi = 3.14159265358979323846;

lv_obj_t *blue;
lv_obj_t *red;
lv_obj_t *tabview;

void disabled() {}

void competition_initialize() {}


