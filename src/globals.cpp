#include "main.h"
#include <array>
#include "globals.hpp"
#include "pid.hpp"


pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor leftBack(11);
pros::Motor leftFront(2);
pros::Motor rightBack(20);
pros::Motor rightFront(9);
pros::ADIEncoder verticalEncoder1('E','F', false);
pros::ADIEncoder verticalEncoder2('G','H', false);
pros::ADIEncoder horizontalEncoder('C','D', false);

std::array<double, 3> anglerPIDParams = {0.07, 0, 0};
std::array<double, 3> drivebasePIDParams = {23, 0, 0};
std::array<double, 3> turningPID = {160, 0, 170};
std::array<double, 3> adjustmentPIDParams = {2,0,0};

int maxSpeed = 12000;
double verticalOffset1 = 5.13; // needs to be changed depending on vertical tracking wheel placement
double verticalOffset2 = 5.13;
double horizontalOffset = 0.75;
double horiToInch = (pi*2.75)/360.0;
double vertToInch = (pi*2.75)/360.0;
double imuToRad = pi/180.0;
double globalX = 0, globalY = 0;

const double pi = 3.14159265358979323846;

void disabled() {}

void competition_initialize() {}


void initialize()
{

    int time = pros::millis();
    int iter = 0;

    pros::lcd::initialize();

    verticalEncoder1.reset();
    verticalEncoder2.reset();
    horizontalEncoder.reset();

    master.clear();

}