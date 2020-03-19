#include "main.h"
#include <array>
#include "globals.hpp"
#include "pid.hpp"


pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor leftBack(12);
pros::Motor leftFront(11);
pros::Motor rightBack(19);
pros::Motor rightFront(20);
pros::Motor leftIntake(10);
pros::Motor rightIntake(15);
pros::Motor lift(14);
pros::Imu inertial(21);
pros::ADIEncoder verticalEncoder('A','B', false);
pros::ADIEncoder horizontalEncoder('C','D', false);

std::array<double, 3> anglerPIDParams = {0.07, 0, 0};
std::array<double, 3> drivebasePIDParams = {0.032, 0, 0};

int maxSpeed = 12000;
double verticalOffset = 5; // needs to be changed depending on vertical tracking wheel placement
double horizontalOffset = 5;

const double pi = 3.14159265358979323846;

void initialize() {
	pros::lcd::initialize();
    
    int time = pros::millis();
    int iter = 0;

    while (inertial.is_calibrating()) {
        printf("IMU calibrating... %d\n", iter);
        iter += 10;
        pros::delay(10);
    }

}

void disabled() {}

void competition_initialize() {}