#include "main.h"
#include <array>
#include "globals.hpp"
#include "pid.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor leftBack(2);
pros::Motor leftFront(12);
pros::Motor rightBack(9);
pros::Motor rightFront(19);
pros::Motor leftIntake(15);
pros::Motor rightIntake(16);
pros::Motor indexer(3);
pros::Motor shooter(17);
pros::Imu inertial(1);
pros::ADIEncoder verticalEncoder1('E','F', false);
pros::ADIEncoder verticalEncoder2('A','B', false);
pros::ADIEncoder horizontalEncoder('C','D', true);
pros::ADIAnalogIn line_tracker('G');
std::array<double, 3> anglerPIDParams = {0.07, 0, 0};
std::array<double, 3> drivebasePIDParams = {23, 0, 0};
std::array<double, 3> turningPID = {160, 0, 170};
std::array<double, 3> adjustmentPIDParams = {2,0,0};

int maxSpeed = 12000;
int INDEX_THRESHOLD = 2600; // needs to be changed depending on ambient lighting conditions
double verticalOffset1 = 6.1675; // needs to be changed depending on vertical tracking wheel placement
double verticalOffset2 = 6.1675;
double horizontalOffset = 6.8075;
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

    inertial.reset();
    while (inertial.is_calibrating()) {
      pros::lcd::set_text(7, "IMU calibrating ...");
      iter += 10;
      pros::delay(10);
    }
    // should print about 2000 ms
    pros::lcd::set_text(7, "IMU is done calibrating (took %d ms)\n" + std::to_string(iter - time));

    verticalEncoder1.reset();
    verticalEncoder2.reset();
    horizontalEncoder.reset();

    master.clear();


}
