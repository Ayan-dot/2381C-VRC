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
pros::ADIAnalogIn line_tracker1('G');
pros::ADIAnalogIn line_tracker2('H');
std::array<long double, 3> pointTurnPIDParams = {8000, 0, 40};
std::array<long double, 3> drivebasePIDParams = {52, 2, 180};
std::array<long double, 3> turningPID = {10000, 0, 0};
std::array<long double, 3> strafePIDParams = {20, 0.01, 90};

int maxSpeed = 12000;
int INDEX_THRESHOLD = 2700; // needs to be changed depending on ambient lighting conditions
const long double EPS = 1e-8;
long double accelerationTime = 750; // time in milliseconds given to accelerate
long double ANGLE_Kp = 150.0; // angle correction coefficient to help make sure the robot looks straight when moving straight, countering veering
long double STRAFE_Kp = 2000.0; // side-side correction cofficient to help make sure that the robot moves straight by shifting it side to side if it veers to one side back on track
long double verticalOffset1 = 6.1675; // needs to be changed depending on vertical tracking wheel placement
long double verticalOffset2 = 6.1675;
long double horizontalOffset = 6.8075;
long double horiToInch = (pi*2.75)/360.0;
long double vertToInch = (pi*2.75)/360.0;
long double imuToRad = pi/180.0;
long double globalX = 0, globalY = 0;

const long double pi = asin(1) * 2.0;

void disabled() {}

void competition_initialize() {
  initialize();
}


void initialize()
{

    int time = pros::millis();
    int iter = 0;

    pros::lcd::initialize();

    inertial.reset();
    while (inertial.is_calibrating()) {
      pros::lcd::set_text(7, "IMU calibrating ...");
      iter += 20;
      pros::delay(20);
    }
    // should print about 2000 ms
    pros::lcd::set_text(7, "IMU is done calibrating (took %d ms)\n" + std::to_string(iter - time));

    verticalEncoder1.reset();
    verticalEncoder2.reset();
    horizontalEncoder.reset();

    master.clear();


}
