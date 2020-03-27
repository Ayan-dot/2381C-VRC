#include "main.h"
#include "globals.hpp"
#include "robot/intakes.cpp"
#include "posTracking.cpp"
#include <cmath>



void intake_tasks_fn(void *param)
{
    while (true)
    {
        Intakes run(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1), master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
        rightIntake.move_voltage(run.rightSpeed());
        leftIntake.move_voltage(run.leftSpeed());
    }
}

double currentx = 0, currenty = 0;

void drive_tasks_fn(void *param)
{
    while (true)
    {

        leftFront.move(-1 * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + 0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
        leftBack.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + -0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
        rightBack.move(-1 * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + -0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
        rightFront.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + 0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    }
    pros::delay(10);
}

void opcontrol()
{
    double lastpos = 0, currentpos = 0;
    double lastposH = 0, currentposH = 0;
    double newAngle = 0, lastAngle = 0;
    double globalX = 0, globalY = 0;
    while (true)
    {

        leftFront.move(-1 * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + 0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
        leftBack.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + -0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
        rightBack.move(-1 * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + -0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
        rightFront.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + 0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));

        // custom reset button for imu
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X))
        {
            inertial.reset();
            while (inertial.is_calibrating())
            {
                pros::delay(10);
            }
        }
        currentpos = verticalEncoder.get_value() * vertToInch;
        currentposH = -(horizontalEncoder.get_value() * horiToInch);
        newAngle = inertial.get_rotation()* imuToRad;
        positionTracking robotPos(newAngle, lastAngle, currentposH, lastposH, currentpos, lastpos);
        
        if (!isnan(robotPos.returnX()) || !isnan(robotPos.returnY()))
        {
            globalX += robotPos.returnX();
            globalY += robotPos.returnY();
        }
        lastposH = currentposH;
        lastpos = currentpos;
        lastAngle = newAngle;
        pros::lcd::print(0, "X: %f", globalX);
        pros::lcd::print(1, "Y: %f", globalY);
        pros::delay(10);
    }
}
