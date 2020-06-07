#include "main.h"
#include "globals.hpp"
#include "robot/intakes.cpp"
#include "posTracking.hpp"
#include <cmath>
#include "autoSelect/selection.h"
#include "lvgl/lvglPage.hpp"


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
    odomBoi init();
    init.reset();
    
    double lastposR = 0, currentposR = 0; // variables to hold right vertical tracking wheel encoder position, in intervals of 10 ms
    double lastposL = 0, currentposL = 0; // variables to hold left vertical tracking wheel encoder position, in intervals of 10 ms
    double lastposH = 0, currentposH = 0; // horizontal counterparts of above variables
    double newAngle = 0, lastAngle = 0; // angles taken by inertial sensor (IMU), in intervals of 10 ms
    double globalX = 0, globalY = 0; // global X and Y coordinates of the robot
    
    // wait for imu to calibrate
    pros::delay(3000);

    while (true) // control loop
    {

        leftFront = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) + master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        leftBack = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) - master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) + master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        rightFront = -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) + master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        rightBack = -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) - master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) + master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		    //pros::delay(20);

        currentposR = verticalEncoder2.get_value() * vertToInch; // reverses vertical encoder, finds position and converts to inches
        currentposL = verticalEncoder1.get_value() * vertToInch; // reverses vertical encoder, finds position and converts to inches
        currentposH = horizontalEncoder.get_value() * horiToInch; // same function as above, horizontal counterpart

        positionTracking robotPos(lastAngle, currentposH, lastposH, currentposL, lastposL, currentposR, lastposR); // creates a Position tracking class, where math is done.

        if (!isnan(robotPos.returnX()) || !isnan(robotPos.returnY())) // to avoid turning global coordinates into null values when calculations are initializing, conditional statement
        {
            globalX += robotPos.returnX(); // adds the horizontal vector passed by the position tracking class to the global X coordinate
            globalY += robotPos.returnY(); // same function as above, vertical counterpart
        }
        // since we are doing inertial averaging, lastAngle is modified
        // perform a weighted average
        double trackingWheelWeight = 0.5;
        double imuWeight = 0.5;
        
        lastAngle = robotPos.returnOrient() * trackingWheelWeight + inertial.get_rotation() * (pi / 180.0) * imuWeight;
        lastposH = currentposH; // sets the last values for the function as the current values, to continue the loop
        lastposR = currentposR;
        lastposL = currentposL;
         // ""

        init.setData(globalX, globalY, lastAngle);

        pros::lcd::set_text(1, "X:" + std::to_string(globalX));
        pros::lcd::set_text(2, "Y:" + std::to_string(globalY));
        pros::lcd::set_text(6, "A:" + std::to_string(lastAngle));
        pros::lcd::set_text(3, "L:" + std::to_string(verticalEncoder1.get_value()));
        pros::lcd::set_text(4, "R:" + std::to_string(verticalEncoder2.get_value()));
        pros::lcd::set_text(5, "B:" + std::to_string(horizontalEncoder.get_value()));
        pros::lcd::set_text(7, "I: " + std::to_string(inertial.get_rotation()));
        // prints angle on controller

        // 1000 ticks is 24 inches on 2.75


        pros::delay(20);      // runs loop every 10ms
    }
}