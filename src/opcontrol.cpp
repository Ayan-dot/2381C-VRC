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
    double lastposR = 0, currentposR = 0; // variables to hold right vertical tracking wheel encoder position, in intervals of 10 ms
    double lastposL = 0, currentposL = 0; // variables to hold left vertical tracking wheel encoder position, in intervals of 10 ms
    double lastposH = 0, currentposH = 0; // horizontal counterparts of above variables
    double newAngle = 0, lastAngle = 0; // angles taken by inertial sensor (IMU), in intervals of 10 ms
    // double globalX = 0, globalY = 0; // global X and Y coordinates of the robot
    while (true) // control loop 
    {

        // master.print(0, 0, "Rot: %f", inertial.get_rotation());
        leftFront = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		leftBack = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		rightFront = -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		rightBack = -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		pros::delay(20);

        currentposR = verticalEncoder2.get_value() * vertToInch; // reverses vertical encoder, finds position and converts to inches
        currentposL = verticalEncoder1.get_value() * vertToInch; // reverses vertical encoder, finds position and converts to inches
        currentposH = (horizontalEncoder.get_value() * horiToInch); // same function as above, horizontal counterpart
     
        positionTracking robotPos(lastAngle, currentposH, lastposH, currentposL, lastposL, currentposR, lastposR); // creates a Position tracking class, where math is done. 
        
        if (!isnan(robotPos.returnX()) || !isnan(robotPos.returnY())) // to avoid turning global coordinates into null values when calculations are initializing, conditional statement 
        {
            globalX += robotPos.returnX(); // adds the horizontal vector passed by the position tracking class to the global X coordinate
            globalY += robotPos.returnY(); // same function as above, vertical counterpart
        }
        
        lastposH = currentposH; // sets the last values for the function as the current values, to continue the loop
        lastposR = currentposR;
        lastposL = currentposL;  
        lastAngle = robotPos.returnOrient(); // ""
        
        // pros::lcd::print(0, "L: %f", verticalEncoder1.get_value()); // prints X coord on brain
        // pros::lcd::print(1, "R: %f", verticalEncoder2.get_value()); // prints Y coord on brain 
        // pros::lcd::print(2, "B: %f", horizontalEncoder.get_value());
        pros::lcd::set_text(1, "X:" + std::to_string(globalX));
        pros::lcd::set_text(2, "Y:" + std::to_string(globalY));
        pros::lcd::set_text(6, "A:" + std::to_string(lastAngle));
        pros::lcd::set_text(3, "L:" + std::to_string(verticalEncoder1.get_value()));
        pros::lcd::set_text(4, "R:" + std::to_string(verticalEncoder2.get_value()));
        pros::lcd::set_text(5, "B:" + std::to_string(horizontalEncoder.get_value()));
        // prints angle on controller
        
        // 1000 ticks is 24 inches on 2.75
       

        pros::delay(10);      // runs loop every 10ms
    }
}
