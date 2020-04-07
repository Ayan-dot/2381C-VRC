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

        master.print(0, 0, "Rot: %f", inertial.get_rotation());

         currentposR = -verticalEncoder2.get_value() * vertToInch; // reverses vertical encoder, finds position and converts to inches
        currentposL = -verticalEncoder1.get_value() * vertToInch; // reverses vertical encoder, finds position and converts to inches
        currentposH = -(horizontalEncoder.get_value() * horiToInch); // same function as above, horizontal counterpart
        newAngle = (inertial.get_rotation()) * imuToRad; // gets inertial angle, converts to radians. Use of rotation as opposed to heading is to account for vector math with negative angles.
        positionTracking robotPos(newAngle, lastAngle, currentposH, lastposH, currentposL, lastposL, currentposR, lastposR); // creates a Position tracking class, where math is done. 
        
        if (!isnan(robotPos.returnX()) || !isnan(robotPos.returnY())) // to avoid turning global coordinates into null values when calculations are initializing, conditional statement 
        {
            globalX += robotPos.returnX(); // adds the horizontal vector passed by the position tracking class to the global X coordinate
            globalY += robotPos.returnY(); // same function as above, vertical counterpart
        }
        
        lastposH = currentposH; // sets the last values for the function as the current values, to continue the loop
        lastposR = currentposR;
        lastposL - currentposL;  
        lastAngle = robotPos.returnOrient(); // ""
        
        pros::lcd::print(0, "X: %f", globalX); // prints X coord on brain
        pros::lcd::print(1, "Y: %f", globalY); // prints Y coord on brain 
        master.print(1,0, "I: %f", lastAngle); // prints angle on controller
        
        pros::delay(50);

        master.print(0, 0, "Iner: %f", inertial.get_rotation());        
        pros::delay(10);      // runs loop every 10ms
    }
}
