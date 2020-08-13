#include "main.h"
#include "globals.hpp"
#include "robot/intakes.cpp"
#include "posTracking.cpp"
#include <cmath>

void opcontrol()
{
    long double lastposR = 0, currentposR = 0; // variables to hold right vertical tracking wheel encoder position, in intervals of 10 ms
    long double lastposL = 0, currentposL = 0; // variables to hold left vertical tracking wheel encoder position, in intervals of 10 ms
    long double lastposH = 0, currentposH = 0; // horizontal counterparts of above variables
    long double newAngle = 0, lastAngle = 0; // angles taken by inertial sensor (IMU), in intervals of 10 ms

    long double k = 3.0;

    // long double globalX = 0, globalY = 0; // global X and Y coordinates of the robot
    // wait for imu to calibrate
    pros::delay(3000);
    while (true) // control loop
    {

        // master.print(0, 0, "Rot: %f", inertial.get_rotation());
        leftFront = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) + master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        leftBack = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) - master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) + master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        rightFront = -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) + master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        rightBack = -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) - master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) + master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		    //pros::delay(20);

        // ODOMETRY PROCEDURE //
        /*
        * Featuring imu averaging (more like imu domination)
        */
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
        //long double trackingWheelWeight = 1.0 - min((abs(lastAngle) / k), 0.9);
        //long double imuWeight = min((abs(lastAngle) / k), 0.9);
        long double trackingWheelWeight = 0.01;
        long double imuWeight = 0.99;
        long double imuScaling = 0.9965;
        lastAngle = robotPos.returnOrient() * trackingWheelWeight + inertial.get_rotation() * (pi / 180.0) * imuWeight * imuScaling;
        lastposH = currentposH; // sets the last values for the function as the current values, to continue the loop
        lastposR = currentposR;
        lastposL = currentposL;

        pros::lcd::set_text(1, "X:" + std::to_string(globalX));
        pros::lcd::set_text(2, "Y:" + std::to_string(globalY));
        pros::lcd::set_text(6, "A:" + std::to_string(lastAngle));
        pros::lcd::set_text(3, "L:" + std::to_string(verticalEncoder1.get_value()));
        pros::lcd::set_text(4, "R:" + std::to_string(verticalEncoder2.get_value()));
        pros::lcd::set_text(5, "B:" + std::to_string(horizontalEncoder.get_value()));
        pros::lcd::set_text(7, "I: " + std::to_string(inertial.get_rotation() * imuScaling));


        // INTAKE CONTROLS //
        /*
        * L1 - Intake
        * L2 - outtake
        * Y - Only left intake intakes
        * X - Only right intake intakes
        */
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
          // intake
          leftIntake.move_velocity(200);
          rightIntake.move_velocity(-200);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
          leftIntake.move_velocity(-200);
          rightIntake.move_velocity(200);
        }
        else {
          leftIntake.move_velocity(-0);
          rightIntake.move_velocity(-0);
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
          leftIntake.move_velocity(200);
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
          rightIntake.move_velocity(-200);
        }

        // INDEXING CONTROLS //
        // R1 - indexes the first ball to shooting bay, and indexes the remaining accordingly

        shooter.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        // we know the ball is indexed if the line tracker reports <= INDEX_THRESHOLD
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
          if (line_tracker1.get_value() > INDEX_THRESHOLD) {
            // we do not have a ball properly indexed yet
            indexer.move_velocity(-200);
            shooter.move_velocity(10);
          } else {
            shooter.move_velocity(0);
            if (line_tracker2.get_value() > INDEX_THRESHOLD) {
              indexer.move_velocity(-200);
            }
          }
        } else {
          indexer.move_velocity(0);
          shooter.move_velocity(0);
        }

        // SHOOTING CONTROLS //
        // R2 - fires balls
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
          // shoots indexed ball
          shooter.move_velocity(180);
        } else {
          if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && line_tracker1.get_value() > INDEX_THRESHOLD) {

          } else
            shooter.move_velocity(0);
        }

        // DISCARDING CONTROLS //
        // L1 + L2
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
          shooter.move_velocity(-150);
          leftIntake.move_velocity(0);
          rightIntake.move_velocity(0);
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
          int iter = 0;
          inertial.reset();
          while (inertial.is_calibrating()) {
            pros::lcd::set_text(7, "IMU calibrating ...");
            iter += 10;
            pros::delay(10);
          }
          lastAngle = robotPos.returnOrient();
        }

        pros::delay(20);      // runs loop every 10ms
    }
}
