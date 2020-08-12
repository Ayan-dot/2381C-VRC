#include "main.h"
#include "posTracking.cpp"
#include "globals.hpp"
#include <array>
#include "pid.hpp"

long double currentx = 0, currenty = 0;

//initation of array pointer values
PID* rightdrivebasePIDController = new PID(
    &drivebasePIDParams[0],
    &drivebasePIDParams[1],
    &drivebasePIDParams[2]);

PID* leftdrivebasePIDController = new PID(
    &drivebasePIDParams[0],
    &drivebasePIDParams[1],
    &drivebasePIDParams[2]);

PID* strafePIDController = new PID(
    &strafePIDParams[0],
    &strafePIDParams[1],
    &strafePIDParams[2]);

PID* turningPIDController = new PID(
    &turningPID[0],
    &turningPID[1],
    &turningPID[2]);

// ODOM TASK
// make the global information on location and heading GLOBAL in this file
long double lastposR, currentposR, lastposL, currentposL, lastposH, currentposH, newAngle, lastAngle;

void vector_tasks_fn(void *param) {
    lastposR = 0, currentposR = 0; // variables to hold right vertical tracking wheel encoder position, in intervals of 10 ms
    lastposL = 0, currentposL = 0; // variables to hold left vertical tracking wheel encoder position, in intervals of 10 ms
    lastposH = 0, currentposH = 0; // horizontal counterparts of above variables
    newAngle = 0, lastAngle = 0; // angles taken by inertial sensor (IMU), in intervals of 20 ms
    // long double globalX = 0, globalY = 0; // global X and Y coordinates of the robot

    while (true) // control loop
    {
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
        // end of odom procedure

        pros::lcd::print(0, "X: %f", globalX); // prints X coord on brain
        pros::lcd::print(1, "Y: %f", globalY); // prints Y coord on brain
        pros::lcd::print(2, "I: %f", lastAngle); // prints angle on controller
        pros::lcd::print(3, "Iner: %f", inertial.get_rotation());
        pros::delay(20);      // runs loop every 10ms
    }
}

// motion translation PROCEDURES
void translationPID(long double x1, long double y1, long double x2, long double y2, long double heading, int time, int timeAllocated) {
  //////////////////////////////////////////////////////////////////
  /* Moves the Robot from point x1 and y1 (where it is right now) */
  /* to x2 and y2, while maintaining a constant heading           */
  //////////////////////////////////////////////////////////////////

  // Step 1, using x1, y1, x2, y2, and heading, let us calculate the STANDARD FORM equation of the lines
  // this can be done by simply finding a second pair of coordinates, as two points define a line
  const long double l = 0.1; // how far away in euclidean distance the second point we want is, can be any arbirary number

  // [ROBOT POSITION POINT] the first line is defined by x1 and y1, and the heading
  long double x1Other = x1 + l * sin(heading);
  long double y1Other = y1 + l * cos(heading);

  // [ROBOT DESTINATION POINT] the second line is defined by x1 and y2, and the heading rotated by 90 degrees, or pi/2 radians
  long double x2Other = x2 + l * sin(heading + pi / 2.0);
  long double y2Other = y2 + l * cos(heading + pi / 2.0);

  // With the above sets of new coordinates, we can define the STANDARD FORM of both lines

  // [ROBOT POSITION LINE PARALLEL TO HEADING]
  long double A1 = y1 - y1Other; // reversed because we have to assign negative sign to either numerator or denominator
  long double B1 = x1Other - x1;
  long double C1 = -(A1 * x1 + B1 * y1);

  // [ROBOT DESTINATION LINE PERPENDICULAR TO HEADING]
  long double A2 = y2 - y2Other;
  long double B2 = x2Other - x2;
  long double C2 = -(A2 * x2 + B2 * y2);

  // with the STANDARD FORM equations for both lines, let us calculate the intersection point of the two
  // let (x, y) denote the coordinate intersection of the two lines, represented in STANDARD FORM
  long double x = (B1*C2-B2*C1)/(B2*A1-B1*A2);
  long double y = (A1*C2-A2*C1)/(A2*B1-A1*B2);
  cout << "VARS: " << x1 << ' ' << y1 << ' ' << x1Other << ' ' << y1Other << ' ' << x2 << ' ' << y2 << ' ' << x2Other << ' ' << y2Other << ' ' << x <<  ' ' << y << '\n';

  // since Euclidean distance is always a positive value, we must know the direction in which the error is
  // to do this, we need to define another line
  // LINE: going through x1, y1, heading rotated 90 degrees anticlockwise

  // Define another point on 1
  long double x3Other = x1 + l * sin(heading - pi / 2.0);
  long double y3Other = y1 + l * cos(heading - pi / 2.0);

  // With this information, we can now determine the signage of the vertical and horizontal components (relative to heading)
  int horizontalSign = 1;
  int verticalSign = 1;

  // determine the relative direction of the horizontal component
  long double d1 = (x - x1) * (y1Other - y1) - (y - y1) * (x1Other - x1);
  if (d1 < 0) horizontalSign = -1;

  // ^ for the vertical component
  long double d2 = (x - x1) * (y3Other - y1) - (y - y1) * (x3Other - x1);
  if (d2 < 0) verticalSign = -1;

  // With the intersection point and direction, we can now calculate the Euclidean distance forwards relative the the robotics direction
  // and horizontally relative to the robot heading to get our x and y compoenents for our overall translation vector
  long double yComponent = sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1)) * verticalSign;
  long double xComponent = sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2)) * horizontalSign;
  cout << "COMPONENT VECTORS: " << yComponent << ' ' << xComponent << '\n';

   // To send this information to the motors, we pass the original encoder position, and the original encoder position
   // plus their respective local components for the PID loop
   long double leftDest = verticalEncoder1.get_value() + yComponent * (360 / (2.75 * pi));
   long double rightDest = verticalEncoder2.get_value() + yComponent * (360 / (2.75 * pi));
   long double horizontalDest = horizontalEncoder.get_value() + xComponent * (360 / (2.75 * pi));
   cout << "DESTINATIONS: " << leftDest << ' ' << rightDest << ' ' << horizontalDest << '\n';

   while (true) {
     // [Y COMPONENT] Forward Backward (relative to heading) voltage output
     long double yVoltageLeft = leftdrivebasePIDController->update(leftDest, verticalEncoder1.get_value(), -1);
     long double yVoltageRight = rightdrivebasePIDController->update(rightDest, verticalEncoder2.get_value(), -1);
     cout << "FB Volt: " << yVoltageLeft << ' ' << yVoltageRight << '\n';

     // [X COMPONENT] Side to side (relative to heading) voltage output
     long double xVoltage = strafePIDController->update(horizontalDest, horizontalEncoder.get_value(), 250);
     cout << "H Volt: " << xVoltage << '\n';
     pros::lcd::set_text(5, "xVolt: " + to_string(xVoltage));
     pros::lcd::set_text(6, "hDest: " + to_string(horizontalDest));
     pros::lcd::set_text(7, "hEnc: " + std::to_string(horizontalEncoder.get_value()));

     // [ANGLE] Heading correction voltage output
     long double angleVoltageLeft = turningPIDController->update(heading, lastAngle, -1);
     long double angleVoltageRight = -turningPIDController->update(heading, lastAngle, -1);
     cout << "Ang Volt: " << angleVoltageLeft << ' ' << angleVoltageRight << '\n';

     // to allow for a smoother acceleration gradient, time is considered in scaling the voltage outputs
     // so we limit speed the robot can travel in the first accelerationTime ms of acceleration
     long double currentTime = pros::millis() - time;
     long double coefTime = 1.0;
     if (currentTime < accelerationTime) {
       coefTime = sqrt(currentTime) / sqrt(accelerationTime);
     }

     // NOTE: On the robot, LEFT SIDE MOTORS are POSITIVE POWER FORWARDS MVMT, and RIGHT SIDE MOTORS are NEGATIVE POWER FORWARDS MVMT
     long double finalVoltageLeftFront = (yVoltageLeft + xVoltage + angleVoltageLeft) * coefTime;
     long double finalVoltageLeftBack = (yVoltageLeft - xVoltage + angleVoltageLeft) * coefTime;
     long double finalVoltageRightFront = -(yVoltageRight - xVoltage + angleVoltageRight) * coefTime;
     long double finalVoltageRightBack = -(yVoltageRight + xVoltage + angleVoltageRight) * coefTime;
     cout << "FINAL OUTPUT: " << finalVoltageLeftFront << ' ' << finalVoltageLeftBack << ' ' << finalVoltageRightFront << ' ' << finalVoltageRightBack << '\n';

     // Stop the robot when the maximum allowed time is reached
     if (pros::millis() - time >= timeAllocated) {
       cout << "PID STOPPED" << '\n';
       leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
       rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
       leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
       rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

       leftFront.move_velocity(0);
       rightFront.move_velocity(0);
       leftBack.move_velocity(0);
       rightBack.move_velocity(0);
       return;
     }

     // SEND IT! (the voltage to the drive motors)
     leftFront.move_voltage(finalVoltageLeftFront);
     leftBack.move_voltage(finalVoltageLeftBack);
     rightFront.move_voltage(finalVoltageRightFront);
     rightBack.move_voltage(finalVoltageRightBack);

     pros::delay(20);
   }
   return;
}

//pid function made for turning: it takes in 2 arguments, the angle you want to turn to and the direction in which you are turning
void turnPID(long double constInertial, DIRECTION direction) {
    //coefficients that direct the direction of rotation for the motors
    int cofLB = 1, cofLF = 1, cofRB = 1, cofRF = 1;
    // verticalEncoder.reset(); // ?
    long double original = inertial.get_rotation();
    //switch case changes motor rotation depending on direction you want to turn
    switch(direction) {
        case RIGHT:
            // cofRB = -1;
            // cofLB = -1;
            break;

        case LEFT:
            // cofRF = -1;
            // cofLF = -1;
            break;
    }

    //while loop initiating the PID function
    while(true) {
        //calls the PID class passing two arguments,
        long double voltage, negative;
        // debug: print out the voltage
        printf("Voltage: %lf\n", voltage);
        printf("Inertial: %lf\n", inertial.get_rotation());

        //gets motors to move
        leftFront.move_voltage(voltage * cofLF);
        leftBack.move_voltage(voltage * cofLB);
        rightFront.move_voltage(voltage * cofRF);
        rightBack.move_voltage(voltage * cofRB);

        //checks the inertial sensor reading at an error of +- 2 degrees and exits function once destination reached


        if (abs(inertial.get_rotation()) >= abs(constInertial) - 0.5 && abs(inertial.get_rotation()) <= abs(constInertial) + 0.5) {
            printf("TURNING STOPPED\n");
            voltage = 0;
            return;
        }
        // else {
        //
        //     if(constInertial < original) {
        //         negative = -inertial.get_rotation();
        //     }
        //     else {
        //         negative = inertial.get_rotation();
        //     }
        //
        //     voltage = turningPIDController->update(abs(constInertial), negative);
        // }

        pros::delay(10);
    }

}

void autonomous() {
    // wait for imu to calibrate
    pros::delay(3000);
    pros::Task position_task(vector_tasks_fn, (void*)"PROS", TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT,"Print X and Y Task");
    //846.21458975586392379449251804839‬ ticks = 24 inches
    translationPID(0.0, 0.0, 0.0, 96.0, 0.0, pros::millis(), 5000);
    translationPID(globalX, globalY, 0.0, 0.0, 0.0, pros::millis(), 10000);
    pros::delay(20);
}
