#include "main.h"
#include "posTracking.cpp"
#include "globals.hpp"
#include <array>
#include "pid.hpp"

double currentx = 0, currenty = 0;

//initation of array pointer values
PID* anglerPIDController = new PID(
    &anglerPIDParams[0],
    &anglerPIDParams[1],
    &anglerPIDParams[2]);

PID* rightdrivebasePIDController = new PID(
    &drivebasePIDParams[0],
    &drivebasePIDParams[1],
    &drivebasePIDParams[2]);

PID* leftdrivebasePIDController = new PID(
    &drivebasePIDParams[0],
    &drivebasePIDParams[1],
    &drivebasePIDParams[2]);

PID* adjustmentPIDController = new PID(
    &adjustmentPIDParams[0],
    &adjustmentPIDParams[1],
    &adjustmentPIDParams[2]);

PID* turningPIDController = new PID(
    &turningPID[0],
    &turningPID[1],
    &turningPID[2]);

//driving PID taking in 4 args, distance, direction
void forwardBackPID(double distance, DIRECTION direction, int time, int timeAllocated, double local_globalX, double local_globalY) {
    int cofLB = -1, cofLF = -1, cofRB = 1, cofRF = 1;
    // we know 2.75pi inches is 360 ticks
    // 2.75pi / 360 = distance / x
    // 360 / 2.75pi = x / distance

    switch (direction) {
        case REVERSE:
            // cofLB *= -1;
            // cofLF *= -1;
            // cofRB *= -1;
            // cofRF *= -1;
            distance *= -1;
            break;
          default:
            break;
    }

    distance = distance * (360 / (2.75 * pi));
    double leftDest = verticalEncoder1.get_value() + distance;
    double rightDest = verticalEncoder2.get_value() + distance;
    double ini = verticalEncoder1.get_value();
    std::cout << "LR: " << leftDest << ' ' << rightDest << ' ' << verticalEncoder1.get_value() << '\n';
    pros::lcd::set_text(5, "pid running...");

    bool leftReached = false;
    bool rightReached = false;

    while(true) {
        //converts inches to ticks
        //double encDistance = distance *(1.0/vertToInch);
        double leftadjustment = 0.0, rightadjustment = 0.0;

        /* determine the scaling if the robot is veering off course */
        double e1 = leftDest - verticalEncoder1.get_value();
        double e2 = rightDest - verticalEncoder2.get_value();
        if (abs(e1) > abs(e2)) {
          // beef up right side
          rightadjustment = (e1 - e2) * ANGLE_Kp;
        } else if (abs(e2) > abs(e1)) {
          leftadjustment = (e2 - e1) * ANGLE_Kp;
        }

        /* determine the horizontal strafe correction using global heading provided by ODOM */
        double coTopLeft = STRAFE_Kp;
        double coTopRight = STRAFE_Kp;

        //double currentRatio = (globalY - local_globalY) / (globalX - local_globalX);
        double currentRatio = (globalX - local_globalX) / abs(globalY - local_globalY + EPS);
        double e = abs(currentRatio);
        if (currentRatio > 0.0) {
          // increase X, decrease Y
          coTopLeft *= -1 * e;
          coTopRight *= e;
        } else {
          // decrease X, increase Y
          coTopRight *= -1 * e;
          coTopLeft *= e;
        }
        cout << "STRAFE DEBUG: " << currentRatio << ' ' << coTopLeft << ' ' << coTopRight << '\n';

        //voltage output for pid
        double voltageL = leftdrivebasePIDController->update(verticalEncoder1.get_value(), leftDest) + leftadjustment;
        double voltageR = rightdrivebasePIDController->update(verticalEncoder2.get_value(), rightDest) + rightadjustment;

        // modify the set voltages if necessary
        if (leftReached) voltageL = 0;
        if (rightReached) voltageR = 0;

        // limit speed the robot can travel in the first accelerationTime ms of acceleration
        double currentTime = pros::millis() - time;
        double coTime = 1.0;
        if (currentTime < accelerationTime) {
          coTime = sqrt(currentTime) / sqrt(accelerationTime);
        }


        //sets voltage to 0 when distance is reached (exit condition)
        // if (ini < leftDest) {
        //   if(verticalEncoder1.get_value() > leftDest - 5 || verticalEncoder2.get_value() > rightDest - 5) {
        //     if (verticalEncoder1.get_value() > leftDest - 5 && verticalEncoder2.get_value() > rightDest - 5) {
        //       pros::lcd::set_text(5, "pid stoppped");
        //       voltageL = 0;
        //       voltageR = 0;
        //       leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        //       rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        //       leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        //       rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        //
        //       leftFront.move_velocity(0);
        //       rightFront.move_velocity(0);
        //       leftBack.move_velocity(0);
        //       rightBack.move_velocity(0);
        //       return;
        //     } else if (verticalEncoder1.get_value() > leftDest - 5) {
        //       leftReached = true;
        //     } else {
        //       rightReached = true;
        //     }
        //   }
        // } else {
        //   if(verticalEncoder1.get_value() < leftDest + 5 || verticalEncoder2.get_value() < rightDest + 5) {
        //     if (verticalEncoder1.get_value() < leftDest + 5 && verticalEncoder2.get_value() < rightDest + 5) {
        //       pros::lcd::set_text(5, "pid stoppped");
        //       voltageL = 0;
        //       voltageR = 0;
        //       leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        //       rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        //       leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        //       rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        //
        //       leftFront.move_velocity(0);
        //       rightFront.move_velocity(0);
        //       leftBack.move_velocity(0);
        //       rightBack.move_velocity(0);
        //       return;
        //     } else if (verticalEncoder1.get_value() < leftDest + 5) {
        //       leftReached = true;
        //     } else {
        //       rightReached = true;
        //     }
        //   }
        // }
        if (pros::millis() - time >= timeAllocated) {
          voltageL = 0;
          voltageR = 0;
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

        // pros::lcd::set_text(6, "voltage: " + std::to_string(voltageR));
        // master.print(0, 0, "TL: %f", coTopLeft);
        // pros::delay(50);
        // master.print(1, 0, "VL: %f", voltageL * cofLF * coTime);

        //gets motors to move certain speed
        leftFront.move_voltage((voltageL * cofLF + coTopLeft) * coTime);
        leftBack.move_voltage((voltageL * cofLB + coTopRight) * coTime);
        rightFront.move_voltage((voltageR * cofRF + coTopRight * -1) * coTime);
        rightBack.move_voltage((voltageR * cofRB + coTopLeft * -1) * coTime);

        pros::delay(20);

    }
}

//pid function made for turning: it takes in 2 arguments, the angle you want to turn to and the direction in which you are turning
void turnPID(double constInertial, DIRECTION direction) {
    //coefficients that direct the direction of rotation for the motors
    int cofLB = 1, cofLF = 1, cofRB = 1, cofRF = 1;
    // verticalEncoder.reset(); // ?
    double original = inertial.get_rotation();
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
        double voltage, negative;
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

// ODOM TASK
// make the global information on location and heading GLOBAL in this file
double lastposR, currentposR, lastposL, currentposL, lastposH, currentposH, newAngle, lastAngle;

void vector_tasks_fn(void *param) {
    lastposR = 0, currentposR = 0; // variables to hold right vertical tracking wheel encoder position, in intervals of 10 ms
    lastposL = 0, currentposL = 0; // variables to hold left vertical tracking wheel encoder position, in intervals of 10 ms
    lastposH = 0, currentposH = 0; // horizontal counterparts of above variables
    newAngle = 0, lastAngle = 0; // angles taken by inertial sensor (IMU), in intervals of 20 ms
    // double globalX = 0, globalY = 0; // global X and Y coordinates of the robot

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
        //double trackingWheelWeight = 1.0 - min((abs(lastAngle) / k), 0.9);
        //double imuWeight = min((abs(lastAngle) / k), 0.9);
        double trackingWheelWeight = 0.01;
        double imuWeight = 0.99;
        double imuScaling = 0.9965;
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

void autonomous() {
    // wait for imu to calibrate
    pros::delay(3000);
    pros::Task position_task(vector_tasks_fn, (void*)"PROS", TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT,"Print X and Y Task");
    //846.21458975586392379449251804839‬ ticks = 24 inches

    int reee = globalX;
    cout << "Running first func" << '\n';
    forwardBackPID(96, FORWARD, pros::millis(), 5000, 0, globalY);
    pros::delay(1000);
    cout << "Running second func" << '\n';
    forwardBackPID(96, REVERSE, pros::millis(), 5000, 0, globalY);
    // //pros::delay(500);
    // //turnPID(270, LEFT);
    // pros::delay(500);
    // turnPID(180, RIGHT);
    //
    // pros::delay(500);
    // turnPID(0, LEFT);
    //
    // pros::delay(1000);
    // // motionPID(846,20, REVERSE);
    pros::delay(20);
}

// //combines turning and driving all into one function
// void combineAlgorithm(double targetX, double targetY, DIRECTION direction) {
//     //calls the class getting it to calculate desired angle and driving distance
//     motion moveTo(globalX, globalY, targetX, targetY);
//
//     //angle returned by class
//     double angle = moveTo.returnAngle();
//
//     //distance returned by class
//     double distance = moveTo.returnDistance();
//
//     //acounts for margin of error, there is no point turning such a small degree
//     if(!(angle < 0.5)) {
//         //calls the turning function
//         turnPID(inertial.get_heading(), direction);
//     }
//
//     //calls the driving PID to drive a certain distance
//     distance = distance *(1.0/vertToInch);
//     motionPID(distance, FORWARD,0,0);
//     while((targetX-globalX)>0.3||(targetY-globalY)>0.3){
//         // courseCorrect(targetX, targetY, FORWARD);
//         pros::delay(10);
//     }
// }
// void courseCorrect(double targetX, double targetY , DIRECTION direction){
//     motion moveCourse(globalX, globalY, targetX, targetY);
//     double angle = moveCourse.returnAngle();
//     double distance = moveCourse.returnDistance();
//     // turnCorrection turnCorrection(angle);
//     motionPID(distance, FORWARD, turnCorrection.returnL(), turnCorrection.returnR());
//
//
// }
