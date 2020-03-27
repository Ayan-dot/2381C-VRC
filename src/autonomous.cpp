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

PID* drivebasePIDController = new PID(
    &drivebasePIDParams[0],
    &drivebasePIDParams[1],
    &drivebasePIDParams[2]);

PID* turningPIDController = new PID(
    &turningPID[0],
    &turningPID[1],
    &turningPID[2]);

//driving PID taking in 5 arguments, distance you need to drive, target destination for inertial, direction, target x and y destination
void motionPID(double distance, double constInertial, DIRECTION direction, double x, double y) {
    verticalEncoder.reset();
    
    int cofLB = 1, cofLF = 1, cofRB = 1, cofRF = 1;

    switch (direction) {
        case FORWARD:
            cofRB = -1;
            cofLF = -1;
            break;

        case REVERSE:
            cofRF = -1;
            cofLB = -1;
            break;
    }

    while(true) {
        //converts inches to ticks
        double encDistance = (distance / (3.25 * pi) * 360);
        
        //voltage output for pid
        double voltage = drivebasePIDController->update(abs(encDistance), abs(verticalEncoder.get_value()));

        //sets voltage to 0 when distance is reached
        if(abs(verticalEncoder.get_value()) > abs(encDistance)) {
            pros::lcd::set_text(5, "Hi");
            voltage = 0;
        }

        //gets motors to move certain speed
        leftFront.move_voltage(voltage * cofLF);
        leftBack.move_voltage(voltage * cofLB);
        rightFront.move_voltage(voltage * cofRF);
        rightBack.move_voltage(voltage * cofRB);
        
        // if certain conditions are reached leave function
        if (voltage == 0 || ((currentx > x - 0.4 && currentx < x + 0.4) && (currenty > y - 0.4 && currenty < y + 0.4))) {
            return;
        }

        pros::delay(10);

    }
}

//pid function made for turning: it takes in 2 arguments, the angle you want to turn to and the direction in which you are turning
void turnPID(double constInertial, DIRECTION direction) {
    //coefficients that direct the direction of rotation for the motors
    int cofLB = 1, cofLF = 1, cofRB = 1, cofRF = 1;
    verticalEncoder.reset();

    //switch case changes motor rotation depending on direction you want to turn
    switch(direction) {
        case LEFT:
            cofRB = -1;
            cofLB = -1;
            break;

        case RIGHT:
            cofRF = -1;
            cofLF = -1;
            break;
    }

    //while loop initiating the PID function
    while(true) {
        //calls the PID class passing two arguments, 
        double voltage = turningPIDController->update(constInertial, inertial.get_heading());

        //gets motors to move
        leftFront.move_voltage(voltage * cofLF);
        leftBack.move_voltage(voltage * cofLB);
        rightFront.move_voltage(voltage * cofRF);
        rightBack.move_voltage(voltage * cofRB);
        
        //checks the inertial sensor reading at an error of +- 2 degrees and exits function once destination reached
        if (inertial.get_heading() >= constInertial - 2 && inertial.get_heading() <= constInertial + 2) {
            return;
        }
    }

}

//ayan's odometry system in opcontrol
void vector_tasks_fn(void *param) {
    double lastpos = 0, currentpos = 0;
    double lastposH = 0, currentposH = 0;
    double newAngle = 0, lastAngle = 0;
    double globalX = 0, globalY = 0;
    while (true)
    {
        currentpos = verticalEncoder.get_value() * vertToInch;
        currentposH = -(horizontalEncoder.get_value() * horiToInch);
        newAngle = inertial.get_rotation()* imuToRad;
        
        positionTracking robotPos(newAngle, lastAngle, currentposH, lastposH, currentpos, lastpos);
        
        if (!isnan(robotPos.returnX()) || !isnan(robotPos.returnY()))
        {
            globalX += robotPos.returnX();
            globalY += robotPos.returnY();
            
            currentx = globalX;
            currenty = globalY;
        }
        
        lastposH = currentposH;
        lastpos = currentpos;
        lastAngle = newAngle;
        
        pros::lcd::print(0, "X: %f", globalX);
        pros::lcd::print(1, "Y: %f", globalY);
        pros::delay(10);
    }
}

void autonomous() {
    pros::Task position_task(vector_tasks_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,"Print X and Y Task");

}

//combines turning and driving all into one function
void combineAlgorithm(double targetX, double targetY, DIRECTION direction) {
    //calls the class getting it to calculate desired angle and driving distance
    motion moveTo(currentx, currenty, targetX, targetY);

    //angle returned by class
    double angle = moveTo.returnAngle();
    
    //distance returned by class
    double distance = moveTo.returnDistance();

    //acounts for margin of error, there is no point turning such a small degree
    if(!(angle < 0.5)) {
        //calls the turning function
        turnPID(inertial.get_heading(), direction); 
    }
    
    //calls the driving PID to drive a certain distance
    motionPID(distance, inertial.get_heading(), FORWARD, targetX, targetY);
}
