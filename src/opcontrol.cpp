#include "main.h"
#include "globals.hpp"
#include "robot/intakes.cpp"
#include "posTracking.cpp"

void intake_tasks_fn(void *param) {
   while(true) {
        Intakes run(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1), master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
        rightIntake.move_voltage(run.rightSpeed());
        leftIntake.move_voltage(run.leftSpeed());
   }
}

void printVector_tasks_fn(void *param) {
    double newAngle = 0, oldAngle = 0, angDiff = 0;
    double newEnc = 0, oldEnc = 0, enDiff = 0, newEnc1 = 0, oldEnc1 = 0, enDiff1 = 0;
    double angOrientation = 0;
    double oldX = 0, oldY = 0; // these values must be changed to reflect our coordinate system

    while(true) {
        // oldAngle = inertial.get_heading(); I dont believe this is needed, at least initially
        oldEnc = verticalEncoder.get_value();
        oldEnc1 = horizontalEncoder.get_value();
        pros::delay(10);

        newAngle = inertial.get_heading();
        newEnc = verticalEncoder.get_value();
        newEnc1 = verticalEncoder.get_value();

        angDiff = newAngle - oldAngle;
        enDiff = newEnc - oldEnc;
        enDiff1 = newEnc1 = oldEnc1;


        positionTracking findPos(oldAngle, newAngle, angDiff, enDiff, enDiff1, oldX, oldY);

        pros::lcd::set_text(0, "X Value" + std::to_string((int) findPos.returnX()));
        pros::lcd::set_text(1, "Y value" + std::to_string((int) findPos.returnY()));
        oldX = findPos.returnX();
        oldY = findPos.returnY();
        
        oldAngle = findPos.returnOrientation();
    }
}

void opcontrol() {
    pros::Task intake_task(intake_tasks_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,"Intake Task");
    pros::Task vector2_task(printVector_tasks_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,"Vector Task");
    
    while (true) {
        leftFront.move(-1 * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + 0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
        leftBack.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + -0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
        rightBack.move(-1 * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + -0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
        rightFront.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + 0.8 * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
        
    }
}