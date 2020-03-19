#include "main.h"
#include "posTracking.cpp"
#include "globals.hpp"

void autonomous() {
    pros::Task intake_task(vector_tasks_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,"Print X and Y Task");
}

void vector_tasks_fn(void *param) {
    double newAngle = 0, oldAngle = 0, angDiff = 0;
    double newEnc = 0, oldEnc = 0, enDiff = 0;
    double angOrientation = 0;

    while(true) {
        oldAngle = inertial.get_heading();
        oldEnc = verticalEncoder.get_value();

        pros::delay(10);      

        newAngle = inertial.get_heading();
        newEnc = verticalEncoder.get_value();

        angDiff = newAngle - oldAngle;
        enDiff = newEnc - oldEnc;

        positionTracking findPos(oldAngle, newAngle, enDiff, horizontalEncoder.get_value(), angOrientation);

        pros::lcd::set_text(0, std::to_string((int) findPos.returnX()));
        pros::lcd::set_text(1, std::to_string((int) findPos.returnY()));

        angOrientation = findPos.returnOrientation();
    }
}