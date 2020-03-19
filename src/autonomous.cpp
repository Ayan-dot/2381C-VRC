#include "main.h"
#include "posTracking.cpp"
#include "globals.hpp"

void vector_tasks_fn(void *param) {
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

        pros::lcd::set_text(0, std::to_string((int) findPos.returnX()));
        pros::lcd::set_text(1, std::to_string((int) findPos.returnY()));
        oldX = findPos.returnX();
        oldY = findPos.returnY();

        oldAngle = findPos.returnOrientation();
    }
}
void autonomous() {
    pros::Task intake_task(vector_tasks_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,"Print X and Y Task");
}
