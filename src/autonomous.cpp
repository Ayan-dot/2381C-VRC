#include "main.h"
#include "posTracking.cpp"
#include "globals.hpp"

void autonomous() {
    
}

void robotVector() {
    double newAngle = 0, oldAngle = 0, angDiff = 0;
    double newEnc = 0, oldEnc = 0, enDiff = 0;

    while(true) {
        oldAngle = inertial.get_heading();
        oldEnc = verticalEncoder.get_value();

        pros::delay(10);      

        newAngle = inertial.get_heading();
        newEnc = verticalEncoder.get_value();

        angDiff = newAngle - oldAngle;
        enDiff = newEnc - oldEnc;

        positionTracking findPos(oldAngle, newAngle, enDiff, horizontalEncoder.get_value());

        pros::lcd::set_text(0, std::to_string((int) findPos.returnX()));
        pros::lcd::set_text(1, std::to_string((int) findPos.returnY()));
    }
}