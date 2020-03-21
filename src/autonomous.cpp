// #include "main.h"
// #include "posTracking.cpp"
// #include "globals.hpp"
// #include <array>
// #include "pid.hpp"

// double currentx1 = 0;
// double currenty1 = 0;

// enum DIRECTION
// {
//   FORWARD,
//   REVERSE,
//   LEFT,
//   RIGHT,
//   LEFT_FORWARD,
//   LEFT_BACK,
//   RIGHT_FORWARD,
//   RIGHT_BACK,
//   TRANS_UP,
//   TRANS_DOWN
// };

// PID* anglerPIDController = new PID(
//     &anglerPIDParams[0],
//     &anglerPIDParams[1],
//     &anglerPIDParams[2]);

// PID* drivebasePIDController = new PID(
//     &drivebasePIDParams[0],
//     &drivebasePIDParams[1],
//     &drivebasePIDParams[2]);

// class drivePID {
//     private:
//         double encDistance = 0;
//         double voltage = 0;
//     public:
//         drivePID(double distance, double currentPosition) {
            
//             encDistance = (distance * 360) / (3.75 * pi);
//             voltage = drivebasePIDController->update(abs(encDistance), currentPosition);

//             if (abs(rightBack.get_position()) > abs(encDistance)) {
//                 pros::lcd::set_text(5, "Hi");
//                 voltage = 0;
//             }
//         }

//         double voltageOut() {
//             return voltage;
//         }
// };

// void motionPID(double distance, double constInertial, DIRECTION direction) {
//     verticalEncoder.reset();
//     int cofLB = 1, cofLF = 1, cofRB = 1, cofRF = 1;

//     switch (direction) {
//         case FORWARD:
//             cofRB = -1;
//             cofLF = -1;
//             break;

//         case REVERSE:
//             cofRF = -1;
//             cofLB = -1;
//             break;
//     }

//     while(true) {
//         drivePID speed(distance, verticalEncoder.get_value());
        
//         double voltage = speed.voltageOut();
        
//         leftFront.move_voltage(voltage * cofLF);
//         leftBack.move_voltage(voltage * cofLB);
//         rightFront.move_voltage(voltage * cofRF);
//         rightBack.move_voltage(voltage * cofRB);
        
//         if (voltage == 0) {
//             return;
//         }

//         pros::delay(10);
//     }
// }

// void turnPID(double constInertial, DIRECTION direction) {
//     int cofLB = 1, cofLF = 1, cofRB = 1, cofRF = 1;
//     verticalEncoder.reset();

//     switch(direction) {
//         case LEFT:
//             cofRB = -1;
//             cofLB = -1;
//             break;

//         case RIGHT:
//             cofRF = -1;
//             cofLF = -1;
//             break;
//     }

//     while(true) {
//         drivePID speed(constInertial, inertial.get_heading());
        
//         double voltage = speed.voltageOut();
        
//         leftFront.move_voltage(voltage * cofLF);
//         leftBack.move_voltage(voltage * cofLB);
//         rightFront.move_voltage(voltage * cofRF);
//         rightBack.move_voltage(voltage * cofRB);
        
//         if (inertial.get_heading() >= constInertial - 2 && inertial.get_heading() <= constInertial + 2) {
//             return;
//         }
//     }

// }

// void vector_tasks_fn(void *param) {
//     double newAngle = 0, oldAngle = 0, angDiff = 0;
//     double newEnc = 0, oldEnc = 0, enDiff = 0, newEnc1 = 0, oldEnc1 = 0, enDiff1 = 0;
//     double angOrientation = 0;
//     double oldX = 0, oldY = 0; // these values must be changed to reflect our coordinate system

//     while(true) {
//         // oldAngle = inertial.get_heading(); I dont believe this is needed, at least initially
//         oldEnc = verticalEncoder.get_value();
//         oldEnc1 = horizontalEncoder.get_value();
        
//         pros::delay(50);

//         newAngle = inertial.get_heading();
//         newEnc = verticalEncoder.get_value();
//         newEnc1 = verticalEncoder.get_value();

//         angDiff = newAngle - oldAngle;
//         enDiff = newEnc - oldEnc;
//         enDiff1 = newEnc1 = oldEnc1;


//         positionTracking findPos(oldAngle, newAngle, angDiff, enDiff, enDiff1, oldX, oldY);

//         pros::lcd::set_text(0, std::to_string((int) findPos.returnX()));
//         pros::lcd::set_text(1, std::to_string((int) findPos.returnY()));
//         oldX = findPos.returnX();
//         oldY = findPos.returnY();
        
//         currentx1 = findPos.returnX();
//         currenty1 = findPos.returnY();

//         oldAngle = findPos.returnOrientation();
//     }
// }
// void autonomous() {
//     pros::Task intake_task(vector_tasks_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,"Print X and Y Task");

// }

// void combineAlgorithm(double targetX, double targetY) {
//     motion moveTo(currentx1, currenty1, targetX, targetY);

//     double angle = moveTo.returnAngle();
//     double distance = moveTo.returnDistance();

//     if(angle != 0) {
//         turnPID(inertial.get_heading(), LEFT); //sample code direction left or right needs to be changed depending on situation
//     }

//     motionPID(distance, inertial.get_heading(), FORWARD);
// }
