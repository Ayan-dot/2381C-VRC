// double currentx1 = 0;
// double currenty1 = 0;


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

//     double lastpos = 0, currentpos = 0;
//     double lastposH = 0, currentposH = 0;
//     double newAngle = 0, lastAngle = 0;
//     double globalX = 0, globalY = 0;
    
//     while (true)
//     {
//         currentpos = -verticalEncoder.get_value() * vertToInch;
//         currentposH = -(horizontalEncoder.get_value() * horiToInch);
//         newAngle = inertial.get_rotation()* imuToRad;
//         positionTracking robotPos(newAngle, lastAngle, currentposH, lastposH, currentpos, lastpos);
        
//         if (!isnan(robotPos.returnX()) || !isnan(robotPos.returnY()))
//         {
//             globalX += robotPos.returnX();
//             globalY += robotPos.returnY();
//         }
//         lastposH = currentposH;
//         lastpos = currentpos;
//         lastAngle = newAngle;
//         pros::lcd::print(0, "X: %f", globalX);
//         pros::lcd::print(1, "Y: %f", globalY);
//         pros::delay(10);
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
