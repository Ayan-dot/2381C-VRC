/*
  ___  ____   ___  __  _____
 |__ \|___ \ / _ \/_ |/ ____|
    ) | __) | (_) || | |
   / / |__ < > _ < | | |
  / /_ ___) | (_) || | |____
 |____|____/ \___/ |_|\_____|

All code is the property of 2381C, Kernel Bye. ANY UNAUTHORIZED REPRODUCTION
OR DISTRIBUTION OF THIS CODE IS STRICTLY FORBIDDEN. Please contact team 2381C
directly with any questions, concerns or suggestions you may have.

autonomous.cpp [contains]:
  * 120 programming skills routine
  * all functions used for programming skills routine

NOTE: All relevant mathematical calculations (odometry and motion) are documented in extensive detail in our paper regarding robot motion
  * https://drive.google.com/file/d/1zBMroM90nDU6iHqsbI_qOgd120M7x-rd/view

*/

// Necessary imports from other files within the src directory, header files in the include directory, and C++ libraries
#include "main.h"
#include "posTracking.cpp"
#include "globals.hpp"
#include <array>
#include "pid.hpp"

// Initation of array pointer values (kp at index 0, ki at index 1, kd at index 2) for PID class
// Kp, Ki, and Kd constants at those indexes are specified in globals.cpp, where other tunable constants reside
PID *rightdrivebasePIDController = new PID(
    &drivebasePIDParams[0],
    &drivebasePIDParams[1],
    &drivebasePIDParams[2]);

PID *leftdrivebasePIDController = new PID(
    &drivebasePIDParams[0],
    &drivebasePIDParams[1],
    &drivebasePIDParams[2]);

PID *strafePIDController = new PID(
    &strafePIDParams[0],
    &strafePIDParams[1],
    &strafePIDParams[2]);

PID *turningPIDController = new PID(
    &turningPID[0],
    &turningPID[1],
    &turningPID[2]);

PID *pointTurnPIDController = new PID(
    &pointTurnPIDParams[0],
    &pointTurnPIDParams[1],
    &pointTurnPIDParams[2]);

//***********//
// ODOM TASK //
//***********//
// make the odometry information on location and heading GLOBAL within this file
long double lastposR, currentposR, lastposL, currentposL, lastposH, currentposH, newAngle, lastAngle;

void vector_tasks_fn(void *param)
{
  lastposR = 0, currentposR = 0; // variables to hold right vertical tracking wheel encoder position, in intervals of 10 ms
  lastposL = 0, currentposL = 0; // variables to hold left vertical tracking wheel encoder position, in intervals of 10 ms
  lastposH = 0, currentposH = 0; // horizontal counterparts of above variables
  newAngle = 0, lastAngle = 0;   // angles taken by inertial sensor (IMU), in intervals of 20 ms
  // long double globalX = 0, globalY = 0; // global X and Y coordinates of the robot

  while (true) // control loop
  {
    // ODOMETRY PROCEDURE //
    /*
    * Featuring imu averaging
    */
    currentposR = verticalEncoder2.get_value() * vertToInch;  // reverses vertical encoder, finds position and converts to inches
    currentposL = verticalEncoder1.get_value() * vertToInch;  // reverses vertical encoder, finds position and converts to inches
    currentposH = horizontalEncoder.get_value() * horiToInch; // same function as above, horizontal counterpart

    positionTracking robotPos(lastAngle, currentposH, lastposH, currentposL, lastposL, currentposR, lastposR); // creates a Position tracking class for odometry, where math is done (posTracking.cpp).

    if (!isnan(robotPos.returnX()) || !isnan(robotPos.returnY())) // to avoid turning global coordinates into null values when calculations are initializing, conditional statement
    {
      globalX += robotPos.returnX(); // adds the horizontal vector passed by the position tracking class to the global X coordinate
      globalY += robotPos.returnY(); // same function as above, vertical counterpart
    }
    // since we are doing inertial averaging, lastAngle is modified
    // perform a weighted average
    long double trackingWheelWeight = 0.01;
    long double imuWeight = 0.99;
    long double imuScaling = 0.9965;
    lastAngle = robotPos.returnOrient() * trackingWheelWeight + inertial.get_rotation() * (pi / 180.0) * imuWeight * imuScaling;
    lastposH = currentposH; // sets the last values for the function as the current values, to continue the loop
    lastposR = currentposR;
    lastposL = currentposL;
    // end of odom procedure

    pros::lcd::print(0, "X: %f", globalX);   // prints X coord on brain
    pros::lcd::print(1, "Y: %f", globalY);   // prints Y coord on brain
    pros::lcd::print(2, "I: %f", lastAngle); // prints angle on brain
    pros::lcd::print(3, "Iner: %f", inertial.get_rotation());
    pros::delay(20); // runs loop every 20ms
  }
}

//**************//
// ROBOT DEPLOY //
//**************//
void deploy() // deploy function (used at the beginning of autonomous)
{
  // shakes robot fowards and back slightly, while moving preload up to contact the hood to deploy the hood with the intakes
  leftFront.move_velocity(160);
  leftBack.move_velocity(160);
  rightFront.move_velocity(-160);
  rightBack.move_velocity(-160);
  shooter.move_velocity(180);
  pros::delay(90);
  leftFront.move_velocity(-150);
  leftBack.move_velocity(-150);
  rightFront.move_velocity(150);
  rightBack.move_velocity(150);
  shooter.move_velocity(-200);
  pros::delay(110);
  leftFront.move_velocity(0);
  leftBack.move_velocity(0);
  rightFront.move_velocity(0);
  rightBack.move_velocity(0);
  shooter.move_velocity(0);
}

//****************//
// STATIC DESCORE //
//****************//
void descoreProcedureStatic(int time) // descore function when robot is not moving (not undergoing translationPID)
{ //time based static descore function
  while (line_tracker1.get_value() <= INDEX_THRESHOLD) // INDEX_THRESHOLD constant is adjustable in globals.cpp depending on ambient lighting conditions
  {
    indexer.move_velocity(200);
    shooter.move_velocity(-85);
    leftIntake.move_velocity(-85);
    rightIntake.move_velocity(85);
  }

  indexer.move_velocity(-180);
  shooter.move_velocity(-160);
  leftIntake.move_velocity(155);
  rightIntake.move_velocity(-155);

  pros::delay(time);
  indexer.move_velocity(0);
  shooter.move_velocity(0);
  leftIntake.move_velocity(0);
  rightIntake.move_velocity(0);
}

//****************//
// MOVING DESCORE //
//****************//
void descoreProcedureMoving(int numBalls) // continuous descore function when robot is moving (undergoing translationPID)
{ //continuous descore function (when moving)
  if (numBalls >= 1)
  {
    if (line_tracker1.get_value() <= INDEX_THRESHOLD) // if balls are fully indexed, un-index them to avoid jamming
    {

      indexer.move_velocity(200);
      shooter.move_velocity(-85);
      leftIntake.move_velocity(-85);
      rightIntake.move_velocity(85);
    }
    else
    {

      indexer.move_velocity(-180);
      shooter.move_velocity(-160);
    }
  }
  else
  {

    indexer.move_velocity(0);
    shooter.move_velocity(0);
  }
  return;
}

//*******************************//
// INDEXING AND INTAKE PROCEDURE //
//*******************************//
void intakeIndexingProcedure(bool runIntakes, bool runIndexer, bool runIntakesBackwards, int numShot)
{
  if (runIntakes || runIntakesBackwards)
  {
    leftIntake.move_velocity((runIntakes ? 200 : -100)); // define intake movement speed
    rightIntake.move_velocity((runIntakes ? -200 : 100));
  }
  if (numShot > 0)
  {
    descoreProcedureMoving(numShot); // if balls to be shot are a non-zero number
  }
  else if (runIndexer)
  {
    if (line_tracker1.get_value() > INDEX_THRESHOLD)
    {
      // we do not have a ball properly indexed yet
      indexer.move_velocity(-200);
      shooter.move_velocity(10);
    }
    else
    {
      shooter.move_velocity(0);
      shooter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

      if (line_tracker2.get_value() > INDEX_THRESHOLD)
      {
        indexer.move_velocity(-180);
      }
      else
      {
        indexer.move_velocity(0);
      }
    }
  }
  else if (runIntakesBackwards)
  {
    indexer.move_velocity(200);
  }
  else
  {
    indexer.move_velocity(0);
  }

  return;
}

//********************//
// SHOOTING PROCEDURE //
//********************//
void shootingProcedure(bool slowrun)
{
  int time = pros::millis();
  while (line_tracker1.get_value() > INDEX_THRESHOLD && pros::millis() - time < 1000)
  {
    // we do not have a ball properly indexed yet
    indexer.move_velocity(-200);
    shooter.move_velocity(20);
  }
  pros::delay(50);
  indexer.move_velocity(0);
  shooter.move_velocity(180);
  if (slowrun)
  {
    leftIntake.move_velocity(60);
    rightIntake.move_velocity(-60);
  }
  pros::delay(500);
  shooter.move_velocity(0);
  return;
}

//*********************************//
// MOTION ALGORITHM STATE FUNCTION //
//*********************************//

// arguments: target x position, target y position, target robot heading, current time, time given to complete the motion, intakes off/on, indexer off/on, reverse intakes off/on, number of balls to shoot, max voltage output to drivebase motors
void translationPID(long double x2, long double y2, long double heading, int time, int timeAllocated, bool runIntakes, bool runIndexer, bool runIntakesBackwards, int numDescore, int numShoot, double maxVolt)
{
  int ballShot = 0;
  int numBalls = numDescore;
  int maxTime = 0;
  if (line_tracker1.get_value() > INDEX_THRESHOLD) // if the balls are not fully indexed (defining amount of time to descore)
  {
    maxTime = 500.0 * numDescore;
  }
  else
  {
    maxTime = 550.0 * numDescore;
  }

  while (true)
  {
    // Run the intaking indexing procedure if commanded to do so

    if (runIntakes || runIndexer || runIntakesBackwards) //if any intaking / conveying function is called
    {
      if (pros::millis() - time < maxTime) // as long as max time for descore is not exceeded
      {
        numBalls = numDescore; //continue to set the descore variable to a non-zero value
      }
      else
      {
        numBalls = 0;
      }
      intakeIndexingProcedure(runIntakes, runIndexer, runIntakesBackwards, numBalls);
      if (ballShot < numShoot) // if the number of balls to shoot is greater than the balls already shot
      {
        shootingProcedure(false);
        ballShot++;
      }
    }
    else
    {
      leftIntake.move_velocity(0);
      rightIntake.move_velocity(0);
      indexer.move_velocity(0);
      shooter.move_velocity(0);
    }

    //////////////////////////////////////////////////////////////////
    /* Moves the Robot from point x1 and y1 (where it is right now) */
    /* to x2 and y2, while maintaining a constant heading           */
    //////////////////////////////////////////////////////////////////

    // UPDATE: translation PID now accounts for turning while strafing

    /**********/
    /** MATH **/
    /**********/

    // Current robot x and y position is given by odometry (global variables)
    long double x1 = globalX;
    long double y1 = globalY;

    // Step 1, using x1, y1, x2, y2, and heading, let us calculate the STANDARD FORM equation of the lines
    // this can be done by simply finding a second pair of coordinates, as two points define a line
    const long double l = 100.0; // how far away in euclidean distance the second point we want is, can be any arbirary number

    // [ROBOT POSITION POINT] the first line is defined by x1 and y1, and the heading. We now use this to define a second point on the line
    long double x1Other = x1 + l * sin(lastAngle);
    long double y1Other = y1 + l * cos(lastAngle);

    // [ROBOT POSITION LINE PARALLEL TO HEADING (line NT)]
    long double A1 = y1 - y1Other; // reversed because we have to assign negative sign to either numerator or denominator
    long double B1 = x1Other - x1;
    long double C1 = -(A1 * x1 + B1 * y1);

    // [ROBOT DESTINATION LINE PERPENDICULAR PREVIOUS LINE (line MT)]
    long double A2 = B1;
    long double B2 = -A1;
    long double C2 = -(A2 * x2 + B2 * y2);

    // with the STANDARD FORM equations for both lines, let us calculate the intersection point of the two
    // let (x, y) denote the coordinate intersection of the two lines, represented in STANDARD FORM
    long double x = (B1 * C2 - B2 * C1) / (B2 * A1 - B1 * A2);
    long double y = (A1 * C2 - A2 * C1) / (A2 * B1 - A1 * B2);
    //cout << "VARS: " << x1 << ' ' << y1 << ' ' << x1Other << ' ' << y1Other << ' ' << x2 << ' ' << y2 << ' ' << x2Other << ' ' << y2Other << ' ' << x <<  ' ' << y << '\n';

    // since Euclidean distance is always a positive value, we must know the direction in which the error is
    // to do this, we need to define another line
    // LINE: going through x1, y1, heading rotated 90 degrees anticlockwise

    // Define another point on the line perpendicular to line NT in calculations
    long double x3Other = x1 + l * sin(heading - pi / 2.0);
    long double y3Other = y1 + l * cos(heading - pi / 2.0);

    // With this information, we can now determine the signage of the vertical and horizontal components (relative to heading)
    int horizontalSign = 1;
    int verticalSign = 1;

    // determine the relative direction of the horizontal component
    long double d1 = (x2 - x1) * (y1Other - y1) - (y2 - y1) * (x1Other - x1);
    if (d1 < 0)
      horizontalSign = -1;
    cout << "D1: " << d1 << '\n';

    // same for the vertical component
    long double d2 = (x - x1) * (y3Other - y1) - (y - y1) * (x3Other - x1);
    if (d2 < 0)
      verticalSign = -1;
    cout << "D2: " << d2 << '\n';

    // With the intersection point and direction, we can now calculate the Euclidean distance forwards relative to the robot's direction
    // and horizontally relative to the robot heading to get our x and y components (errors) for our overall translation vector
    long double yComponent = abs(sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1))) * verticalSign;
    long double xComponent = abs(sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2))) * horizontalSign;
    cout << "COMPONENT VECTORS: " << yComponent << ' ' << xComponent << '\n';

    // To send this information to the motors, we pass the original encoder position, and the destination encoder position
    // plus their respective local components for the PID loop. We use ticks to inch conversion as function arguments are in inches
    long double leftDest = verticalEncoder1.get_value() + yComponent * (360 / (2.75 * pi));
    long double rightDest = verticalEncoder2.get_value() + yComponent * (360 / (2.75 * pi));
    long double horizontalDest = horizontalEncoder.get_value() + xComponent * (360 / (2.75 * pi));
    cout << "DESTINATIONS: " << leftDest << ' ' << rightDest << ' ' << horizontalDest << '\n';

    /*********/
    /** PID **/
    /*********/

    // [Y COMPONENT] Forward Backward (relative to heading) voltage output
    long double yVoltageLeft = leftdrivebasePIDController->update(leftDest, verticalEncoder1.get_value(), -1);
    long double yVoltageRight = rightdrivebasePIDController->update(rightDest, verticalEncoder2.get_value(), -1);
    cout << "FB Volt: " << yVoltageLeft << ' ' << yVoltageRight << '\n';

    // [X COMPONENT] Side to side (relative to heading) voltage output
    long double xVoltage = strafePIDController->update(horizontalDest, horizontalEncoder.get_value(), -1);
    cout << "H Volt: " << xVoltage << '\n';
    pros::lcd::set_text(5, "lDest: " + to_string(leftDest));
    pros::lcd::set_text(6, "rDest: " + to_string(rightDest));
    pros::lcd::set_text(7, "hDest: " + to_string(horizontalDest));

    // [ANGLE] Heading correction voltage output
    long double angleVoltageLeft = turningPIDController->update(heading, lastAngle, -1);
    long double angleVoltageRight = -turningPIDController->update(heading, lastAngle, -1);
    cout << "Ang Volt: " << angleVoltageLeft << ' ' << angleVoltageRight << '\n';

    // to allow for a smoother acceleration gradient, time is considered in scaling the voltage outputs
    // so we limit speed the robot can travel in the first accelerationTime ms of acceleration
    // accelerationTime, like other tunable constants, is adjusted in globals.cpp
    long double currentTime = pros::millis() - time;
    long double coefTime = 1.0;
    if (currentTime < accelerationTime)
    {
      coefTime = sqrt(currentTime) / sqrt(accelerationTime); // based off of the square root curve approaching the target voltage
    }

    // NOTE: On the robot, LEFT SIDE MOTORS are POSITIVE POWER FORWARDS MVMT, and RIGHT SIDE MOTORS are NEGATIVE POWER FORWARDS MVMT
    long double finalVoltageLeftFront = (yVoltageLeft + xVoltage + angleVoltageLeft) * coefTime;
    long double finalVoltageLeftBack = (yVoltageLeft - xVoltage + angleVoltageLeft) * coefTime;
    long double finalVoltageRightFront = -(yVoltageRight - xVoltage + angleVoltageRight) * coefTime;
    long double finalVoltageRightBack = -(yVoltageRight + xVoltage + angleVoltageRight) * coefTime;

    // As we do not want to send voltages higher than MAX_VOLTAGE to the motors, scale all of the motors accordingly
    long double maxOutput = max({finalVoltageLeftFront, finalVoltageLeftBack, finalVoltageRightFront, finalVoltageRightBack});
    if (maxOutput > maxVolt)
    {
      long double scaling = maxVolt / maxOutput; // scale down all of the motor outputs by a fraction >= 0 < 1 such that the max voltage does not exceed MAX_VOLTAGE
      finalVoltageLeftFront *= scaling;
      finalVoltageLeftBack *= scaling;
      finalVoltageRightFront *= scaling;
      finalVoltageRightBack *= scaling;
    }

    cout << "FINAL OUTPUT: " << finalVoltageLeftFront << ' ' << finalVoltageLeftBack << ' ' << finalVoltageRightFront << ' ' << finalVoltageRightBack << '\n';

    // Stop the robot when the maximum allowed time is reached

    if (pros::millis() - time >= timeAllocated)
    {
      cout << "PID STOPPED" << '\n';
      leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

      leftFront.move_velocity(0);
      rightFront.move_velocity(0);
      leftBack.move_velocity(0);
      rightBack.move_velocity(0);

      // stop the indexing and intakes
      leftIntake.move_velocity(0);
      rightIntake.move_velocity(0);
      indexer.move_velocity(0);
      shooter.move_velocity(0);
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

//pid function made for point turning: it takes in 2 arguments, the angle you want to turn to and the direction in which you are turning
void turnPID(long double targetAngle, int time, int timeAllocated)
{
  //while loop initiating the PID function
  while (true)
  {
    // [ANGLE] Heading correction voltage output (set up a PID for both left and right sides of the x drive using motor encoder values for current and destination)
    long double angleVoltageLeft = pointTurnPIDController->update(targetAngle, lastAngle, -1);
    long double angleVoltageRight = pointTurnPIDController->update(targetAngle, lastAngle, -1);

    pros::lcd::set_text(5, "Ang: " + to_string(lastAngle));
    leftFront.move_voltage(angleVoltageLeft);
    leftBack.move_voltage(angleVoltageLeft);
    rightFront.move_voltage(angleVoltageRight);
    rightBack.move_voltage(angleVoltageRight);

    // if the allocated time is exceeded, stop the robot, and exit the function
    if (pros::millis() - time > timeAllocated)
    {
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
    pros::delay(20);
  }
}

void autonomous()
{

  // The lines below are a combination of functions that have been used to create our 120 point programming skills run.
  pros::Task position_task(vector_tasks_fn, (void *)"PROS", TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT, "Print X and Y Task");
  //PRERUN
  deploy();
  //

  //GOAL 1 PROCEDURE
  translationPID(15.0, 3.0, lastAngle, pros::millis(), 1000, false, false, false, 0, 0, 8600);
  translationPID(15.0, 35.0, -pi / 4.0, pros::millis(), 1100, true, true, false, 0, 0, 10000);
  translationPID(6.5, 42.0, -pi / 4.0, pros::millis(), 800, false, false, false, 0, 0, 9000);
  shootingProcedure(false);
  //
  //GOAL 2 PROCEDURE
  translationPID(12.5, 34.0, lastAngle, pros::millis(), 600, false, true, false, 0, 0, 8600);
  turnPID(pi / 4.0, pros::millis(), 600);
  translationPID(18.0, 43.0, pi / 5.8, pros::millis(), 800, true, true, false, 0, 0, 10000);
  turnPID(pi / 2.0, pros::millis(), 400);
  translationPID(45.0, 34.0, pi / 2.0, pros::millis(), 1000, true, true, false, 0, 0, 10000);
  translationPID(63.0, 33.0, pi / 2.0, pros::millis(), 900, true, false, false, 0, 0, 10000);
  turnPID(0.0, pros::millis(), 600);
  translationPID(63.0, 39.0, lastAngle, pros::millis(), 600, true, true, false, 0, 0, 8600);
  shootingProcedure(false);
  shootingProcedure(false);
  //
  //GOAL 3 PROCEDURE
  translationPID(63.0, 25.0, pi / 4.3, pros::millis(), 1100, false, false, false, 0, 0, 10000);
  translationPID(89.5, 44.0, pi / 4.4, pros::millis(), 1800, true, true, false, 0, 0, 10000);
  translationPID(117.0, 38.0, pi / 3.9, pros::millis(), 900, true, true, false, 0, 0, 10000);
  translationPID(119.0, 45.0, pi / 4.0, pros::millis(), 700, false, false, false, 0, 0, 10000);
  shootingProcedure(false);
  shootingProcedure(true);
  translationPID(119.0, 45.0, pi / 4.0, pros::millis(), 720, true, true, false, 0, 0, 10000);
  translationPID(100.0, 43.0, pi / 2.0, pros::millis(), 500, false, false, false, 0, 0, 10000);
  turnPID(pi, pros::millis(), 700);
  descoreProcedureStatic(950);
  //
  //GOAL 4 PROCEDURE
  translationPID(111.0, 45.0, lastAngle, pros::millis(), 100, false, false, false, 1, 0, 10000);
  translationPID(110.0, 10.0, pi + pi / 4.0, pros::millis(), 1200, true, true, false, 0, 0, 8600);
  translationPID(81.0, -15.5, pi + pi / 4.0, pros::millis(), 1200, true, true, false, 0, 0, 8600);
  turnPID(pi / 2.0, pros::millis(), 800);
  translationPID(112.0, -12.0, pi / 2.0, pros::millis(), 1000, true, true, false, 0, 0, 8600);
  translationPID(118.0, -12.0, pi / 2.0, pros::millis(), 300, false, false, false, 0, 0, 8600);
  shootingProcedure(false);
  shootingProcedure(true);
  //
  //GOAL 5 PROCEDURE
  translationPID(118.0, -12.0, pi / 2.0, pros::millis(), 0, true, true, false, 0, 0, 8600);
  translationPID(103.0, -12.0, lastAngle, pros::millis(), 400, true, true, false, 0, 0, 8600);
  turnPID(pi, pros::millis(), 600);
  translationPID(110.0, -55.0, pi / 2.0 + pi / 4.0, pros::millis(), 1700, true, true, false, 1, 0, 10000);
  translationPID(120.0, -68.7, pi / 2.0 + pi / 4.0, pros::millis(), 1000, false, false, false, 0, 0, 8600);
  translationPID(120.0, -70.5, pi / 2.0 + pi / 4.0, pros::millis(), 800, true, true, false, 0, 0, 8600);
  shootingProcedure(false);
  //
  //GOAL 6 PROCEDURE
  translationPID(109.0, -55.0, pi, pros::millis(), 800, false, false, false, 0, 0, 10000);
  turnPID(pi + pi / 2.0, pros::millis(), 600);
  translationPID(64.0, -60.0, lastAngle, pros::millis(), 1500, true, true, false, 2, 0, 8600);
  translationPID(63.0, -55.0, lastAngle, pros::millis(), 300, true, true, false, 0, 0, 10500);
  turnPID(pi, pros::millis(), 400);
  translationPID(62.5, -69.0, pi, pros::millis(), 600, true, false, false, 0, 0, 8600);
  shootingProcedure(true);
  translationPID(64.0, -55.0, pi, pros::millis(), 450, true, true, false, 0, 0, 8600);
  turnPID(pi + pi / 4.0, pros::millis(), 330);
  descoreProcedureStatic(500);
  //
  //GOAL 7 PROCEDURE
  translationPID(32.0, -70.0, pi + pi / 4.2, pros::millis(), 1400, true, true, false, 0, 0, 10000);
  translationPID(7.5, -64.0, pi + pi / 4.0, pros::millis(), 900, true, true, false, 0, 0, 10000);
  translationPID(5.0, -73.5, pi + pi / 4.1, pros::millis(), 600, false, false, false, 0, 0, 10000);
  shootingProcedure(true);
  translationPID(5.5, -73.50, pi + pi / 4.1, pros::millis(), 700, true, true, false, 0, 0, 8600);
  translationPID(15.0, -70.0, lastAngle, pros::millis(), 700, false, true, false, 0, 0, 8600);
  turnPID(2 * pi, pros::millis(), 700);
  descoreProcedureStatic(800);
  //
  //GOAL 8 PROCEDURE
  translationPID(13.0, -38.0, 2 * pi + pi / 4.0, pros::millis(), 1350, true, true, false, 0, 0, 10500);
  translationPID(41.0, -5.0, 2 * pi + pi / 4.0, pros::millis(), 1350, true, true, false, 0, 0, 10000);
  turnPID(pi + pi / 2.0, pros::millis(), 800);
  translationPID(11.25, -12.25, pi + pi / 2.0, pros::millis(), 1120, true, true, false, 0, 0, 10000);
  shootingProcedure(true);
  shootingProcedure(true);
  translationPID(11.25, -12.25, pi + pi / 2.0, pros::millis(), 30, true, true, false, 0, 0, 10000);
  translationPID(25.0, -12.25, pi + pi / 2.0, pros::millis(), 700, false, true, false, 0, 0, 10000);
  turnPID(2 * pi + pi / 4.0, pros::millis(), 500);
  descoreProcedureStatic(500);
  //
  //MIDDLE GOAL PROCEDURE
  translationPID(61.0, 13.0, 2 * pi + pi / 2.0, pros::millis(), 1200, true, true, false, 0, 0, 10000);
  turnPID(3 * pi, pros::millis(), 550);
  translationPID(58.0, -0.0, 3 * pi, pros::millis(), 900, true, true, false, 0, 0, 11000);
  translationPID(58.0, 7.0, 3 * pi, pros::millis(), 600, false, false, false, 0, 0, 11000);
  translationPID(58.0, 0.0, 3 * pi, pros::millis(), 500, false, false, false, 0, 0, 11000);
  translationPID(58.0, 7.0, 3 * pi, pros::millis(), 500, false, false, false, 0, 0, 11000);
  translationPID(58.0, 0.0, 3 * pi, pros::millis(), 500, false, false, false, 0, 0, 11000);
  translationPID(56.0, 10.0, 3 * pi, pros::millis(), 800, false, false, false, 0, 0, 11000);
  translationPID(52.0, 0, 2 * pi + pi / 2.0 + pi / 3.9, pros::millis(), 900, false, false, false, 0, 0, 10000);
  shootingProcedure(false);
  //
  //POSTRUN
  translationPID(52.0, 9.0, 2 * pi + pi / 2.0 + pi / 4.05, pros::millis(), 600, false, false, false, 0, 0, 10000);
  //


}
