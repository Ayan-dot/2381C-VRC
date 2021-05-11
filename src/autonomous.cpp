/*
  ___  ____   ___  __  _____
 |__ \|___ \ / _ \/_ |/ ____|
    ) | __) | (_) || | |
   / / |__ < > _ < | | |
  / /_ ___) | (_) || | |____
 |____|____/ \___/ |_|\_____|

2381C <Team Captain: allentao7@gmail.com>

This file is part of 2381C's codebase for 2020-21 VEX Robotics VRC Change
Up Competition.

This file can not be copied, modified, or distributed without the express
permission of 2381C.

All relevant mathematical calculations for odometry and motion profiling are
documented and have been explained in extensive detail in our paper about
robot motion. The paper is located in the docs (documentation) folder.

autonomous.cpp [contains]:
  - 120 programming skills routine
  - Functions used for programming skills routine
*/
#include "main.h"
#include "posTracking.cpp"
#include "globals.hpp"
#include <array>
#include "pid.hpp"

/*
Using array pointer values with arrays in the form {kp, ki, kd} to initialize
PID controllers for the left drivebase, the right drivebase, strafing and for
point turns.

These array pointer values are pre-tuned and declared globally in globals.cpp.
*/
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

/*
The following variables contain information about the location and heading of
the robot. They are declared outside of the scope of the function for the sake
of persistency and for global access.
*/
long double lastposR, currentposR, lastposL, currentposL, lastposH, currentposH, newAngle, lastAngle;
int ballsinBotG = 0;

/**
 * The function for the odometry task. Keeps track of the position of the robot
 * simultaneously while the robot performs its autonomous sequences. An
 * important characteristic of the function is that it modifies the global
 * variables discussed above for the allowing external access to the state.
 *
 * @param *param: a pointer
 */
void vector_tasks_fn(void *param)
{
  // variables to hold right vertical tracking wheel encoder position, in
  // intervals of 20 ms
  lastposR = 0, currentposR = 0;

  // variables to hold left vertical tracking wheel encoder position, in
  // intervals of 20 ms
  lastposL = 0, currentposL = 0;

  // horizontal counterparts of above variables
  lastposH = 0, currentposH = 0;

  // angles taken by inertial sensor (IMU), in intervals of 20 ms
  newAngle = 0, lastAngle = pi/2.0;

  // begin the main control loop, wherein the odometry procedure occurs
  while (true)
  {
    // reverse vertical encoders, find position, convert to inches
    currentposR = verticalEncoder2.get_value() * vertToInch;
    currentposL = verticalEncoder1.get_value() * vertToInch;

    // same as above, horizontal counterpart
    currentposH = horizontalEncoder.get_value() * horiToInch;

    // create and initialize a positionTracking instance for odometry
    positionTracking robotPos(lastAngle, currentposH, lastposH, currentposL, lastposL, currentposR, lastposR);

    // to avoid turning global coordinates into null values when calculations
    // are initializing
    if (!isnan(robotPos.returnX()) || !isnan(robotPos.returnY()))
    {
      // adds the horizontal vector passed by the position tracking class to
      // the global X coordinate
      globalX += robotPos.returnX();

      // same as above, vertical counterpart
      globalY += robotPos.returnY();
    }

    /*
    Perform IMU averaging, which in turn will modify the last angle. Rather
    than raw averaging, a weighted average will be computed.
    */

    // IMU averaging weights
    long double trackingWheelWeight = 0.0;
    long double imuWeight = 1.0;
    long double imuScaling = inerCoef;

    lastAngle = robotPos.returnOrient() * trackingWheelWeight + (inertial.get_rotation()) * (pi / 180.0) * imuWeight * imuScaling;
    lastposH = currentposH;
    lastposR = currentposR;
    lastposL = currentposL;

    // print x, y coordinates, the angle and IMU rotation to the brain
    pros::lcd::print(0, "X: %f", globalX);
    pros::lcd::print(1, "Y: %f", globalY);
    pros::lcd::print(2, "I: %f", lastAngle);
    pros::lcd::print(3, "Iner: %f", inertial.get_rotation());

    // set loop frequency (t = 20ms)
    pros::delay(20);
  }
}

/**
 * This function is a macro for the deploy procedure of the robot.
 */
void deploy()
{
  leftIntake.move_velocity(-200);
  rightIntake.move_velocity(200);
  indexer.move_velocity(200);
  pros::delay(250);
  indexer.move_velocity(0);
  pros::delay(250);
  leftIntake.move_velocity(0);
  rightIntake.move_velocity(0);

  /*
  To deploy, run conveyors out (as if you are spitting balls out through the bottom), and intakes in

  */
  // /*
  // Drive forwards, outtake to deploy everything
  // */leftBack.move_velocity(0);
  // leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  //
  // rightBack.move_velocity(0);
  // rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  //
  //
  // leftFront.move_velocity(160);
  //
  // rightFront.move_velocity(-160);
  //
  // pros::delay(150);
  // leftFront.move_velocity(0);
  //
  // rightFront.move_velocity(0);
  //
  // leftIntake.move_velocity(150);
  // rightIntake.move_velocity(-150);
  //   pros::delay(150);
  //   leftIntake.move_velocity(0);
  //   rightIntake.move_velocity(0);




}

/**
 * The static descore function ejects balls from the robots from the "pooper".
 * In particular this function is labelled static since it is called when the
 * robot is not moving or undergoing any controlled motion with the
 * translation PID.
 *
 * @param time the time in ms for which the robot will eject the balls
 */


/**
 * This function is a macro for the indexing and intake procedure.
 *
 * @param runIntakes
 * @param runIndexer
 * @param runIntakesBackwards
 * @param numShot
 */
void intakeIndexingProcedure(bool runIntakes, bool runIndexer, bool runIntakesBackwards, int numShot)
{
  if (runIntakes || runIntakesBackwards)
  {
    // define intake movement speed
    leftIntake.move_velocity((runIntakes ? 200 : -100));
    rightIntake.move_velocity((runIntakes ? -200 : 100));
  }

  if (numShot > 0)
  {
    // if balls to be shot are a non-zero number
    //descoreProcedureMoving(numShot);
  }

  else if (runIndexer)
  {
    if (line_tracker1.get_value() > COUNTER_THRESHOLD)
    {
      // we do not have a ball properly indexed yet
      indexer.move_velocity(-200);
      shooter.move_velocity(10);
    }
    else
    {
      shooter.move_velocity(0);
      // PID hold the shooter motor so that indexing balls does not allow the
      // top ball to be moved out of optimal shooting position
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

  // if intakes must be run backwards, reverse the indexer as well to move
  // balls downwards
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

/**
 * A macro executing the shooting procedure for the robot. Tuned to optimize
 * the motion of the ball.
 *
 * @param slowrun a boolean defining whether intakes will run slowly to
 *                maintain their grip on the balls — important to ensure
 *                consistency when intaking balls after shooting
 */
void shootingProcedure(bool slowrun)
{
  int time = pros::millis();
  while (line_tracker1.get_value() > COUNTER_THRESHOLD && pros::millis() - time < 1000)
  {
    // we do not have a ball properly indexed yet
    indexer.move_velocity(-200);
    shooter.move_velocity(20);
  }
  pros::delay(50);

  indexer.move_velocity(0);
  shooter.move_velocity(180);

  // if intakes must be run slowly
  if (slowrun)
  {
    leftIntake.move_velocity(60);
    rightIntake.move_velocity(-60);
  }
  pros::delay(500);

  shooter.move_velocity(0);

  return;
}

/**
 * The motion algorithm of the state function is expressed with a translation
 * PID. As suggested by its name, this algorithm moves the robot.
 *
 * @param x2 the target x position
 * @param y2 the target y position
 * @param heading the target heading
 * @param time time when the function is called
 * @param timeAllocated time allocated for completing the function
 * @param runIntakes whether intakes will be on (true) or off (false)
 * @param runIndexer whether the indexer will be on (true) or off (false)
 * @param runIntakesBackwards whether intakes will be reversed
 * @param numDescore number of balls to descore
 * @param numShoot number of balls to shoot
 * @param maxVolt the max volt sent to the drivebase
 */

/*void ballDistro(int ballShoot, int ballGrab){
  int shot = 0;
  int took = 0;

}*/


int ball_taken = 0;
int ball_shot = 0;
bool botCovered = true;
bool topCovered = true;
bool runTask = false;
void countingProcess(void *param){
    while(runTask){
    if(line_tracker1.get_value() >= COUNTER_THRESHOLD && botCovered){
      botCovered = false;
    }
    else if(line_tracker1.get_value() < COUNTER_THRESHOLD && !botCovered){
      ball_taken++;
      botCovered = true;
    }
    if(line_tracker2.get_value() >= INDEX_THRESHOLD && topCovered){
      ball_shot++;
      topCovered = false;
    }
    else if(line_tracker2.get_value() < INDEX_THRESHOLD && !topCovered){
      topCovered = true;
    }
    pros::delay(10);
    }
}

void ballDistro(int ballShoot, int ballGrab, bool corner){
  // you only want the 150ms delay after shooting after the last ball being shot, so that
  // conditional should only be run ONCE
  bool shotsDone = false;
   ball_shot = 0;
   ball_taken = 0;
  int retainSpeed = 0;
  bool keepIntake = true;
  bool keepShoot = true;
  
  if (line_tracker1.get_value() < COUNTER_THRESHOLD) {
   botCovered = true;
  } else {  
   botCovered = false;
  } 
  topCovered = true;
  
   double timePrime = pros::millis();
   while(line_tracker2.get_value() >= INDEX_THRESHOLD  && pros::millis()- timePrime < 1800){
   indexer.move_velocity(-200);
   shooter.move_velocity(190);
   }
  
  // set the conveyor system to be running up initially
  runTask = true;
  indexer.move_velocity(-200);
  leftIntake.move_velocity(-200);
  rightIntake.move_velocity(200);
  shooter.move_velocity(190);
  retainSpeed = 200;

  while(true){
  if(ball_taken >= ballGrab && ball_shot >= ballShoot){
    break;
  }
  else if(ball_taken >= ballGrab){
    leftIntake.move_velocity(0);
    rightIntake.move_velocity(0);
  }
  else{
    shooter.move_velocity(190);
    shooter.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  }
  pros::delay(10);

  }
  runTask = false;

  

  // while(ball_taken < ballGrab || ball_shot < ballShoot){
  //   pros::lcd::set_text(7, "RS " + to_string(retainSpeed));
  //   pros::lcd::set_text(5, "BALLS " + to_string(ball_taken));
  //   pros::lcd::set_text(4, "BALLS " + to_string(ball_shot));
  //   // counting
    
    
  //   // stop conditions for intaking:
  //   if(ball_taken>=ballGrab){
  //     // DEBUG
  //     if (ball_taken > ballGrab) {
  //       // you better have terminal open 
  //      // cout << "Overintake ERR: " << "Balls taken: " << took << '\n';
  //     }
  //     leftIntake.move_velocity(0);
  //     rightIntake.move_velocity(0);
  //     indexer.move_velocity(0);
  //     retainSpeed = 0;
  //   }
  //   // stop conditions for shooting
  //   if(shotsDone) {
  //     shooter.move_velocity(0);
  //     shooter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  //     leftIntake.move_velocity(-retainSpeed);
  //     rightIntake.move_velocity(retainSpeed);
  //   }
    // if(ball_shot>=ballShoot){
    //   shotsDone = true; // so we don't delay 150ms more than once
    //   // DEBUG
    //   if (shot > ballShoot) {
    //     // you better have terminal open 
    //     cout << "Overshoot ERR: " << "Balls shot: " << shot << '\n';
    //   }
    //   // stop the intakes so it doesn't overintake
    //   leftIntake.move_velocity(0);
    //   rightIntake.move_velocity(0);
    //   // 150ms delay to make sure ball leaves robot
    //   double extraTime = pros::millis();
    //   while(line_tracker2.get_value() >= INDEX_THRESHOLD && (pros::millis() - extraTime) < 150){
    //     pros::delay(10);
    //   }
    //   leftIntake.move_velocity(-retainSpeed);
    //   rightIntake.move_velocity(retainSpeed);
    //   shooter.move_velocity(0);
    //   shooter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);  
    // }
    // pros::delay(10);
  }


void ballDistro2(int ballShoot, int ballGrab, bool corner){
  int shot = 0;
  int took = 0;
  int booltToggle;
  if(line_tracker1.get_value() < COUNTER_THRESHOLD){
    booltToggle = true;
  }
  else{
    booltToggle = false;
  }
  int boolsToggle = true;
  if(corner){
  leftFront.move_velocity(60);
  rightFront.move_velocity(-60);
  rightBack.move_velocity(-60);
  leftBack.move_velocity(60);
  //leftIntake.move_velocity(-60);
//  rightIntake.move_velocity(60);
}
double time = pros::millis();
  while(line_tracker2.get_value() >= INDEX_THRESHOLD && pros::millis()-time < 1800){
    indexer.move_velocity(-200);
    shooter.move_velocity(190);
  }
  shooter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  indexer.move_velocity(0);
  shooter.move_velocity(0);
  double timer = pros::millis();
  
  while((shot!=ballShoot || took!=ballGrab) && pros::millis()-timer < 2500){
    pros::lcd::set_text(7, "BALLS " + to_string(shot));
    if(took!=ballGrab){
    indexer.move_velocity(-110);}
    else{
      indexer.move_velocity(0);
    }
  if(shot!=ballShoot){
    shooter.move_velocity(200);
  }
  else{
    shooter.move_velocity(0);
    shooter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }
  if(took!=ballGrab){
    leftIntake.move_velocity(-200);
    rightIntake.move_velocity(200);
  }
  else{
    leftIntake.move_velocity(0);
    rightIntake.move_velocity(0);
  }
  if(line_tracker1.get_value() < COUNTER_THRESHOLD_LOW && !boolsToggle){
  took++;
  boolsToggle = true;
  }
  if(line_tracker1.get_value() >= COUNTER_THRESHOLD && boolsToggle){
    boolsToggle = false;
  }
  if(line_tracker2.get_value() < INDEX_THRESHOLD_LOW && !booltToggle){
  //  shot++;
    booltToggle = true;
  }
  if(line_tracker2.get_value()>= INDEX_THRESHOLD &&booltToggle == true){
    booltToggle = false;
    shot++;

    if(shot==ballShoot){
      leftIntake.move_velocity(0);
      rightIntake.move_velocity(0);
      pros::delay(150);
    }
    if(took == ballGrab){
      indexer.move_velocity(0);
    }
  }
}
indexer.move_velocity(0);
shooter.move_velocity(0);
leftIntake.move_velocity(0);
rightIntake.move_velocity(0);

  if(took>ballGrab){
    pros::delay(10000);
  }
  leftFront.move_velocity(0);
  rightFront.move_velocity(0);
  rightBack.move_velocity(0);
  leftBack.move_velocity(0);


}
int translationPID(long double x2, long double y2, long double heading, int time, int timeAllocated, bool runIntakes, bool runIndexer, int runIntakesGrip, int runIntakesBack, int removeBalls, int numGrab, double maxVolt)
{
  int ballShot = 0;
  int maxTime = 0;
  bool toggleBallUp = false;
  bool toggleBallsMax = false;
  int ballstoReach = numGrab;
  int ballsinBot = 0;
  bool addActive = false;
  bool timeSet = false;
  double refTime;
  bool indStart = false;

  // keep track of current time
double curTime = pros::millis();

  // if the balls are not fully indexed (defining amount of time to descore)


  while (true)
  {

    // run the intake indexing procedure if commanded to do so
    // if any intake / conveying function is called

    if(removeBalls>0){
      double maxRem;
      if(removeBalls==1){
        maxRem = 770;
        if(!timeSet){
         refTime = pros::millis();
          timeSet = true;
        }}
      else{
        maxRem = 1560;
        if(!timeSet){

         refTime = pros::millis();
          timeSet = true;
        /*  else{
            indexer.move_velocity(90);
            shooter.move_velocity(-80);
          } */
        }
      }
    if((pros::millis()-refTime)<maxRem && timeSet){
        indexer.move_velocity(-180);
        shooter.move_velocity(-200);
      }
      else{
        removeBalls=0;
        indexer.move_velocity(0);
        shooter.move_velocity(0);
        indexer.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        shooter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      }

    }
    long double currentTime = pros::millis() - time;
    long double indTime;

    long double coefTime = 1.0;
    if (runIntakes)
    {
      if(ballstoReach>0){
      leftIntake.move_velocity(-200);
      rightIntake.move_velocity(200);}
      else{
        leftIntake.move_velocity(0);
        rightIntake.move_velocity(0);
      }
      if(line_tracker1.get_value() < COUNTER_THRESHOLD && !indStart){
        ballstoReach--;
        indStart = true;
        indTime = pros::millis()-time;
      }
      if(indStart && (currentTime - indTime) < 315 && line_tracker2.get_value() >= INDEX_THRESHOLD){
        indexer.move_velocity(-200);
        if(!removeBalls)
        shooter.move_velocity(160);
        else{
        shooter.move_velocity(-150);
      }}
      else if(((currentTime - indTime) >= 315 && removeBalls==0) || line_tracker2.get_value() < INDEX_THRESHOLD){
        indStart = false;
        indexer.move_velocity(0);
        shooter.move_velocity(0);
        indexer.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        shooter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      }
      else{
        indexer.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        shooter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      }
    }
    else if(runIntakesBack){
      if(!indStart){
        indStart = true;
        indTime = pros::millis()-time;
      }
      if(indStart && (currentTime - indTime) < 700){
        indexer.move_velocity(140);
        shooter.move_velocity(-130);
        leftIntake.move_velocity(155);
        rightIntake.move_velocity(-155);
      }
      else{
        indexer.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        shooter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      }
    }
    else if(runIntakesGrip){
      leftIntake.move_velocity(80);
      rightIntake.move_velocity(-80);
    }
    else if(runIndexer){
      if(pros::millis()-time <= 400){
      indexer.move_velocity(-180);}

    }


    /*
    The below procedure translates the robot from point x1 and y1 (its current)
    location on the field to point x2 and y2 while maintaining a constant
    heading
    */

    // current robot x and y position is given by odometry (global variables)
    long double x1 = globalX;
    long double y1 = globalY;

    // step 1, using x1, y1, x2, y2, and heading, let us calculate the STANDARD FORM equation of the lines
    // this can be done by simply finding a second pair of coordinates, as two points define a line

    // how far away in euclidean distance the second point we want is, can be
    // any arbitrary number
    const long double l = 100.0;

    // [ROBOT POSITION POINT] the first line is defined by x1 and y1, and the
    // heading—we now use this to define a second point on the line
    long double x1Other = x1 + l * sin(lastAngle);
    long double y1Other = y1 + l * cos(lastAngle);

    // [ROBOT POSITION LINE PARALLEL TO HEADING (line NT)]
    // reversed because we have to assign negative sign to either numerator
    // or denominator
    long double A1 = y1 - y1Other;
    long double B1 = x1Other - x1;
    long double C1 = -(A1 * x1 + B1 * y1);

    // [ROBOT DESTINATION LINE PERPENDICULAR PREVIOUS LINE (line MT)]
    long double A2 = B1;
    long double B2 = -A1;
    long double C2 = -(A2 * x2 + B2 * y2);

    // with the STANDARD FORM equations for both lines, let us calculate the
    // intersection point of the two; let (x, y) denote the coordinate
    // intersection of the two lines, represented in STANDARD FORM
    long double x = (B1 * C2 - B2 * C1) / (B2 * A1 - B1 * A2);
    long double y = (A1 * C2 - A2 * C1) / (A2 * B1 - A1 * B2);

    // since Euclidean distance is always a positive value, we must know the direction in which the error is
    // to do this, we need to define another line
    // LINE: going through x1, y1, heading rotated 90 degrees anticlockwise

    // define another point on the line perpendicular to line NT in calculations
    long double x3Other = x1 + l * sin(heading - pi / 2.0);
    long double y3Other = y1 + l * cos(heading - pi / 2.0);

    // with this information, we can now determine the signage of the vertical
    // and horizontal components (relative to heading)
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

    // with the intersection point and direction, we can now calculate the
    // Euclidean distance forwards relative to the robot's direction and
    // horizontally relative to the robot heading to get our x and y components
    // (errors) for our overall translation vector
    long double yComponent = abs(sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1))) * verticalSign;
    long double xComponent = abs(sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2))) * horizontalSign;
    cout << "COMPONENT VECTORS: " << yComponent << ' ' << xComponent << '\n';

    // to send this information to the motors, we pass the original encoder
    // position, and the destination encoder position plus their respective
    // local components for the PID loop, and we use ticks to inch conversion
    // as function arguments are in inches
    long double leftDest = verticalEncoder1.get_value() + yComponent * (360 / (2.75 * pi));
    long double rightDest = verticalEncoder2.get_value() + yComponent * (360 / (2.75 * pi));
    long double horizontalDest = horizontalEncoder.get_value() + xComponent * (360 / (2.75 * pi));
    cout << "DESTINATIONS: " << leftDest << ' ' << rightDest << ' ' << horizontalDest << '\n';

    // begin PID controllers

    // [y component] forward backward (relative to heading) voltage output
    long double yVoltageLeft = leftdrivebasePIDController->update(leftDest, verticalEncoder1.get_value(), -1);
    long double yVoltageRight = rightdrivebasePIDController->update(rightDest, verticalEncoder2.get_value(), -1);
    cout << "FB Volt: " << yVoltageLeft << ' ' << yVoltageRight << '\n';

    // [x component] side to side (relative to heading) voltage output
    long double xVoltage = strafePIDController->update(horizontalDest, horizontalEncoder.get_value(), -1);
    cout << "H Volt: " << xVoltage << '\n';
    pros::lcd::set_text(5, "lDest: " + to_string(leftDest));
    pros::lcd::set_text(6, "rDest: " + to_string(rightDest));


    // [angle] angle heading correction with voltage output
    long double angleVoltageLeft = turningPIDController->update(heading, lastAngle, -1);
    long double angleVoltageRight = -turningPIDController->update(heading, lastAngle, -1);
    cout << "Ang Volt: " << angleVoltageLeft << ' ' << angleVoltageRight << '\n';

    /*
    To allow for a smoother acceleration gradient, time is considered in
    scaling the voltage outputs. So, we limit speed the robot can travel in the
    first accelerationTime ms of acceleration. accelerationTime, like other
    tunable constants, is adjusted in globals.cpp.
    */


    if (currentTime < accelerationTime)
    {
      // based off of the square root curve approaching the target voltage
      coefTime = sqrt(currentTime) / sqrt(accelerationTime);
    }

    // the left side motors are positive power for forwards movement and the
    // right side motors are negative power for forwards movement
    long double finalVoltageLeftFront = (yVoltageLeft + xVoltage + angleVoltageLeft) * coefTime;
    long double finalVoltageLeftBack = (yVoltageLeft - xVoltage + angleVoltageLeft) * coefTime;
    long double finalVoltageRightFront = -(yVoltageRight - xVoltage + angleVoltageRight) * coefTime;
    long double finalVoltageRightBack = -(yVoltageRight + xVoltage + angleVoltageRight) * coefTime;

    // prevent exceeding the MAX_VOLTAGE value by scaling the motors accordingly
    long double maxOutput = max({finalVoltageLeftFront, finalVoltageLeftBack, finalVoltageRightFront, finalVoltageRightBack});
    if (maxOutput > maxVolt)
    {
      // scale down all of the motor outputs by a fraction >= 0 < 1 such that
      // the max voltage does not exceed MAX_VOLTAGE
      long double scaling = maxVolt / maxOutput;
      finalVoltageLeftFront *= scaling;
      finalVoltageLeftBack *= scaling;
      finalVoltageRightFront *= scaling;
      finalVoltageRightBack *= scaling;
    }

    cout << "FINAL OUTPUT: " << finalVoltageLeftFront << ' ' << finalVoltageLeftBack << ' ' << finalVoltageRightFront << ' ' << finalVoltageRightBack << '\n';

    // stop the robot if the maximum allocated time is exceeded
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
      return ballsinBot;
    }

    // move motors with final voltages
    leftFront.move_voltage(finalVoltageLeftFront);
    leftBack.move_voltage(finalVoltageLeftBack);
    rightFront.move_voltage(finalVoltageRightFront);
    rightBack.move_voltage(finalVoltageRightBack);
    pros::delay(20);
  }

  return ballsinBot;
}

/**
 * The turn PID function is used for making point turns.
 *
 * @param targetAngle a double defining the target angle
 * @param time the start time of the procedure
 * @param timeAllocated an integer reflecting the allocated time in ms before the
 *                      procedure is abandoned
 */

void turnPID(long double targetAngle, int time, int timeAllocated, bool runIntakesBack)
{
  // begin turn PID loop
  while (true)
  {
    // correction of angle haeding using voltage output (set up a PID for both
    // left and right sides of the x drive using motor encoder values for
    // current and destination)
    long double angleVoltageLeft = pointTurnPIDController->update(targetAngle, lastAngle, -1);
    long double angleVoltageRight = pointTurnPIDController->update(targetAngle, lastAngle, -1);

    pros::lcd::set_text(5, "Ang: " + to_string(lastAngle));
    leftFront.move_voltage(angleVoltageLeft);
    leftBack.move_voltage(angleVoltageLeft);
    rightFront.move_voltage(angleVoltageRight);
    rightBack.move_voltage(angleVoltageRight);

    if(runIntakesBack){
        leftIntake.move_velocity(200);
        rightIntake.move_velocity(-200);
      }

    // if the allocated time is exceeded, stop the robot, and quit the function
    if (pros::millis() - time > timeAllocated)
    {
      cout << "PID STOPPED" << '\n';

      leftFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      rightFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      leftBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      rightBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      leftIntake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      rightIntake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      leftFront.move_velocity(0);
      rightFront.move_velocity(0);
      leftBack.move_velocity(0);
      rightBack.move_velocity(0);
      leftIntake.move_velocity(0);
      rightIntake.move_velocity(0);

      return;
    }
    pros::delay(20);
  }
}
void middleCycle(){
  /*leftFront.move_velocity(100);
  rightFront.move_velocity(-100);
  rightBack.move_velocity(-100);
  leftBack.move_velocity(100);*/

  //while(line_tracker2.get_value()>=INDEX_THRESHOLD){
  //  shooter.move_velocity(180);
    //indexer.move_velocity(-180);
//  }

  shooter.move_velocity(0);
  indexer.move_velocity(0);
  shooter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  indexer.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  while(line_tracker1.get_value()>=COUNTER_THRESHOLD){
    leftIntake.move_velocity(-200);
    rightIntake.move_velocity(200);
  }
  pros::delay(250);
  leftIntake.move_velocity(0);
  rightIntake.move_velocity(0);
  if(line_tracker2.get_value()<INDEX_THRESHOLD && line_tracker1.get_value()<COUNTER_THRESHOLD){
    shooter.move_velocity(200);
    indexer.move_velocity(-80);
    pros::delay(320);
  }
  shooter.move_velocity(0);
  indexer.move_velocity(0);
  for(int i = 0; i < 2; i++){
    while(line_tracker1.get_value()>=COUNTER_THRESHOLD){
      leftIntake.move_velocity(-200);
      rightIntake.move_velocity(200);
    }
    //shooter.move_velocity(-150);
    indexer.move_velocity(-190);
    pros::delay(250);
    leftIntake.move_velocity(0);
    rightIntake.move_velocity(0);
    //shooter.move_velocity(-140);
    //indexer.move_velocity(-180);
    //shooter.move_velocity(0);
    //indexer.move_velocity(0);
  }
  pros::delay(300);
  leftIntake.move_velocity(0);
  rightIntake.move_velocity(0);
  shooter.move_velocity(0);
  indexer.move_velocity(0);
  leftFront.move_velocity(0);
  rightFront.move_velocity(0);
  rightBack.move_velocity(0);
  leftBack.move_velocity(0);

}

/**
 * The autonomous function contains the full procedure for creating the
 * programming skills run.
 *
 * Currently, our robot consistently scores: *120* points.
 */
void autonomous()
{
  // ALLEN TESTING PLZ REMOVE

  // start by creating an odom instance
  pros::Task position_task(vector_tasks_fn, (void *)"PROS", TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT, "Print X and Y Task");
  pros::Task counting_task(countingProcess, (void *)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Counting Balls");
//translationPID(0, 24.0, 0, pros::millis(), 2000, true, false, 0, 0, 2, 1,12000);
//pros::delay(100000);
  // while(true) {
  //  ballDistro(2, 1, 0);
///   pros::delay(1000);
//   }
  // // begin run
  ballDistro(2,1,0);

  // turnPID(-pi / 2.0, pros::millis(), 700);
  //ballDistro(2,2,0);
  //CODE FOR RUN -------- 
  /*
  deploy();
  translationPID(13.3, 0.0, 0, pros::millis(), 600, false, false, 0, 0, 0,0,12000);
 translationPID(14.0, 32.0, -pi/3.6, pros::millis(), 1000, true, false, 0, 0, 0,2,12000);
  translationPID(3.2, 42.2, -pi/4.0, pros::millis(), 730, false, false, 0, 0, 0,1,12000);
  ballDistro(2,2,0);
  translationPID(16.0, 26.0, -pi/2.5, pros::millis(), 460, false, false, 1, 0, 0,0,12000);
  translationPID(18.0, 22.0, -pi/5.2, pros::millis(), 800, false, false, 0, 1, 0,0,12000);
  translationPID(20.0, 42.0, pi/6.5, pros::millis(), 1290, true, false, 0, 0, 0,1,12000);
  translationPID(39.0, 10.0, pi/2.0, pros::millis(), 1640, true, false, 0, 0, 0,1,12000);
  translationPID(59.0, 10.0, pi/2.0, pros::millis(), 1500, true, false, 0, 0, 0,1,12000);
  //translationPID(57.0, 33.0, pi/5.5, pros::millis(), 920, true, false, 0, 0, 0,1,12000);
  turnPID(0.0, pros::millis(), 750, false);
  translationPID(59.0, 33.0, 0, pros::millis(), 1220, true, false, 0, 0, 0,1,12000);
  translationPID(59.0, 39.0, 0, pros::millis(), 500, false, false, 0, 0, 0,0,7000);

  ballDistro(2,1,0);
  translationPID(59.0, 34.0, 0, pros::millis(), 520, false, false, 1, 0, 0,0,12000);
  translationPID(59.0, 30.0,-pi/5.0, pros::millis(), 720, false, false, 0, 1, 0,0,12000);
  translationPID(59.0, 24.5, 0, pros::millis(), 800, true, false, 0, 0, 0,1,12000);
  turnPID(pi/2.0, pros::millis(), 650, false);
  translationPID(98.0, 24.5, pi/2.0, pros::millis(), 1450, true, false, 0, 0, 0,1,12000);
  translationPID(95.0, 28.5, 0, pros::millis(), 900, true, false, 0, 0, 0,1,12000);
  pros::delay(5000);
  translationPID(57.0, 35.0, 0, pros::millis(), 500, false, false, 0, 0, 0,0,12000);
  //pros::delay(230);

  //translationPID(57.0, 40.0, -pi/2.0, pros::millis(), 720, false, false, 0, 0, 0,0,12000);
  turnPID(-pi, pros::millis(), 800, false);

  translationPID(60.0, 0.0, -pi, pros::millis(), 1820, true, false, 0, 0, 1,2,7000);
  //ballDistro(1,3);
  ballDistro(1,3,0);
  translationPID(58.7, 14.0, -pi, pros::millis(), 820, false, true, 0, 0, 0,0,12000);
  translationPID(58.7, 23.0, -pi, pros::millis(), 720, false, false, 0, 0, 0,0,12000);
  turnPID(-pi-pi/2.0, pros::millis(), 600, false);
  translationPID(106.7, 22.0, -pi-pi/1.60, pros::millis(), 1780, true, false, 0, 0, 2,1,12000);
  //translationPID(109.7, 40.0, -pi-pi/2.0-pi/4.0, pros::millis(), 300, true, false, 0, 0, 0,1,12000);
  translationPID(94.7, 36.0, -2*pi, pros::millis(), 1200, true, false, 0, 0, 0,2,12000);
  translationPID(94.7, 44.0, -2*pi, pros::millis(), 800, true, false, 0, 0, 0,2,12000);
  translationPID(107.7, 41.0, -pi-pi/2.0-pi/4.0, pros::millis(), 900, true, false, 0, 0, 0,1,12000);
  translationPID(115.7, 48.0, -pi-pi/2.0-pi/3.7, pros::millis(), 900, false, false, 0, 0, 0,0,12000);
  ballDistro(2,2,0);
  translationPID(100.7, 30.0, -pi-pi/2.0, pros::millis(), 900, false, false, 0, 1, 0,0,12000);
  turnPID(-pi+pi/4.0, pros::millis(), 600, false);
  translationPID(88, -7.0, -pi, pros::millis(), 1500, true, false, 0, 0, 2,1,12000);
  translationPID(94, -7.0, -pi-pi/2.0, pros::millis(), 1000, true, false, 0, 0, 0,1,12000);
  translationPID(114, -8.5, -pi-pi/2.0, pros::millis(), 900, false, false, 0, 0, 0,0,12000);
  ballDistro(1,1,0);
  translationPID(105, -8.5, -pi-pi/2.0, pros::millis(), 700, false, false, 0, 0, 0,0,12000);
  turnPID(-pi, pros::millis(), 600, false);
  //translationPID(110, -10.5, -pi, pros::millis(), 600, true, false, 0, 0, 2,0,12000);
  translationPID(110, -50.0, -pi, pros::millis(), 1100, true, false, 0, 0, 2,1,12000);
  translationPID(100, -65.5, -pi, pros::millis(), 1100, true, false, 0, 0, 0,1,12000);
  translationPID(104, -52.0, -pi-pi/3.95, pros::millis(), 800, true, false, 0, 0, 0,0,12000);
  translationPID(120.5, -64.8, -pi-pi/3.95, pros::millis(), 1600, false, false, 0, 0, 0,0,12000);
  ballDistro(2,2,0);
  translationPID(104, -47.0, -pi, pros::millis(), 1100, false, false, 0, 1, 0,0,12000);
  turnPID(-pi/2.0, pros::millis(), 600, false);
  translationPID(62, -63.0, -pi/2.0, pros::millis(), 1200, true, false, 0, 0, 2,1,12000);
  translationPID(62, -58.0, -pi, pros::millis(), 1000, true, false, 0, 0, 0,1,12000);
  turnPID(-pi, pros::millis(), 200, false);
  translationPID(64.4, -67.0, -pi, pros::millis(), 800, false, false, 0, 0, 0,0,12000);
  ballDistro(1,0,0);
  translationPID(65, -59.0, -pi/2.0, pros::millis(), 800, false, false, 0, 0, 1,0,12000);
  translationPID(65, -59.0, 0.0, pros::millis(), 800, false, false, 0, 0, 2,0,12000);
  translationPID(65, -42.0, 0, pros::millis(), 1000, true, false, 0, 0, 0,1,12000);
  turnPID(-pi/4.0, pros::millis(), 500, false);
  translationPID(43, -16.0, -pi/4.0, pros::millis(), 1600, true, false, 0, 0,1,2,12000);
  translationPID(15, -17.0, -pi/2.0, pros::millis(), 1600, true, false, 0, 0, 0,2,12000);
  translationPID(5, -17.0, -pi/2.0, pros::millis(), 900, true, false, 0, 0, 0,0,12000);
  ballDistro(1,0,0);
  translationPID(18, -15.0, -pi, pros::millis(), 900, false, false, 0, 0, 2,0,12000);
  translationPID(7.2, -90.0, -pi, pros::millis(), 900, true, false, 0, 0, 0,1,12000);
  turnPID(-pi+pi/4.0, pros::millis(), 500, false);
  translationPID(5, -64.8, -pi+pi/4.0, pros::millis(), 900, false, false, 0, 0, 0,1,12000);
  ballDistro(2,2,0);
  */



















//   deploy();
//   //
//   // goal 1
//   translationPID(0.0, 12.0, lastAngle, pros::millis(), 700, true, true, 0, 0, 2, 12000);
//   turnPID(-pi/2.0, pros::millis(), 400);
//   translationPID(-36.0, 15.0, -pi/2.0-pi/4.0, pros::millis(), 1200, true, true, 0, 0, 2, 12000);
//   translationPID(-40.0, 5.0, -pi/2.0-pi/4.0, pros::millis(), 600, false, false, 0, 0, 0, 12000);
//   ballDistro(2,2,2);
//
//
//   translationPID(-28.0, 16.0, -pi-pi/6.0, pros::millis(), 700, false, false, 1, 0, 0, 12000);
//   translationPID(-27.0, 17.0, -pi, pros::millis(), 900, false, false, 0, 1, 0, 12000);
//   // translationPID(-30.0, 19.0, -pi/6.0, pros::millis(), 1400, false, false, 0, 1, 0, 10000);
//   turnPID(-pi / 6.0, pros::millis(), 600);
//
//   translationPID(-44.0, 15.0, -pi/3.7, pros::millis(), 900, true, false, 0, 0, 3, 12000);
//
//   translationPID(-10.0, 43.0, 0.0, pros::millis(), 1600, true, false, 0, 0, 3, 12000);
//
//   translationPID(-8.0, 60.0, 0.0, pros::millis(), 700, true, false, 0, 0, 3, 12000);
//
//   turnPID(-pi / 2.0, pros::millis(), 500);
//
//   translationPID(-32.0, 61.0, -pi/2.0, pros::millis(), 1000, true, false, 0, 0, 3, 10000);
//
//   translationPID(-44.0, 61.0, -pi/2.0, pros::millis(), 600, false, false, 0, 0, 3, 10000);
//   ballDistro(3,1,3);
//
// translationPID(-25.0, 61.0, -pi/1.9, pros::millis(), 700, false, false, 2, 0, 0, 12000);
// translationPID(-27.0, 61.0, -pi/1.5, pros::millis(), 600, false, false, 0, 1, 0, 12000);
// turnPID(-pi / 3.0, pros::millis(), 400);
// translationPID(-49.0, 100.0, -pi/3.0, pros::millis(), 1400, true, false, 0, 0, 1, 12000);
// translationPID(-35.0, 109.0, -pi/4.0, pros::millis(), 900, true, false, 0, 0, 1, 12000);
// translationPID(-49.0, 125.0, -pi/4.0, pros::millis(), 700, false, false, 0, 0, 1, 12000);
// ballDistro(1,2,1);
// translationPID(-35.0, 106.0, -pi/4.0, pros::millis(), 800, false, false, 1, 0, 0, 12000);
// translationPID(-35.0, 106.0, 0.0, pros::millis(), 400, false, false, 0, 0, 0, 12000);
// // translationPID(-35.0, 120.0, -pi, pros::millis(), 800, false, false, 0, 0, 0, 12000);
// translationPID(-34.0, 105.0, 0, pros::millis(), 600, false, false, 0, 1, 0, 12000);
// turnPID(pi/2.0, pros::millis(), 1000);






  // shootingProcedure(false);
  //
  // // goal 2
  // translationPID(12.5, 34.0, lastAngle, pros::millis(), 600, false, true, false, 0, 0, 8600);
  // turnPID(pi / 4.0, pros::millis(), 600);
  // translationPID(18.0, 43.0, pi / 5.8, pros::millis(), 800, true, true, false, 0, 0, 10000);
  // turnPID(pi / 2.0, pros::millis(), 400);
  // translationPID(45.0, 34.0, pi / 2.0, pros::millis(), 1000, true, true, false, 0, 0, 10000);
  // translationPID(63.0, 33.0, pi / 2.0, pros::millis(), 900, true, false, false, 0, 0, 10000);
  // turnPID(0.0, pros::millis(), 600);
  // translationPID(63.0, 39.0, lastAngle, pros::millis(), 600, true, true, false, 0, 0, 8600);
  // shootingProcedure(false);
  // shootingProcedure(false);
  //
  // // goal 3
  // translationPID(63.0, 25.0, pi / 4.3, pros::millis(), 1100, false, false, false, 0, 0, 10000);
  // translationPID(89.5, 44.0, pi / 4.4, pros::millis(), 1800, true, true, false, 0, 0, 10000);
  // translationPID(117.0, 38.0, pi / 3.9, pros::millis(), 900, true, true, false, 0, 0, 10000);
  // translationPID(119.0, 45.0, pi / 4.0, pros::millis(), 700, false, false, false, 0, 0, 10000);
  // shootingProcedure(false);
  // shootingProcedure(true);
  // translationPID(119.0, 45.0, pi / 4.0, pros::millis(), 720, true, true, false, 0, 0, 10000);
  // translationPID(100.0, 43.0, pi / 2.0, pros::millis(), 500, false, false, false, 0, 0, 10000);
  // turnPID(pi, pros::millis(), 700);
  // descoreProcedureStatic(950);
  //
  // // goal 4
  // translationPID(111.0, 45.0, lastAngle, pros::millis(), 100, false, false, false, 1, 0, 10000);
  // translationPID(110.0, 10.0, pi + pi / 4.0, pros::millis(), 1200, true, true, false, 0, 0, 8600);
  // translationPID(81.0, -15.5, pi + pi / 4.0, pros::millis(), 1200, true, true, false, 0, 0, 8600);
  // turnPID(pi / 2.0, pros::millis(), 800);
  // translationPID(112.0, -12.0, pi / 2.0, pros::millis(), 1000, true, true, false, 0, 0, 8600);
  // translationPID(118.0, -12.0, pi / 2.0, pros::millis(), 300, false, false, false, 0, 0, 8600);
  // shootingProcedure(false);
  // shootingProcedure(true);
  //
  // // goal 5
  // translationPID(118.0, -12.0, pi / 2.0, pros::millis(), 0, true, true, false, 0, 0, 8600);
  // translationPID(103.0, -12.0, lastAngle, pros::millis(), 400, true, true, false, 0, 0, 8600);
  // turnPID(pi, pros::millis(), 600);
  // translationPID(110.0, -55.0, pi / 2.0 + pi / 4.0, pros::millis(), 1700, true, true, false, 1, 0, 10000);
  // translationPID(120.0, -68.7, pi / 2.0 + pi / 4.0, pros::millis(), 1000, false, false, false, 0, 0, 8600);
  // translationPID(120.0, -70.5, pi / 2.0 + pi / 4.0, pros::millis(), 800, true, true, false, 0, 0, 8600);
  // shootingProcedure(false);
  //
  // // goal 6
  // translationPID(109.0, -55.0, pi, pros::millis(), 800, false, false, false, 0, 0, 10000);
  // turnPID(pi + pi / 2.0, pros::millis(), 600);
  // translationPID(64.0, -60.0, lastAngle, pros::millis(), 1500, true, true, false, 2, 0, 8600);
  // translationPID(63.0, -55.0, lastAngle, pros::millis(), 300, true, true, false, 0, 0, 10500);
  // turnPID(pi, pros::millis(), 400);
  // translationPID(62.5, -69.0, pi, pros::millis(), 600, true, false, false, 0, 0, 8600);
  // shootingProcedure(true);
  // translationPID(64.0, -55.0, pi, pros::millis(), 450, true, true, false, 0, 0, 8600);
  // turnPID(pi + pi / 4.0, pros::millis(), 330);
  // descoreProcedureStatic(500);
  //
  // // goal 7
  // translationPID(32.0, -70.0, pi + pi / 4.2, pros::millis(), 1400, true, true, false, 0, 0, 10000);
  // translationPID(7.5, -64.0, pi + pi / 4.0, pros::millis(), 900, true, true, false, 0, 0, 10000);
  // translationPID(5.0, -73.5, pi + pi / 4.1, pros::millis(), 600, false, false, false, 0, 0, 10000);
  // shootingProcedure(true);
  // translationPID(5.5, -73.50, pi + pi / 4.1, pros::millis(), 700, true, true, false, 0, 0, 8600);
  // translationPID(15.0, -70.0, lastAngle, pros::millis(), 700, false, true, false, 0, 0, 8600);
  // turnPID(2 * pi, pros::millis(), 700);
  // descoreProcedureStatic(800);
  //
  // // goal 8
  // translationPID(13.0, -38.0, 2 * pi + pi / 4.0, pros::millis(), 1350, true, true, false, 0, 0, 10500);
  // translationPID(41.0, -5.0, 2 * pi + pi / 4.0, pros::millis(), 1350, true, true, false, 0, 0, 10000);
  // turnPID(pi + pi / 2.0, pros::millis(), 800);
  // translationPID(11.25, -12.25, pi + pi / 2.0, pros::millis(), 1120, true, true, false, 0, 0, 10000);
  // shootingProcedure(true);
  // shootingProcedure(true);
  // translationPID(11.25, -12.25, pi + pi / 2.0, pros::millis(), 30, true, true, false, 0, 0, 10000);
  // translationPID(25.0, -12.25, pi + pi / 2.0, pros::millis(), 700, false, true, false, 0, 0, 10000);
  // turnPID(2 * pi + pi / 4.0, pros::millis(), 500);
  // descoreProcedureStatic(500);
  //
  // // goal 9 (middle goal)
  // translationPID(61.0, 13.0, 2 * pi + pi / 2.0, pros::millis(), 1200, true, true, false, 0, 0, 10000);
  // turnPID(3 * pi, pros::millis(), 550);
  // translationPID(58.0, -0.0, 3 * pi, pros::millis(), 900, true, true, false, 0, 0, 11000);
  // translationPID(58.0, 7.0, 3 * pi, pros::millis(), 600, false, false, false, 0, 0, 11000);
  // translationPID(58.0, 0.0, 3 * pi, pros::millis(), 500, false, false, false, 0, 0, 11000);
  // translationPID(58.0, 7.0, 3 * pi, pros::millis(), 500, false, false, false, 0, 0, 11000);
  // translationPID(58.0, 0.0, 3 * pi, pros::millis(), 500, false, false, false, 0, 0, 11000);
  // translationPID(56.0, 10.0, 3 * pi, pros::millis(), 800, false, false, false, 0, 0, 11000);
  // translationPID(52.0, 0, 2 * pi + pi / 2.0 + pi / 3.9, pros::millis(), 900, false, false, false, 0, 0, 10000);
  // shootingProcedure(false);
  //
  // // end run
  // translationPID(52.0, 9.0, 2 * pi + pi / 2.0 + pi / 4.05, pros::millis(), 600, false, false, false, 0, 0, 10000);
}
