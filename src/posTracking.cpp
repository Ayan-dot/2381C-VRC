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

posTracking.cpp [contains]:
  - Odometry class and method from which we use as a task in autonomous.cpp to
    track the robot's absolute position
*/
#include "main.h"
#include "globals.hpp"
#include <cmath>
using namespace std;

// total set of possible directions defined as enum
enum DIRECTION
{
    FORWARD,
    REVERSE,
    LEFT,
    RIGHT,
    LEFT_FORWARD,
    LEFT_BACK,
    RIGHT_FORWARD,
    RIGHT_BACK,
    TRANS_UP,
    TRANS_DOWN
};

/**
 * The motion class performs basic calculations regarding the position and
 * heading of the robot.
 */
class motion
{
private:
    long double desiredAngle = 0, distanceMove = 0;
    DIRECTION turn;
    long double diffx = 0, diffy = 0;

public:
    /**
     * Initialize the class by taking current and target values. As well,
     * compute the angle between the two positions.
     *
     * @param currentX the current x position
     * @param currentY the current y position
     * @param moveX the target x position
     * @param moveY the target y position
     */
    motion(long double currentX, long double currentY, long double moveX, long double moveY)
    {
        diffx = moveX - currentX;
        diffy = moveY - currentY;

        desiredAngle = atan2(diffy, diffx) * 180 / pi;
        distanceMove = pow(pow((diffy), 2) + pow((diffx), 2), 0.5);
    }

    // return the angle difference between the points
    long double returnAngle()
    {
        return desiredAngle;
    }

    // return the Euclidean distance between the points
    long double returnDistance()
    {
        return distanceMove;
    }
};

/**
 * The odometry class which keepts track of the actual postion of the robot
 * at all times.
 */
class positionTracking
{
private:
    // NOTE: if you don't understand what these variables refer to, read the
    // odometry paper in the docs (documentation) folder to learn more

    // change in robot heading
    long double angle = 0;

    // changes in distances tracked by tracking wheels (left, right, and back respectively)
    long double L, R, B = 0;

    // the change in robot heading (angle) divided by 2
    long double halfang = 0;

    // global absolute robot heading
    long double globalang = 0;

    // length of chord ON (see mathematical write up, or video)
    long double h = 0;

    // length of chord MN
    long double h2 = 0;

    // placeholder x and y displacements values that will be added to global x and y values
    long double xplacehold = 0, yplacehold = 0;

public:
    /**
     * The position tracking procedure requires complex math that cannot be
     * adequately explained within this source code. To gain a thorough
     * understanding of this procedure, please read the math paper presented
     * in our documentation folder. If you have any questions or concerns,
     * please contact our team captain Allen Tao <allentao7@gmail.com>.
     *
     * This function makes use of proofs and investigations made in the paper.
     */
    positionTracking(long double lastAng, long double currentX, long double lastX, long double currentYL, long double lastYL, long double currentYR, long double lastYR)
    {
        B = currentX - lastX;
        L = currentYL - lastYL;
        R = currentYR - lastYR;
        angle = (L - R) / (verticalOffset1 + verticalOffset2);
        // arc length formula: r (theta) = s
        // theta = s / r
        //angle = B / horizontalOffset;
        //angle = L / verticalOffset1;
        //angle = R / verticalOffset2;
        if (angle != 0)
        {
            halfang = angle / 2.0;
            h = 2.0 * sin(halfang) * (((R) / angle) + verticalOffset2);
            h2 = 2.0 * sin(halfang) * (((B) / angle) + horizontalOffset);
        }

        else
        {
            // special case to angle division by zero error (when angle == 0)
            h = R;
            h2 = B;
            halfang = 0;
        }

        // convert local displacements to the global coordinate system
        globalang = lastAng + halfang;
        yplacehold += h * cos(globalang);
        xplacehold += h * sin(globalang);
        yplacehold += h2 * (-sin(globalang));
        xplacehold += h2 * cos(globalang);
        globalang += halfang;
    }

    long double returnX()
    {
        return xplacehold;
    }

    long double returnY()
    {
        return yplacehold;
    }

    long double returnOrient()
    {
        return globalang;
    }
};
