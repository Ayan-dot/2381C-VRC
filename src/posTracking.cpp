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

posTracking.cpp [contains]:
  * Odometry class and method from which we use as a task in autonomous.cpp to track the robot's absolute position

NOTE: All relevant mathematical calculations (odometry and motion) are documented in extensive detail in our paper regarding robot motion
  * https://drive.google.com/file/d/1zBMroM90nDU6iHqsbI_qOgd120M7x-rd/view
  
*/

// Necessary imports
#include "main.h"
#include "globals.hpp"
#include <cmath>

using namespace std;

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

class motion
{
private:
    long double desiredAngle = 0, distanceMove = 0;
    DIRECTION turn;
    long double diffx = 0, diffy = 0;

public:
    motion(long double currentX, long double currentY, long double moveX, long double moveY)
    {
        diffx = moveX - currentX;
        diffy = moveY - currentY;

        desiredAngle = atan2(diffy, diffx) * 180 / pi;
        distanceMove = pow(pow((diffy), 2) + pow((diffx), 2), 0.5);
    }

    long double returnAngle()
    {
        return desiredAngle;
    }

    long double returnDistance()
    {
        return distanceMove;
    }
};

//****************//
// ODOMETRY CLASS //
//****************//
class positionTracking
{
private:
    long double angle = 0;                        // change in robot heading
    long double L,R,B = 0;                        // changes in distances tracked by tracking wheels (left, right, and back respectively)
    long double halfang = 0;                      // the change in robot heading (angle) divided by 2
    long double globalang = 0;                    // global absolute robot heading
    long double h = 0;                            // length of chord ON (see mathematical write up, or video)
    long double h2 = 0;                           // length of chord MN
    long double xplacehold = 0, yplacehold = 0;   // placeholder x and y displacements values that will be added to global x and y values

public:
    positionTracking(long double lastAng, long double currentX, long double lastX, long double currentYL, long double lastYL, long double currentYR, long double lastYR)
    {
        // Odometry calculations (see write up)
        B = currentX - lastX;
        L = currentYL - lastYL;
        R = currentYR - lastYR;

        angle = (L-R) / (verticalOffset1+verticalOffset2);
        // arc length formula: r (theta) = s
        // theta = s / r
        //angle = B / horizontalOffset;

        if (angle != 0)
        {
            halfang = angle / 2.0;
            h = 2.0 * sin(halfang) * (((R) / angle) + verticalOffset2);
            h2 = 2.0 * sin(halfang) * (((B) / angle)+ horizontalOffset);
        }
        else
        {
            // special case to angle division by zero error (when angle == 0)
            h = R;
            h2 = B;
            halfang = 0;
        }

        // convert all the local displacements onto the global coordinate system using trigonometry
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
    long double returnOrient(){
        return globalang;
    }
};
