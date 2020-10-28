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

class motion // motion profiling class to be called during programming run
{
private:
    long double desiredAngle = 0, distanceMove = 0; // coordinate error values, turn direction, and total remaining distance
    DIRECTION turn;
    long double diffx = 0, diffy = 0; 

public:
    motion(long double currentX, long double currentY, long double moveX, long double moveY)
    {
        diffx = moveX - currentX; // calculate errors in coordinate positions
        diffy = moveY - currentY;

        desiredAngle = atan2(diffy, diffx) * 180 / pi; // calculate desired vector of motion based on error
        distanceMove = pow(pow((diffy), 2) + pow((diffx), 2), 0.5);
    }

    long double returnAngle() // return functions
    {
        return desiredAngle;
    }

    long double returnDistance()
    {
        return distanceMove;
    }
};

class positionTracking
{
private:
    long double angle = 0; // angle placeholder variable for the state function
    long double L,R,B = 0; // tracking wheel distance placeholders 
    long double halfang = 0; // half the angle error
    long double globalang = 0; // global angle, (angle relative to global coordinate plane)
    long double h = 0; // location vector calculated with vertical tracking wheel arc 
    long double h2 = 0; // location vector calculated with vertical tracking wheel arc 
    long double xplacehold = 0, yplacehold = 0; // placeholder values for additions to x and y coordinates

public:
    positionTracking(long double lastAng, long double currentX, long double lastX, long double currentYL, long double lastYL, long double currentYR, long double lastYR)
    {
        B = currentX - lastX; // next 3 lines are to calculate deltaS, or change in distance recorded by our tracking wheels in all 2D planes
        L = currentYL - lastYL;
        R = currentYR - lastYR;

        angle = (L-R) / (verticalOffset1+verticalOffset2); // calculate delta angle
        // arc length formula: r (theta) = s
        // theta = s / r
        //angle = B / horizontalOffset;

        if (angle != 0) // exceptions are thrown when dividing by zero, and if angle change is zero, the vectors can simply be resolved to the vertical and horizontal distances
        {
            halfang = angle / 2.0;
            h = 2.0 * sin(halfang) * (((R) / angle) + verticalOffset2);
            h2 = 2.0 * sin(halfang) * (((B) / angle)+ horizontalOffset);
        }
        else
        {
            h = R;
            h2 = B;
            halfang = 0;
        }

        globalang = lastAng + halfang; // finds net heading that robot moved towards
        yplacehold += h * cos(globalang); // next 4 lines find cartesian displacement through rotated vectors from local headings to global headings
        xplacehold += h * sin(globalang);
        yplacehold += h2 * (-sin(globalang));
        xplacehold += h2 * cos(globalang);
        globalang += halfang;
    }

    long double returnX() // return functions
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