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
    long double diffx = 0, diffy = 0;
    DIRECTION turn;

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

class positionTracking
{
private:
    long double angle = 0;
    long double L, R, B = 0;
    long double halfang = 0;
    long double globalang = 0;
    long double h = 0;
    long double h2 = 0;
    long double xplacehold = 0, yplacehold = 0;

public:
    positionTracking(long double lastAng, long double currentX, long double lastX, long double currentYL, long double lastYL, long double currentYR, long double lastYR)
    {
        B = currentX - lastX;
        L = currentYL - lastYL;
        R = currentYR - lastYR;

        angle = (L - R) / (verticalOffset1 + verticalOffset2);
        // arc length formula: r (theta) = s
        // theta = s / r
        // angle = B / horizontalOffset;

        if (angle != 0)
        {
            halfang = angle / 2.0;
            h = 2.0 * sin(halfang) * (((R) / angle) + verticalOffset2);
            h2 = 2.0 * sin(halfang) * (((B) / angle) + horizontalOffset);
        }
        else
        {
            h = R;
            h2 = B;
            halfang = 0;
        }

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