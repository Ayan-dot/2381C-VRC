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
    double desiredAngle = 0, distanceMove = 0;
    DIRECTION turn;
    double diffx = 0, diffy = 0;

public:
    motion(double currentX, double currentY, double moveX, double moveY)
    {
        diffx = moveX - currentX;
        diffy = moveY - currentY;

        desiredAngle = atan2(diffy, diffx) * 180 / pi;
        distanceMove = pow(pow((diffy), 2) + pow((diffx), 2), 0.5);
    }

    double returnAngle()
    {
        return desiredAngle;
    }

    double returnDistance()
    {
        return distanceMove;
    }
};

class positionTracking
{
private:
    double angle = 0;
    double halfang = 0;
    double globalang = 0;
    double h = 0;
    double h2 = 0;
    double xplacehold = 0, yplacehold = 0;

public:
    positionTracking(double newAng, double lastAng, double currentX, double lastX, double currentY, double lastY)
    {
        angle = newAng - lastAng;
        if (angle != 0)
        {
            halfang = angle / 2.0;
            h = 2.0 * sin(halfang) * ((currentY - lastY) / angle);
            h2 = 2.0 * sin(halfang) * (((currentX - lastX) / angle) + horizontalOffset);
        }
        else
        {
            h = currentY - lastY;
            h2 = currentX - lastX;
            halfang = 0;
        }

        globalang = lastAng + halfang;
        yplacehold += h * cos(globalang);
        xplacehold += h * sin(globalang);
        yplacehold += h2 * (-sin(globalang));
        xplacehold += h2 * cos(globalang);
    }

    double returnX()
    {
        return xplacehold;
    }

    double returnY()
    {
        return yplacehold;
    }
};
