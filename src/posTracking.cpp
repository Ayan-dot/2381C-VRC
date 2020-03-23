#include "main.h"
#include "globals.hpp"
#include <cmath>

using namespace std;

class motion
{
private:
    double desiredAngle = 0, distanceMove = 0;

public:
    motion(double currentX, double currentY, double moveX, double moveY)
    {
        desiredAngle = atan((moveY - currentY) / (moveX - currentX));
        distanceMove = pow(pow((moveY - currentY), 2) + pow((moveX - currentX), 2), 0.5);
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
    double displacement = 0;
    double displacement2 = 0;
    double xVec = 0, yVec = 0;
    double sum = 0;

public:
    positionTracking(double oldAng, double newAng, double angDiff, double vertEncoder, double horiEncoder, double oldX, double oldY)
    {
         vertEncoder = -vertEncoder;
         horiEncoder = -horiEncoder;

        angle = newAng * pi / 180.0;

        oldAng = oldAng * pi / 180.0;
        
        angDiff = angDiff * pi / 180.0;
        
        // parallel wheel portion
        if (angDiff != 0)
        {
            displacement = 2.0 * sin(angDiff / 2.0) * (vertEncoder / angDiff);

            // field centric
            yVec = displacement * cos(angDiff / 2.0 + oldAng);
            xVec = displacement * sin(angDiff / 2.0 + oldAng);
            
            // back wheel portion
            displacement2 = 2.0 * sin(angDiff / 2.0) * (horiEncoder / angDiff + horizontalOffset);
            //  field centric
            xVec += displacement2 * cos(angDiff / 2.0 + oldAng);
            yVec += displacement2 * -sin(angDiff / 2.0 + oldAng);
           
        }
        else
        {
            displacement = vertEncoder;
            xVec = displacement * cos(oldAng);
            yVec = displacement * sin(oldAng);
            
            displacement2 = horiEncoder;
            
            xVec += displacement2 * cos(oldAng);
            yVec += displacement2 * -sin(oldAng);
            
        }
    }

    double returnX()
    {
        
        return xVec;
    }

    double returnY()
    {
        return yVec;
    }

    double returnOrientation()
    {
        return angle;
    }
};
