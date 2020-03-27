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
    double angle = 0; // difference of newAng and lastAng, in other words the angle traversed in the 10ms interval
    double halfang = 0; // half of the above angle 
    double globalang = 0; // variable to hold the global angle, which would be half the difference moved in the current local coordinate plane plus the last angle. 
    double h = 0; // displacement calculated through the vertical tracking wheel
    double h2 = 0; // displacement calculated through the horizontal tracking wheel
    double xplacehold = 0, yplacehold = 0; // placeholder for displacement vectors in both directon

public:
    positionTracking(double newAng, double lastAng, double currentX, double lastX, double currentY, double lastY)
    {
        angle = newAng - lastAng; // finding the difference between the angle 10ms ago and now 
        if (angle != 0) 
        {
            halfang = angle / 2.0; // finding half of the current angle
            h = 2.0 * sin(halfang) * ((currentY - lastY) / angle); // the first part is 2sin(theta/2) - the second part takes the finds the radius through last encoder position and current encoder position
            // (arc length) and angle. Since there is no offset, we do not add the offset to the radius
            h2 = 2.0 * sin(halfang) * (((currentX - lastX) / angle)+ horizontalOffset); // same principle with the horizontal tracking wheel, offset is added
        }
        else  // to avoid dividing by 0
        {
            h = currentY - lastY; // displacement is simply arc length since straight line
            h2 = currentX - lastX; // ""
            halfang = 0; 
        }

        globalang = lastAng + halfang; // finding the global angle through the required calculation
        yplacehold += h * cos(globalang); // the following 4 lines find the x and y global vectors to be added from the local net displacement vectors
        xplacehold += h * sin(globalang);
        yplacehold += h2 * (-sin(globalang));
        xplacehold += h2 * cos(globalang);
    }

    double returnX()
    {
        return xplacehold; // returns x vector to be added to global coordinate
    }

    double returnY()
    {
        return yplacehold; // returns y vector to be added to global coordinate
    }
};
