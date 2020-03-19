#include "main.h"
#include "globals.hpp"
#include <cmath>

using namespace std;

class motion {
    private:
        double desiredAngle = 0, distanceMove = 0;
    public:
        motion(double currentX, double currentY, double moveX, double moveY) {
            desiredAngle = atan((moveY - currentY)/(moveX - currentX));
            distanceMove = pow(pow((moveY - currentY), 2) + pow((moveX - currentX), 2), 0.5);
        }

        double returnAngle() {
            return desiredAngle;
        }

        double returnDistance() {
            return distanceMove;
        }
};

class positionTracking {
    private:
        double angle = 0;
        double xtrans = 0, ytrans = 0;
        double xVec, yVec;
        double placeholder;

    public:
        positionTracking(double oldAng, double newAng, double vertEncoder, double horiEncoder, double prevOrientation) {
            angle = (newAng -  oldAng) * pi / 180 + prevOrientation;
            
            prevOrientation = angle;
            placeholder = prevOrientation;

            xtrans = vertEncoder / angle + verticalOffset;
            ytrans = horiEncoder / angle + horizontalOffset;

            xVec = 2 * sin(angle / 2 * xtrans);
            yVec = 2 * sin(angle / 2 * ytrans);


        }

        double returnX() {
            return xVec;
        }
        
        double returnY() {
            return yVec;
        }

        double returnOrientation() {
            return placeholder;
        }

};