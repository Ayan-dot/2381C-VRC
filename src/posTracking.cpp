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
        double displacement = 0;
        double xVec = 0, yVec = 0;

    public:
        positionTracking(double oldAng, double newAng, double angDiff, double vertEncoder, double horiEncoder, double oldX, double oldY) {
        angle = newAng*pi/180.0;
        oldAng = oldAng*pi/180.0;
        angDiff = angDiff*pi/180.0;
        displacement = 2*sin(angDiff/2)*(vertEncoder/angDiff);
        // field centric;
        xVec = displacement * cos(oldAng+angDiff/2.0);
        yVec = displacement * sin(oldAng+angDiff/2.0);

        displacement = 2*sin(angDiff/2)*(horiEncoder/angDiff+horizontalOffset);
        xVec += displacement * cos(oldAng+angDiff/2.0);
        yVec += displacement * -sin(oldAng+angDiff/2.0);

        }

        double returnX() {
            return xVec;
        }

        double returnY() {
            return yVec;
        }

        double returnOrientation() {
            return angle;
        }

};
