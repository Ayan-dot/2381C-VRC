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
        double xVec, yVec, rVec, polAng;

    public:
        positionTracking(double oldAng, double newAng, double angDiff, double vertEncoder, double horiEncoder, double oldX, double oldY) {
            angle = (newAng) * pi / 180 ;

            if(angDiff != 0){
                xtrans = horiEncoder/ angDiff * (pi / 180) + horizontalOffset;
                ytrans = vertEncoder / angDiff * (pi / 180) + verticalOffset;
                xVec = 2 * sin(angDiff / 2) * xtrans;
                yVec = 2 * sin(angDiff / 2) * ytrans;}
            else { // yippee no math
                xVec = horiEncoder;
                yVec = vertEncoder;
            }

            rVec = pow(pow(xVec, 2) + pow(yVec, 2) , 0.5);
            polAng = atan(yVec / xVec) - (angDiff*(pi / 360) + oldAng);
            xVec = rVec * cos(polAng) + oldX;
            yVec = rVec * sin(polAng) + oldY;



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
