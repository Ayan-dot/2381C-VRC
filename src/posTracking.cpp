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
        double xVec = 0, yVec = 0, rVec = 0, polAng = 0;

    public:
        positionTracking(double oldAng, double newAng, double angDiff, double vertEncoder, double horiEncoder, double oldX, double oldY) {
            angle = (newAng) * pi / 180 ;
            angDiff = (angDiff*pi/180);

            horiEncoder = (horiEncoder / 360.0) * 2.75 * pi;
            vertEncoder = (vertEncoder / 360.0) * 3.25 * pi;

            if(angDiff!=0){
                xtrans = horiEncoder/ angDiff * (pi / 180.0) + horizontalOffset;
                ytrans = vertEncoder / angDiff * (pi / 180.0) + verticalOffset;
                xVec = 2 * sin(angDiff / 2.0) * xtrans;
                yVec = 2 * sin(angDiff / 2.0) * ytrans;
            }
            else { 
                xVec = horiEncoder;
                yVec = vertEncoder;
            }
            
            rVec = pow(pow(xVec, 2) + pow(yVec, 2) , 0.5); 
            
            if(xVec!=0){
                polAng = atan(yVec / xVec) - (angDiff * (pi / 360) + oldAng);
            }
            else{
                polAng = - (angDiff * (pi / 360) + oldAng);
            }
            
            master.print(0, 0, "y: %f", yVec);
            pros::delay(50);
            
            master.print(1, 0, "X: %f", xVec);
            pros::delay(50);
            
            master.print(2, 0, "P %f", polAng);
            
            xVec = rVec * cos(polAng) + oldX;
            yVec = rVec * sin(polAng) + oldY; 
            
            pros::delay(50);
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
