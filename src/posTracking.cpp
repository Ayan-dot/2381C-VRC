#include "main.h"
#include "globals.hpp"
#include <cmath>

using namespace std;

class positionTracking {
    private:
        double angle = 0;
        double xtrans = 0, ytrans = 0;
        double xVec, yVec;
        double angOrientation = 0;

    public:
        positionTracking(double oldAng, double newAng, double vertEncoder, double horiEncoder) {
            angle = (newAng -  oldAng) * pi / 180 + angOrientation;
            
            angOrientation = angle;

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

};