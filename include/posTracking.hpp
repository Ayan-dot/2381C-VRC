#include "main.h"
#include "globals.hpp"

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

class motion {
    public:
        motion(double currentX, double currentY, double moveX, double moveY);

        double returnAngle();

        double returnDistance();

    private:
        double desiredAngle = 0, distanceMove = 0;
        DIRECTION turn;
        double diffx = 0, diffy = 0;

};

class positionTracking {
    public:
        positionTracking(double lastAng, double currentX, double lastX, double currentYL, double lastYL, double currentYR, double lastYR);

        double returnX();

        double returnY();

        double returnOrient();

    private:
        double angle = 0;
        double L,R,B = 0;
        double halfang = 0;
        double globalang = 0;
        double h = 0;
        double h2 = 0;
        double xplacehold = 0, yplacehold = 0;
};