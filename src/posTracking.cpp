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
    double L,R,B = 0;
    double halfang = 0;
    double globalang = 0;
    double h = 0;
    double h2 = 0;
    double xplacehold = 0, yplacehold = 0;

public:
    positionTracking(double lastAng, double currentX, double lastX, double currentYL, double lastYL, double currentYR, double lastYR)
    {
        B = currentX - lastX;
        L = currentYL - lastYL;
        R = currentYR - lastYR;

        angle = (L-R) / (verticalOffset1+verticalOffset2);
        
            
        if (angle != 0)
        {
            halfang = angle / 2.0;
            h = 2.0 * sin(halfang) * (((R) / angle) + verticalOffset2);              
            h2 = 2.0 * sin(halfang) * (((B) / angle)+ horizontalOffset);
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

    double returnX()
    {
        return xplacehold;
    }

    double returnY()
    {
        return yplacehold;
    }
    double returnOrient(){
        return globalang;
    }
};
// class turnCorrection
// {
// private:
// double factor = 0;
// double correction = 0;
// double L_adj = 0, R_adj = 0;

// PID* adjustmentPIDController = new PID(
//     &adjustmentPIDParams[0],
//     &adjustmentPIDParams[1],
//     &adjustmentPIDParams[2]);

// public: 
//     turnCorrection(double reqAng)
//     {
//     if(inertial.get_rotation!=reqAng){
//      correction = inertial.get_rotation - reqAng;
//         while(correction>0){
//         factor = adjustmentPIDController->update(correction,0);
//         R_adj = factor;   
//     }
//         while(correction<0){
//         factor = adjustmentPIDController->update(correction, 0);
//         L_adj = factor;
//         }

// }
//     }
//     double returnR(){
//         return R_adj;
//     }
//     double returnL(){
//         return L_adj;
//     }
// };
