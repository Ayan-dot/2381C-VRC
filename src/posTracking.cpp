#include "main.h"
#include <cmath>
#include "posTracking.hpp"

    // long double desiredAngle = 0, distanceMove = 0;
    // DIRECTION turn;
    // long double diffx = 0, diffy = 0;

    motion::motion(long double currentX, long double currentY, long double moveX, long double moveY)
    {
        diffx = moveX - currentX;
        diffy = moveY - currentY;

        desiredAngle = atan2(diffy, diffx) * 180 / pi;
        distanceMove = pow(pow((diffy), 2) + pow((diffx), 2), 0.5);
    }

    long double motion::returnAngle()
    {
        return desiredAngle;
    }

    long double motion::returnDistance()
    {
        return distanceMove;
    }


    positionTracking::positionTracking(long double lastAng, long double currentX, long double lastX, long double currentYL, long double lastYL, long double currentYR, long double lastYR)
    {
        B = currentX - lastX;
        L = currentYL - lastYL;
        R = currentYR - lastYR;

        angle = (L-R) / (verticalOffset1+verticalOffset2);

        // arc length formula: r (theta) = s
        // theta = s / r
        //angle = B / horizontalOffset;

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

    long double positionTracking::returnX()
    {
        return xplacehold;
    }

    long double positionTracking::returnY()
    {
        return yplacehold;
    }

  long double positionTracking::returnOrient(){
        return globalang;
  }
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
//     turnCorrection(long double reqAng)

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
//     long double returnR(){
//         return R_adj;
//     }
//     long double returnL(){
//         return L_adj;
//     }
// };
//     double returnR(){
//         return R_adj;
//     }
//     double returnL(){
//         return L_adj;
//     }
// };
