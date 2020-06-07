#include "main.h"
#include <cmath>
#include "posTracking.hpp"



    motion::motion(double currentX, double currentY, double moveX, double moveY)
    {
        diffx = moveX - currentX;
        diffy = moveY - currentY;

        desiredAngle = atan2(diffy, diffx) * 180 / pi;
        distanceMove = pow(pow((diffy), 2) + pow((diffx), 2), 0.5);
    }

    double motion::returnAngle()
    {
        return desiredAngle;
    }

    double motion::returnDistance()
    {
        return distanceMove;
    }

    positionTracking::positionTracking(double lastAng, double currentX, double lastX, double currentYL, double lastYL, double currentYR, double lastYR)
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

    double positionTracking::returnX()
    {
        return xplacehold;
    }

    double positionTracking::returnY()
    {
        return yplacehold;
    }
    double positionTracking::returnOrient(){
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