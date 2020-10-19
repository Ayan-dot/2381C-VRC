/*
  ___  ____   ___  __  _____ 
 |__ \|___ \ / _ \/_ |/ ____|
    ) | __) | (_) || | |     
   / / |__ < > _ < | | |     
  / /_ ___) | (_) || | |____ 
 |____|____/ \___/ |_|\_____|

All code is the property of 2381C, Kernel Bye. ANY UNAUTHORIZED REPRODUCTION
OR DISTRIBUTION OF THIS CODE IS STRICTLY FORBIDDEN. Please contact team 2381C
directly with any questions, concerns or suggestions you may have.
*/

#include "main.h"
#include "globals.hpp"

class Intakes
{
private:
    int intakeSetting = 0;

public:
    Intakes(bool outtake, bool intake)
    {
        if (outtake)
        {
            intakeSetting = 1;
        }

        if (intake)
        {
            intakeSetting = 0;
        }

        if (!(outtake && intake))
        {
            intakeSetting = 2;
        }
    }

    double leftSpeed()
    {
        if (intakeSetting == 1)
        {
            return -4800;
        }
        else if (intakeSetting == 0)
        {
            return 12000;
        }
        else
        {
            return 0;
        }
    }

    double rightSpeed()
    {
        if (intakeSetting == 1)
        {
            return 4800;
        }
        else if (intakeSetting == 0)
        {
            return -12000;
        }
        else
        {
            return 0;
        }
    }
};