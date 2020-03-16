#include "main.h"
#include "globals.hpp"

class intakes {
    private:
        int intakeSetting = 0;

    public:
        intakes(bool outtake, bool intake) {
            if(outtake) {
               intakeSetting = 1; 
            }
            
            if(intake) {
                intakeSetting = 0;
            }

            if(!(outtake && intake)) {
                intakeSetting = 2;
            }
        }

        double leftSpeed() {
            if(intakeSetting == 1) {
               return -4800; 
            }
            else if(intakeSetting == 0){
                return 12000;
            }
            else {
                return 0;
            }
        }

        double rightSpeed() {
            if(intakeSetting == 1) {
                return 4800;
            }
            else if(intakeSetting == 0){
                return -12000;
            }
            else {
                return 0;
            }
        }
};