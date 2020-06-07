#pragma once
#include "main.h"
#include <sstream>
#include <iostream>
#include <string>

static const char *btnMap[] = {"Left", "Right", "Skills", ""};

int auton;
int autonCount;

class autons {
    public:
        autons(int default_auton, const char **autons);
        
        int tabWatcher();

    private:

        lv_obj_t *blueBoi;
        lv_obj_t *redBoi;

        static lv_res_t blueAction(lv_obj_t*, const char*);
        static lv_res_t redAction(lv_obj_t*, const char*);
};
