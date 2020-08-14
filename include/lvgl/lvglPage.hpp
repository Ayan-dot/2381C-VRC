#pragma once
#include "main.h"
#include <sstream>
#include <iostream>
#include <string>

class odomBoi {
    public:
        
        odomBoi(int hue);
        
        void setData(double x, double y, double theta);

        void reset();

    private:

        lv_obj_t* led = nullptr; 

        lv_style_t greyT;
        lv_style_t redT;
        lv_style_t blueT;

        lv_obj_t *deviceTab;
        
        lv_obj_t *image;

        lv_obj_t *odomF;

        

        lv_style_t cStyle;
        lv_style_t fStyle; 

        lv_style_t textStyle, resetStyle, color_style;

        lv_obj_t *odomLabel;

        lv_obj_t* labelBoi;

        double fieldDim = 0;

        lv_color_t mainColor = LV_COLOR_BLUE;

        lv_style_t resetPr, resetRel;

        static lv_res_t tileAction(lv_obj_t*); // action when tile is pressed
        static lv_res_t resetAction(lv_obj_t*); // action when reset button is pressed

};