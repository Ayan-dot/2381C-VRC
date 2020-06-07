#include "globals.hpp"
#include "lvgl/lvglPage.hpp"

#define DEFAULT 1

using namespace std;

lv_theme_t *th = lv_theme_material_init(200, NULL);
lv_obj_t * scr = lv_obj_create(NULL, NULL);


static lv_obj_t * slider_label;

static lv_res_t slider_action(lv_obj_t * slide) {
    
    printf("New slider value: %d\n", lv_slider_get_value(slide));
    return LV_RES_OK;
    
}

lv_obj_t * createSlide(lv_obj_t * parent, const char * label2, int16_t max, lv_coord_t xTR, lv_coord_t yTR, lv_coord_t xLM, lv_coord_t yLM, lv_coord_t slideX, lv_coord_t slideY) {
    lv_obj_t * slide = lv_slider_create(parent, NULL);

    lv_obj_t * label = lv_label_create(parent, NULL);
    lv_label_set_text(label, label2);
    lv_obj_align(slide, NULL, LV_ALIGN_IN_TOP_RIGHT, xTR, yTR);
    lv_obj_align(label, slide, LV_ALIGN_OUT_LEFT_MID, xLM, yLM);
    
    
    lv_obj_set_size(slide, slideX, slideY);
    //lv_obj_set_pos(slide, x, y);

    lv_slider_set_range(slide, 0, max);
    
    return slide;
}

odomBoi::odomBoi() {
    
    lv_theme_set_current(th);

    tabview = lv_tabview_create(lv_scr_act(), NULL);

    red = lv_tabview_add_tab(tabview, "Red");
    image = lv_tabview_add_tab(tabview, "Image");
    blue = lv_tabview_add_tab(tabview, "Blue");

    deviceTab = lv_tabview_add_tab(tabview, "Devices");


    lv_slider_set_action(createSlide(deviceTab, "Left Front", 200, -30, 0, -20, -5, 250, 30), slider_action);
    
    createSlide(deviceTab, "Left Back", 200, -30, 70, -20, -5, 250, 30);

    createSlide(deviceTab, "Right Front", 200, -30, 110, -20, -5, 250, 30);

    createSlide(deviceTab, "Right Back", 200, -30, 150, -20, -5, 250, 30);
    
    odomF = lv_tabview_add_tab(tabview, "Odom");

    

    lv_obj_t* field = lv_obj_create(odomF, NULL);
    fieldDim = min(lv_obj_get_width(odomF), lv_obj_get_height(odomF));

    fieldDim = fieldDim - 22;
    lv_obj_set_size(field, fieldDim, fieldDim);
    lv_obj_align(field, NULL, LV_ALIGN_IN_RIGHT_MID, -40, 0);
    
    lv_style_copy(&greyT, &lv_style_plain);
    greyT.body.main_color = LV_COLOR_HEX(0x828F8F);
    greyT.body.grad_color = LV_COLOR_HEX(0x828F8F);
    greyT.body.border.width = 1;
    greyT.body.radius = 0;
    greyT.body.border.color = LV_COLOR_WHITE;

    lv_style_copy(&redT, &greyT);
    redT.body.main_color = LV_COLOR_RED;
    redT.body.grad_color = LV_COLOR_RED;
    lv_style_copy(&blueT, &greyT);
    blueT.body.main_color = LV_COLOR_BLUE;
    blueT.body.grad_color = LV_COLOR_BLUE;


    std::vector<std::vector<lv_style_t*>> tileData = {
    {&greyT, &redT , &greyT, &greyT, &blueT, &greyT},
    {&redT, &greyT, &greyT, &greyT, &greyT, &blueT},
    {&greyT, &greyT, &greyT, &greyT, &greyT, &greyT},
    {&greyT, &greyT, &greyT, &greyT, &greyT, &greyT},
    {&greyT, &greyT, &greyT, &greyT, &greyT, &greyT},
    {&greyT, &greyT, &greyT, &greyT, &greyT, &greyT}
    };

    double tileDim = (fieldDim) / tileData.size();

    for(size_t y = 0; y < 6; y++) {
        for(size_t x = 0; x < 6; x++) {
        lv_obj_t* tileObj = lv_btn_create(field, NULL);
        lv_obj_set_pos(tileObj, x * tileDim, y * tileDim);
        lv_obj_set_size(tileObj, tileDim, tileDim);
        //lv_btn_set_action(tileObj, LV_BTN_ACTION_CLICK, tileAction);
        lv_obj_set_free_num(tileObj, y * 6 + x);
        lv_obj_set_free_ptr(tileObj, this);
        lv_btn_set_toggle(tileObj, false);
        lv_btn_set_style(tileObj, LV_BTN_STYLE_PR, tileData[y][x]);
        lv_btn_set_style(tileObj, LV_BTN_STYLE_REL, tileData[y][x]);
        }
    }

    led = lv_led_create(field, NULL);
    lv_led_on(led);
    lv_obj_set_size(led, (fieldDim) / 15, (fieldDim)/ 15);

    LV_IMG_DECLARE(logo2);
    lv_obj_t * logoBoi = lv_img_create(image, nullptr);
    lv_img_set_auto_size(logoBoi, true);
    lv_img_set_src(logoBoi, &logo2);
    lv_obj_set_pos(logoBoi, 300, 0);


    lv_obj_t * logoBoi2 = lv_img_create(image, nullptr);
    lv_img_set_auto_size(logoBoi2, true);
    lv_img_set_src(logoBoi2, &logo2);
    lv_obj_set_pos(logoBoi2, 150, 20);

    lv_obj_t * logoBoi3 = lv_img_create(image, nullptr);
    lv_img_set_auto_size(logoBoi2, true);
    lv_img_set_src(logoBoi3, &logo2);
    lv_obj_set_pos(logoBoi3, 0, 20);

}

void odomBoi::setData(double x, double y, double theta) {
    double fieldInch = 144/(fieldDim - 22);

    lv_obj_set_pos(led, x * fieldInch, y * fieldInch);

    lv_style_copy(&textStyle, &lv_style_plain);
    textStyle.text.color = LV_COLOR_BLACK;
    
    odomLabel = lv_label_create(odomF, NULL);
    lv_obj_set_pos(odomLabel, 20, 20);
    lv_label_set_style(odomLabel, &textStyle);
    
    std::string text =
    "X: " + std::to_string(x) + "\n" +
    "Y: " + std::to_string(y) + "\n" +
    "Theta: " + std::to_string(theta) + "\n";
    

    lv_label_set_text(odomLabel, text.c_str());

}

lv_res_t odomBoi::resetAction(lv_obj_t* btn) {
    
}

void odomBoi::reset() {
    lv_obj_t* btn = lv_btn_create(odomF, NULL);
    lv_obj_set_size(btn, 125, 50);
    lv_obj_set_pos(btn, 20, 100);
    
    lv_btn_set_action(btn, LV_BTN_ACTION_PR, resetAction);

    /**
     * Button Style
     */
    lv_style_copy(&resetRel, &lv_style_btn_tgl_rel);
    resetRel.body.main_color = mainColor;
    resetRel.body.grad_color = mainColor;
    resetRel.body.border.color = LV_COLOR_WHITE;
    resetRel.body.border.width = 2;
    resetRel.body.border.opa = LV_OPA_100;
    resetRel.body.radius = 2;
    resetRel.text.color = LV_COLOR_WHITE;

    lv_style_copy(&resetPr, &resetRel);
    resetPr.body.main_color = LV_COLOR_WHITE;
    resetPr.body.grad_color = LV_COLOR_WHITE;
    resetPr.text.color = mainColor;

    lv_btn_set_style(btn, LV_BTN_STYLE_REL, &resetRel);
    lv_btn_set_style(btn, LV_BTN_STYLE_PR, &resetPr);

    /**
    * Reset Button Label
    */
   /*Create a style*/
   
    lv_style_copy(&color_style, &lv_style_pretty_color);
    color_style.body.main_color = LV_COLOR_HEX3(0x666);     /*Line color at the beginning*/
    color_style.body.grad_color =  LV_COLOR_HEX3(0x666);    /*Line color at the end*/
    color_style.body.padding.hor = 10;                      /*Scale line length*/
    color_style.body.padding.inner = 8 ;                    /*Scale label padding*/
    color_style.body.border.color = LV_COLOR_HEX3(0x333);   /*Needle middle circle color*/
    color_style.line.width = 3;
    color_style.text.color = LV_COLOR_HEX3(0x333);
    color_style.line.color = LV_COLOR_RED;                  /*Line color after the critical value*/


    labelBoi = lv_label_create(btn, NULL);
    lv_obj_set_style(labelBoi, &color_style);
    lv_label_set_text(labelBoi, "Reset");
}

lv_coord_t min(lv_coord_t x, lv_coord_t y) {
    lv_coord_t min = 0;
    
    if(x > y) {
        min = y;
    }
    else {
        min = x;
    }

    return min;
}




void initialize()
{

    
    
    
    int time = pros::millis();
    int iter = 0;
   

   

    inertial.reset();

    // while (inertial.is_calibrating())
    // {
    //     printf("IMU calibrating... %d\n", iter);
    //     iter += 10;
    //     pros::delay(10);
    // }

    // printf("IMU is done calibrating (took %d ms)\n", iter - time);

    // pros::Task intake_task(intake_tasks_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,"Intake Task");
    // pros::Task vector2_task(printVector_tasks_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,"Vector Task");
}