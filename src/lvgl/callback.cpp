#include "lvgl/callback.hpp"
#include "globals.hpp"

namespace variables {
    lv_obj_t *blueBoi;
    lv_obj_t *redBoi;

	int auton = 0;
	int autonCount = 0;

	void tabWatcher() {
		int activeTab = lv_tabview_get_tab_act(tabview);
		
		if(activeTab < 2) {
			while(1){
				int currentTab = lv_tabview_get_tab_act(tabview);

				if(currentTab != activeTab){
					activeTab = currentTab;
					if(activeTab == 0){
						if(auton == 0) auton = 1;
						auton = abs(auton);
						lv_btnm_set_toggle(red, true, abs(auton)-1);
					}else if(activeTab == 1){
						if(auton == 0) auton = -1;
						auton = -abs(auton);
						lv_btnm_set_toggle(blue, true, abs(auton)-1);
					}else{
						auton = 0;
					}
				}

				pros::delay(20);
			}
		}
			
	}

lv_res_t blueAction(lv_obj_t *btnm, const char *txt)
{
	//printf("blue button: %s released\n", txt);

	for(int i = 0; i < autonCount; i++){
		if(strcmp(txt, btnMap[i]) == 0){
			auton = -(i+1);
		}
	}

	return LV_RES_OK; // return OK because the button matrix is not deleted
}

lv_res_t redAction(lv_obj_t *btnm, const char *txt){
	//printf("red button: %s released\n", txt);

	for(int i = 0; i < autonCount; i++){
		if(strcmp(txt, btnMap[i]) == 0){
			auton = i+1;
		}
	}

	return LV_RES_OK; // return OK because the button matrix is not deleted
}

void initAuton(int default_auton, const char **autons) {
		int i = 0;
			do{
				memcpy(&btnMap[i], &autons[i], sizeof(&autons));
				i++;
			}while(strcmp(autons[i], "") != 0);

		autonCount = i;
		auton = default_auton;

		redBoi = lv_btnm_create(red, NULL);
		lv_btnm_set_map(redBoi, btnMap);
		lv_btnm_set_action(redBoi, redAction);
		lv_btnm_set_toggle(redBoi, true, abs(auton)-1);//3
		lv_obj_set_size(redBoi, 450, 50);
		lv_obj_set_pos(redBoi, 0, 100);
		lv_obj_align(redBoi, NULL, LV_ALIGN_CENTER, 0, 0);

		blueBoi = lv_btnm_create(blue, NULL);
		lv_btnm_set_map(blueBoi, btnMap);
		lv_btnm_set_action(blueBoi, redAction);
		lv_btnm_set_toggle(blueBoi, true, abs(auton)-1);//3
		lv_obj_set_size(blueBoi, 450, 50);
		lv_obj_set_pos(blueBoi, 0, 100);
		lv_obj_align(blueBoi, NULL, LV_ALIGN_CENTER, 0, 0);

		if(auton < 0){
			lv_tabview_set_tab_act(tabview, 1, LV_ANIM_NONE);
		}else if(auton == 0){
			lv_tabview_set_tab_act(tabview, 2, LV_ANIM_NONE);
		}

		pros::Task tabWatcher_task(tabWatcher);
	}


}
