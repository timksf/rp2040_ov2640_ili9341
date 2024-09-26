#ifndef __GUI_H__
#define __GUI_H__

#include "lib/lvgl/lvgl.h"

#define CHECK_SETTINGS 3
#define COMBO_SETTINGS 7

typedef void (*apply_func_t)(uint32_t v);

typedef struct {
    const char *name;
    lv_obj_t *obj;
    apply_func_t apply;
    bool previous;
} camera_setting_cb_t;

typedef struct {
    const char *name;
    lv_obj_t *obj;
    apply_func_t apply;
    const char *options;
    uint32_t choice;
} camera_setting_combo_t;

void create_menu();
void load_main_page();

void hide_menu();
bool disp_camera_preview();

#endif //__GUI_H__