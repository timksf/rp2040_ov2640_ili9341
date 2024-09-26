#include "gui.h"

#include "lib/ov2640/ov2640.h"

static lv_obj_t *menu;
static lv_obj_t *main_page;
static bool camera_preview_enabled;

extern ov2640_config_t cam_config;

void apply_test_pattern(uint32_t v) {
    bool b = (bool) v;
    printf("Set test pattern: %d", b);
}

void apply_hflip(uint32_t v) {
    bool b = (bool) v;
    printf("Set HFLIP: %d", b);
}

void apply_vflip(uint32_t v) {
    bool f = (bool) v;
    printf("Set VFLIP: %d", f);
    ov2640_set_vflip(&cam_config, f);
}

void apply_not_implemented(uint32_t v) {
    printf("Apply not implemented (%d)", v);
}

static camera_setting_cb_t settings_check[CHECK_SETTINGS] = {
    {"Test Pattern", NULL, &apply_test_pattern, false},
    {"Horizontal Flip", NULL, &apply_hflip, false},
    {"Vertical Flip", NULL, &apply_vflip, false}
};

static camera_setting_combo_t settings_combo[COMBO_SETTINGS] = {
    {"Brightness", NULL,        &apply_not_implemented,"Level-2\nLevel-1\nLevel0\nLevel+1\nLevel+2", 0},
    {"Contrast", NULL,          &apply_not_implemented,"Level-2\nLevel-1\nLevel0\nLevel+1\nLevel+2", 0},
    {"Saturation", NULL,        &apply_not_implemented,"Level-2\nLevel-1\nLevel0\nLevel+1\nLevel+2", 0},
    {"Special Effect", NULL,    &apply_not_implemented,"No Effect\nNegative\nBlack&White\nReddish\nGreenish\nBlue\nRetro"},
    {"White Balance", NULL,     &apply_not_implemented,"Sunny\nCloudy\nOffice\nHome", 0},
    {"Auto Exposure", NULL,     &apply_not_implemented,"Preset0\nPreset1\nPreset2\nPreset3\nPreset4", 0},
    {"JPEG Resolution", NULL,   &apply_not_implemented,"320x240(QVGA)\n400x296(CIF)\n480x320(HVGA)\n640x480(VGA)\n800x600(SVGA)\n1280x720(HD)\n1280x1024(SXGA)\n1600x1200(UXGA)", 0}
};

static void first_btn_focus_handler(lv_event_t * e) {
    lv_obj_t * label = lv_event_get_user_data(e);
    if (lv_event_get_code(e) == LV_EVENT_FOCUSED) {
        lv_obj_scroll_to_view(label, LV_ANIM_OFF);
    }
}

static void checkbox_handler(lv_event_t *e) {
    lv_obj_t *obj = lv_event_get_target(e);
    bool checked = lv_obj_get_state(obj) & LV_STATE_CHECKED;
    camera_setting_cb_t *setting = lv_event_get_user_data(e);

    //F messageboxes
    if(setting->apply)
        setting->apply(checked);
}

static void combobox_handler(lv_event_t *e) {
    lv_obj_t *obj = lv_event_get_target(e);
    uint16_t v = lv_dropdown_get_selected(obj);
    camera_setting_combo_t *setting = lv_event_get_user_data(e);

    //F messageboxes
    if(setting->apply)
        setting->apply(v);
}

static void open_camera_preview(lv_event_t *e) {
    hide_menu();
    camera_preview_enabled = true;
}

void create_menu(lv_obj_t *parent) {
    menu = lv_menu_create(parent);
    lv_obj_set_size(menu, lv_obj_get_width(parent), lv_obj_get_height(parent));

    lv_obj_t *menu_cont;
    lv_obj_t *list;
    lv_obj_t *list_btn;
    lv_obj_t *list_txt;
    lv_obj_t *checkbox;
    lv_label_t *label;
    const int32_t btn_width = lv_obj_get_width(parent)/2;

    static lv_style_t menu_item_style;
    lv_style_init(&menu_item_style);
    lv_style_set_border_width(&menu_item_style, 2);
    // lv_style_set_border_color(&menu_item_style, lv_coloor);

    //create settings subpage
    lv_obj_t *settings_page = lv_menu_page_create(menu, "Settings");
    list = lv_list_create(settings_page);
    lv_obj_set_size(list, lv_obj_get_width(parent), lv_obj_get_height(parent));

    //separator label
    list_txt = lv_list_add_text(list, "Camera");

    for(uint8_t i = 0; i < CHECK_SETTINGS; i++) {
        list_btn = lv_list_add_button(list, NULL, settings_check[i].name);
        if(i == 0){
            lv_obj_add_event_cb(list_btn, first_btn_focus_handler, LV_EVENT_FOCUSED, list_txt);
        }
        checkbox = lv_checkbox_create(list_btn);
        lv_checkbox_set_text(checkbox, "");
        lv_obj_align(checkbox, LV_ALIGN_RIGHT_MID, 0, 0);
        settings_check[i].obj = checkbox;
        lv_obj_add_event_cb(checkbox, checkbox_handler, LV_EVENT_VALUE_CHANGED, &settings_check[i]);
    }

    //dropdown menus
    for (uint8_t i = 0; i < COMBO_SETTINGS; i++) {
        list_btn = lv_list_add_button(list, NULL, settings_combo[i].name);
        lv_obj_t *dd = lv_dropdown_create(list_btn);
        lv_obj_align(dd, LV_ALIGN_RIGHT_MID, 0, 0);
        lv_dropdown_set_options(dd, settings_combo[i].options);
        settings_combo[i].obj = dd;
        lv_obj_add_event_cb(dd, combobox_handler, LV_EVENT_VALUE_CHANGED, &settings_combo[i]);
    }

    //create empty preview page as placeholder
    lv_obj_t *preview_page = lv_menu_page_create(menu, NULL);

    //create explorer page (lv_list as content)
    lv_obj_t *files_page = lv_menu_page_create(menu, "File Explorer");
    list = lv_list_create(files_page);
    lv_obj_set_size(list, lv_obj_get_width(parent), lv_obj_get_height(parent));


    list_btn = lv_list_add_btn(list, NULL, "Item 1");
    checkbox = lv_checkbox_create(list_btn);
    lv_checkbox_set_text(checkbox, "Option1");
    lv_obj_align(checkbox, LV_ALIGN_RIGHT_MID, -10, 0);

    list_btn = lv_list_add_btn(list, NULL, "Item 2");
    checkbox = lv_checkbox_create(list_btn);
    lv_checkbox_set_text(checkbox, "Option2");
    lv_obj_align(checkbox, LV_ALIGN_RIGHT_MID, -10, 0);

    //create main page
    main_page = lv_menu_page_create(menu, NULL);

    menu_cont = lv_menu_cont_create(main_page);
    lv_obj_set_width(menu_cont, btn_width);
    label = lv_label_create(menu_cont);
    lv_label_set_text(label, "Settings");
    lv_menu_set_load_page_event(menu, menu_cont, settings_page);
    lv_group_add_obj(lv_group_get_default(), menu_cont);

    menu_cont = lv_menu_cont_create(main_page);
    lv_obj_set_width(menu_cont, btn_width);
    label = lv_label_create(menu_cont);
    lv_label_set_text(label, "Camera Preview");
    lv_obj_add_event_cb(menu_cont, open_camera_preview, LV_EVENT_CLICKED, NULL);
    lv_menu_set_load_page_event(menu, menu_cont, preview_page);
    lv_group_add_obj(lv_group_get_default(), menu_cont);

    menu_cont = lv_menu_cont_create(main_page);
    lv_obj_set_width(menu_cont, btn_width);
    label = lv_label_create(menu_cont);
    lv_label_set_text(label, "File Explorer");
    lv_obj_center(label);
    lv_menu_set_load_page_event(menu, menu_cont, files_page);
    lv_group_add_obj(lv_group_get_default(), menu_cont);

    //version info
    label = lv_label_create(main_page);
    lv_obj_add_flag(label, LV_OBJ_FLAG_FLOATING);
    lv_label_set_text(label, "Version 1.0");
    lv_obj_align(label, LV_ALIGN_BOTTOM_RIGHT, 0, 0);

    lv_obj_t *back_btn = lv_menu_get_main_header_back_button(menu);

    lv_menu_set_page(menu, main_page);
    camera_preview_enabled = false;
}

void load_main_page() {
    lv_obj_clear_flag(menu, LV_OBJ_FLAG_HIDDEN);
    lv_menu_clear_history(menu);
    lv_menu_set_page(menu, main_page);
    // lv_group_focus_next(lv_group_get_default());
    camera_preview_enabled = false;
}

void hide_menu() {
    lv_obj_add_flag(menu, LV_OBJ_FLAG_HIDDEN);
}

bool disp_camera_preview() {
    return camera_preview_enabled;
}