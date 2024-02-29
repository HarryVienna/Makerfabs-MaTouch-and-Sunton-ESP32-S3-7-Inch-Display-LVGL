#include "esp_log.h"

#include "gui.h"


static const char* TAG = "GUI";

void set_brightness(int32_t value) {

    set_backlight_brightness(value);

    char str[10];
    sprintf( str, "%lu", value );

    lv_label_set_text(ui_LabelBrightness, str);
}

void disp_counter(int32_t value) {

    char str[10];
    sprintf( str, "%lu", value );

    lv_label_set_text(ui_LabelCounter, str);
}

void event_screen_init(lv_event_t * e) {
    set_brightness(2);
}

void event_slider_set_backlight_brightness(lv_event_t * e) {
    lv_obj_t * slider = lv_event_get_target(e);
    int32_t value = lv_slider_get_value(slider);
    
    set_brightness(value);
}