#include "esp_log.h"

#include "lvgl.h"
#include "../ui/ui.h"
#include "../display/matouch_esp32_s3.h"

void set_brightness(int32_t value) {

    set_backlight_brightness(value);

    char str[10];
    sprintf( str, "%lu", value );

    lv_label_set_text(ui_LabelBrightness, str);
}

void screen_event_init(lv_event_t * e) {
    set_brightness(2);
}

void slider_event_set_backlight_brightness(lv_event_t * e) {
    lv_obj_t * slider = lv_event_get_target(e);
    int32_t value = lv_slider_get_value(slider);
    
    set_brightness(value);
}


