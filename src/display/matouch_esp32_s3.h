#ifndef MATOUCH_ESP32_S3_H
#define MATOUCH_ESP32_S3_H

#include <stdio.h>

#include "esp_lcd_touch_gt911.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"

// Function declarations
void init_display(void);

void init_backlight(void);

void set_backlight_brightness(uint8_t brightness);

void init_touch(esp_lcd_touch_handle_t *touch_handle);

void init_lcd(esp_lcd_panel_handle_t *panel_handle);

void init_lvgl(esp_lcd_panel_handle_t panel_handle, esp_lcd_touch_handle_t touch_handle);



#endif /* MATOUCH_ESP32_S3_H */