#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"

#include "display/matouch_esp32_s3.h"
#include "ui/ui.h"


static const char* TAG = "MAIN";

extern SemaphoreHandle_t lvgl_mux;

void app_main(void)
{

    init_display();
    set_backlight_brightness(2);

    ESP_LOGI(TAG, "Start LVGL ui");

    xSemaphoreTakeRecursive(lvgl_mux, portMAX_DELAY);
    ui_init();
    xSemaphoreGiveRecursive(lvgl_mux);


}