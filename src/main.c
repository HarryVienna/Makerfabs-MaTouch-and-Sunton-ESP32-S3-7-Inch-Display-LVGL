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

    ESP_LOGI(TAG, "Start LVGL ui");

    xSemaphoreTakeRecursive(lvgl_mux, portMAX_DELAY);
    ui_init();
    xSemaphoreGiveRecursive(lvgl_mux);

    while(1) {
        for (int i = 0; i <= 255; ++i) {
            set_backlight_brightness(i);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        for (int i = 255; i >= 0; --i) {
            set_backlight_brightness(i);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

}