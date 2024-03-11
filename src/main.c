#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"

#include "display/esp32_s3.h"
#include "ui/ui.h"
#include "task/counter_task.h"


static const char* TAG = "MAIN";

extern SemaphoreHandle_t lvgl_mux;


void app_main(void)
{

    init_display();
    set_backlight_brightness(2);

    ESP_LOGI(TAG, "Start LVGL");

    xSemaphoreTakeRecursive(lvgl_mux, portMAX_DELAY);
    ui_init();
    xSemaphoreGiveRecursive(lvgl_mux);

    xTaskCreatePinnedToCore(
      counter_task,   /* Task function. */
      "Counter Task", /* String with name of task. */
      4096,           /* Stack size in bytes. */
      NULL,           /* Parameter passed as input of the task */
      1,              /* Priority of the task. */
      NULL,           /* Task handle. */
      1);             /* Clock task on core 1*/

}