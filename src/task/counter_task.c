
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "counter_task.h"

#include "../gui/gui.h"


static const char* TAG = "COUNTER";

extern SemaphoreHandle_t lvgl_mux;

void counter_task(void *pvParameter){

  uint32_t counter = 1;

  for (;;) {

    xSemaphoreTakeRecursive(lvgl_mux, portMAX_DELAY);
    disp_counter(counter++);
    xSemaphoreGiveRecursive(lvgl_mux);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}