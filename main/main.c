/**
 * CD Player Main Controller
 * Power-on self-test for S1L9226X servo controller
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "s1l9226x.h"

static const char *TAG = "CD_PLAYER";

static s1l9226x_ctx_t servo_ctx = {0};

void app_main(void)
{
    ESP_LOGI(TAG, "CD Player Controller Starting...");

    int result = s1l9226x_power_on_self_test(&servo_ctx);

    if (result == ERR_NONE) {
        ESP_LOGI(TAG, "Self-test PASSED - Disc present, ready for playback");
    } else {
        ESP_LOGE(TAG, "Self-test FAILED - Error: %d", result);
    }
}