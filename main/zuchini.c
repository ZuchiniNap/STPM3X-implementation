#include <stdio.h>
#include "STPM3X.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "STPM3X_APP";

// GPIO Pins
#define RESET_PIN GPIO_NUM_32
#define CS_PIN GPIO_NUM_5
#define SYN_PIN GPIO_NUM_14

STPM3X stpm; // STPM3X Device Object

void app_main(void) {


    ESP_LOGI(TAG, "Starting STPM3X Initialization...");

    // Initialize STPM3X and check if successful
    if (!STPM3X_init(&stpm, RESET_PIN, CS_PIN, SYN_PIN)) {
        ESP_LOGE(TAG, "STPM3X Initialization Failed!");
        return;
    }

    ESP_LOGI(TAG, "STPM3X Successfully Initialized!");

    ESP_LOGI(TAG, "STPM3X Initialization test 3 Completed!");

    // Set calibration values (Modify as needed)
    // STPM3X_setCalibration(&stpm, 1.0, 1.0);

    while (1) {
        // Read RMS current from channel 1 (you can change to channel 2 if needed)
        float rms_current = STPM3X_readRMSCurrent(&stpm, 1);

        if (rms_current >= 0) {
            ESP_LOGI(TAG, "RMS Current on Channel 1: %.2f A", rms_current);
        } else {
            ESP_LOGE(TAG, "Failed to read RMS Current from Channel 1");
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second before next reading

    }
}
