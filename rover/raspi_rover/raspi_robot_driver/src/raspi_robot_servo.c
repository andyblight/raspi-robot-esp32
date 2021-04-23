/*
Useful information for the GPIO functions was found here:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html
*/

#include "raspi_robot_servo.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "esp_log.h"

// Logging name.
static const char* TAG = "raspi_robot_servo";


void servo_init(uint8_t gpio_num) {
  ESP_LOGI(TAG, "Servo using GPIO %d.", gpio_num);
}

int16_t servo_set(uint8_t gpio_num, int16_t angle) {
  ESP_LOGI(TAG, "Servo set GPIO %d to %d degrees.", gpio_num, angle);
  return angle;
}
