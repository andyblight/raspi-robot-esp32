/*
Useful information for the GPIO functions was found here:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html
*/

#include "raspi_robot_servo.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "esp_log.h"

#define MAX_SERVOS (2)

static const int16_t angle_min = -180;
static const int16_t angle_max = 180;

// Logging name.
static const char* TAG = "raspi_robot_servo";

/**** API functions ****/

void servo_init(uint8_t gpio_num) {
  ESP_LOGI(TAG, "Servo using GPIO %d.", gpio_num);
  led_pwm_gpio_init(gpio_pin);
}

int16_t servo_set(uint8_t gpio_num, int16_t angle) {
  uint8_t duty = todo;
  led_pwm_handle_t *handle = get_handle(gpio_num);
  led_pwm_set_duty(handle, duty);
  ESP_LOGI(TAG, "Servo set GPIO %d to %d degrees.", gpio_num, angle);
  return angle;
}
