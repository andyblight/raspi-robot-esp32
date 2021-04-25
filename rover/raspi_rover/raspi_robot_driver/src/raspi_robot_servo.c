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
#include "raspi_robot_led_pwm.h"

#define MAX_SERVOS (2)

// VValues for the servo
static const int16_t k_angle_min = -90;
static const int16_t k_angle_max = 90;
// Number of degrees from minimum to centre.
static const int16_t k_angle_offset = 90;

// Logging name.
static const char* TAG = "raspi_robot_servo";

/**** API functions ****/

void servo_init(uint8_t gpio_num) {
  ESP_LOGI(TAG, "Servo using GPIO %d.", gpio_num);
  led_pwm_gpio_init(gpio_num);
}

int16_t servo_set(uint8_t gpio_num, int16_t angle) {
  // Clamp angle.
  if (angle > k_angle_max) {
    angle = k_angle_max;
  }
  if (angle < k_angle_min) {
    angle = k_angle_min;
  }
  // Convert angle to duty value.
  // Angle in range min to max maps on to duty 0 to 255.
  // Convert +90 to -90 angle to 0 to 180.
  uint16_t single_angle = angle + k_angle_offset;
  uint16_t duty = (single_angle * 256) / 180;
    // Set duty.
  led_pwm_handle_p handle = led_pwm_get_handle(gpio_num);
  led_pwm_set_duty(handle, (uint8_t)duty);
  ESP_LOGI(TAG, "Servo set GPIO %d to %d degrees using %d duty.", gpio_num, angle, duty);
  return angle;
}
