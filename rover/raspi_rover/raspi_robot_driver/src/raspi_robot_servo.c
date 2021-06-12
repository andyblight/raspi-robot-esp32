/*
This document is helpful:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html
Based on example code from:
https://github.com/espressif/esp-idf/,
examples/peripherals/mcpwm/mcpwm_servo_control/mcpwm_servo_control_example.c
with additions to handle two servos.
*/

#include "raspi_robot_servo.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "driver/mcpwm.h"
#include "esp_log.h"

// Although 3 servos can be supported, the code only works for 2.
#define MAX_SERVOS (2)

typedef struct servo_struct {
  mcpwm_unit_t unit_num;
  mcpwm_timer_t timer_num;
  int gpio_num;
  float duty_cycle;
} servo_t;

// ESP32 timer values.
// We use motor unit 1 that can control a maximum of 3 motors/servos.
static const mcpwm_unit_t k_motor_unit_num = MCPWM_UNIT_1;
// Use out of range value, -1 as int is used for this parameter.
static const int k_spare_servo_value = -1;

// You can get these values from the servo datasheet.
// These values are correct for the SG90.
static const uint16_t k_servo_range_degrees = 180;
static const uint16_t k_minimum_pulse_width_us = 1000;
static const uint16_t k_maximum_pulse_width_us = 2000;
static const uint16_t k_pulse_width_range_us =
    k_maximum_pulse_width_us - k_minimum_pulse_width_us;
// Period is 20ms so frequency is 50Hz.
static const uint16_t k_frequency_hz = 50;
// The servo has a range of 0 to 180 degrees with the centre at 90 degrees.
// These values are used for conversion between the API and servo and for range
// checking.
static const int16_t k_angle_min = -90;
static const int16_t k_angle_max = 90;
// Number of degrees from minimum to centre.
static const int16_t k_angle_offset = 90;

// Logging name.
static const char *TAG = "raspi_robot_servo";

// Servo instances.
static servo_t m_servos[MAX_SERVOS];
static int m_initialised = 0;

/**** Private functions ****/

static void servo_array_init(void) {
  for (int servo_num = 0; servo_num < MAX_SERVOS; ++servo_num) {
    servo_t *servo = &(m_servos[servo_num]);
    servo->unit_num = k_motor_unit_num;
    // Hard coded for 2 servos.
    if (servo_num == 0) {
      servo->timer_num = MCPWM_TIMER_0;
    } else {
      servo->timer_num = MCPWM_TIMER_1;
    }
    servo->gpio_num = k_spare_servo_value;
    servo->duty_cycle = 0.0f;
  }
}

static void configure_servo_unit(void) {
  mcpwm_config_t pwm_config;
  pwm_config.frequency = k_frequency_hz;
  pwm_config.cmpr_a = 0;
  pwm_config.cmpr_b = 0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  // Hard coded for 2 servos.
  mcpwm_init(k_motor_unit_num, MCPWM_TIMER_0, &pwm_config);
  mcpwm_init(k_motor_unit_num, MCPWM_TIMER_1, &pwm_config);
}

/** Allocate a servo for the given gpio_num.
 * @param gpio_num The GPIO number to look for.
 * @return Pointer to the servo.  NULL if no spare slots.
 */
static servo_t *allocate_servo(uint8_t gpio_num) {
  ESP_LOGI(TAG, "Allocating servo for GPIO %d.", gpio_num);
  servo_t *servo = NULL;
  for (int servo_num = 0; servo_num < MAX_SERVOS; ++servo_num) {
    ESP_LOGI(TAG, "Testing servo num %d, value %d.", servo_num,
             m_servos[servo_num].gpio_num);
    // Allocate if spare servo found.
    if (m_servos[servo_num].gpio_num == k_spare_servo_value) {
      servo = &(m_servos[servo_num]);
      servo->gpio_num = (int)gpio_num;
      break;
    }
  }
  ESP_LOGI(TAG, "Allocated servo at %p.", servo);
  return servo;
}

/** Get pointer to servo with gpio_num.
 * @param gpio_num The GPIO number to look for.
 * @return Pointer to the servo.  NULL if not found.
 */
static servo_t *get_servo(uint8_t gpio_num) {
  ESP_LOGI(TAG, "Looking for servo on GPIO %d.", gpio_num);
  servo_t *servo = NULL;
  for (int servo_num = 0; servo_num < MAX_SERVOS; ++servo_num) {
    if (m_servos[servo_num].gpio_num == gpio_num) {
      servo = &(m_servos[servo_num]);
      break;
    }
  }
  ESP_LOGI(TAG, "Servo address found %p.", servo);
  return servo;
}

/**** API functions ****/

void servo_init(uint8_t gpio_num) {
  // Do first time initialisation.
  if (m_initialised == 0) {
    servo_array_init();
    configure_servo_unit();
    m_initialised = 1;
  }
  // Allocate servo to this GPIO
  servo_t *servo = allocate_servo(gpio_num);
  if (servo) {
    ESP_LOGI(TAG, "Servo using GPIO %d.", gpio_num);
  } else {
    ESP_LOGE(TAG, "ERROR: All servos used.");
  }
}

int16_t servo_set(uint8_t gpio_num, int16_t angle) {
  servo_t *servo = get_servo(gpio_num);
  if (servo) {
    // Clamp range of given angle.
    if (angle > k_angle_max) {
      angle = k_angle_max;
    }
    if (angle < k_angle_min) {
      angle = k_angle_min;
    }
    // Convert +90 to -90 angle to 0 to 180.
    uint16_t single_angle = angle + k_angle_offset;
    // Convert to duty in microseconds.
    uint32_t duty_us = k_minimum_pulse_width_us;
    duty_us += k_pulse_width_range_us * single_angle / k_servo_range_degrees;
    // Set duty value.
    mcpwm_set_duty_in_us(servo->unit_num, servo->timer_num, MCPWM_OPR_A,
                         duty_us);
    ESP_LOGI(TAG, "Servo set GPIO %d to %d degrees using %d duty in us.",
             gpio_num, angle, duty_us);
  } else {
    ESP_LOGE(TAG, "ERROR: Nor servo found for GPIO %d", gpio_num);
  }
  return angle;
}
