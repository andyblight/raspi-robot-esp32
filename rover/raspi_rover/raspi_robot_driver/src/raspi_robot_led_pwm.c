/*
Useful information for the GPIO functions was found here:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html
The LED PWM controller information was inspired by:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html
https://github.com/espressif/esp-idf/blob/master/examples/peripherals/ledc/main/ledc_example_main.c
*/

#include "raspi_robot_led_pwm.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

// Logging name.
static const char *TAG = "raspi_robot_led_pwm";

// General defines.
#define ESP_INTR_FLAG_DEFAULT 0
#define TESTING 0

// Use 8 bits as we only control with range 0 to 255.
#define DUTY_RESOLUTION (LEDC_TIMER_8_BIT)
// Using high speed mode as it is easier to control.
#define PWM_SPEED_MODE (LEDC_HIGH_SPEED_MODE)
#define PWM_FREQUENCY_HZ (20000)
// Timer to use.
#define PWM_TIMER (LEDC_TIMER_0)
// Maximum number of channels.
// There are two groups of 8 but to keep it simple we only use one group.
#define PWM_CHANNELS (8)

/// The internal handle type.
typedef struct led_pwm_handle {
  bool used;
  ledc_channel_config_t ledc_channel;
} led_pwm_handle_t;

/// List of handles
static led_pwm_handle_t m_handles[PWM_CHANNELS];

void init_driver() {
  // Prepare and set configuration of the timer that will be used by the LED
  // Controller.
  static bool initialised = false;
  if (!initialised) {
    static ledc_timer_config_t ledc_timer = {
        .duty_resolution = DUTY_RESOLUTION,
        .freq_hz = PWM_FREQUENCY_HZ,
        .speed_mode = PWM_SPEED_MODE,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    // Set configuration of timer0 for high speed channels
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    // Initialise the handles.
    for (int i = 0; i < PWM_CHANNELS; ++i) {
      m_handles[i].used = false;
      // Set channel numbers.
      ledc_channel_t channel = LEDC_CHANNEL_MAX;
      switch (i) {
        case 0:
          channel = LEDC_CHANNEL_0;
          break;
        case 1:
          channel = LEDC_CHANNEL_1;
          break;
        case 2:
          channel = LEDC_CHANNEL_2;
          break;
        case 3:
          channel = LEDC_CHANNEL_3;
          break;
        case 4:
          channel = LEDC_CHANNEL_4;
          break;
        case 5:
          channel = LEDC_CHANNEL_5;
          break;
        case 6:
          channel = LEDC_CHANNEL_6;
          break;
        case 7:
          channel = LEDC_CHANNEL_7;
          break;
      }
      m_handles[i].ledc_channel.channel = channel;
    }
  }
}

void add_channel(uint8_t gpio_pin) {
  led_pwm_handle_t *handle = NULL;
  for (int i = 0; i < PWM_CHANNELS; ++i) {
    if (!m_handles[i].used) {
      // Allocate handle.
      m_handles[i].used = true;
      handle = &m_handles[i];
      // Configure channel.
      ledc_channel_config_t *ledc_channel = &(handle->ledc_channel);
      // 'ledc_channel->channel' is preset.
      ledc_channel->duty = 0;
      ledc_channel->gpio_num = gpio_pin;
      ledc_channel->speed_mode = PWM_SPEED_MODE;
      ledc_channel->hpoint = 0;
      ledc_channel->timer_sel = PWM_TIMER;
      ESP_ERROR_CHECK(ledc_channel_config(ledc_channel));
      // Found and allocated handle so exit loop early.
      break;
    }
  }
  if (!handle) {
    ESP_LOGE(TAG, "%s, No spare handles", __FUNCTION__);
  }
}

/********* API functions *********/

void led_pwm_gpio_init(uint8_t gpio_pin) {
  init_driver();
  add_channel(gpio_pin);
}

led_pwm_handle_p get_handle(uint8_t gpio_num) {
  led_pwm_handle_p handle = NULL;
  for (int i = 0; i < PWM_CHANNELS; ++i) {
    if (m_handles[i].used) {
      if (m_handles[i].ledc_channel.gpio_num == gpio_num) {
        handle = (led_pwm_handle_p)&m_handles[i];
        break;
      }
    }
  }
  return handle;
}

void led_pwm_set_duty(led_pwm_handle_p handle, uint8_t duty) {
  if (handle) {
    led_pwm_handle_t *pwm_handle = (led_pwm_handle_t *)handle;
    ledc_channel_config_t *ledc_channel = &(pwm_handle->ledc_channel);
    ESP_LOGI(TAG, "%s, duty %d ", __FUNCTION__, duty);
    ESP_ERROR_CHECK(
        ledc_set_duty(ledc_channel->speed_mode, ledc_channel->channel, duty));
    ESP_ERROR_CHECK(
        ledc_update_duty(ledc_channel->speed_mode, ledc_channel->channel));
  } else {
    ESP_LOGE(TAG, "%s, Invalid handle", __FUNCTION__);
  }
}
