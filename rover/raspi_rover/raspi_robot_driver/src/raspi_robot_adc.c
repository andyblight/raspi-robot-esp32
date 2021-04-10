/*
Useful information for the ADC functions was found here:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html
*/

#include "raspi_robot_adc.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include "esp_log.h"

// Logging name.
static const char *TAG = "raspi_robot_adc";
// Battery pin values.
static adc1_channel_t battery_channel = ADC1_CHANNEL_MAX;
static uint32_t battery_channel_range_mv = 0;
static uint16_t battery_rolling_average = 0;
static const uint16_t num_samples = 8;

adc1_channel_t pin_to_channel(uint8_t pin) {
  adc1_channel_t channel = ADC1_CHANNEL_MAX;
  switch (pin) {
    case 36:
      // Used by hall effect sensor.
      break;
    case 37:
      channel = ADC1_CHANNEL_1;
      break;
    case 38:
      channel = ADC1_CHANNEL_2;
      break;
    case 39:
      // Used by hall effect sensor.
      break;
    case 32:
      channel = ADC1_CHANNEL_4;
      break;
    case 33:
      channel = ADC1_CHANNEL_5;
      break;
    case 34:
      channel = ADC1_CHANNEL_6;
      break;
    case 35:
      channel = ADC1_CHANNEL_7;
      break;
    default:
      ESP_LOGE(TAG, "Pin must be in range 32 to 39.  Given %d", pin);
      break;
  }
  return channel;
}

void adc_init(uint8_t battery_pin, uint32_t battery_range_mv) {
  // Configure ADC1 for use with hall effect sensor.
  adc1_config_width(ADC_WIDTH_BIT_12);
  // Configure battery ADC channel.
  battery_channel = pin_to_channel(battery_pin);
  if (battery_channel != ADC1_CHANNEL_MAX) {
    battery_channel_range_mv = battery_range_mv;
    adc1_config_channel_atten(battery_channel, ADC_ATTEN_DB_0);
    gpio_num_t gpio_num = 0;
    esp_err_t result =  adc1_pad_get_io_num(battery_channel, &gpio_num);
    if (result == ESP_OK) {
      ESP_LOGI(TAG, "ADC using GPIO %d for battery voltage.", gpio_num);
    } else {
      ESP_LOGE(TAG, "ADC battery GPIO not setup.");
    }
  }
}

void adc_tick(void) {
  // Calling the ADC 10 time a second causes lots of false values.
  // Calling the ADC 1 times a second is reliable.
  static int call_count = 0;
  ++call_count;
  if (call_count >= 10) {
    call_count = 0;
    if (battery_channel != ADC1_CHANNEL_MAX) {
      int32_t sample = adc1_get_raw(battery_channel);
      if (sample != -1) {
        battery_rolling_average =
            ((battery_rolling_average * (num_samples - 1)) + sample) /
            num_samples;
      }
      ESP_LOGI(TAG, "sample %d, num_samples %d, average %d", sample,
               num_samples, battery_rolling_average);
    }
  }
}

// This function scales the averaged 12 bit ADC value.
uint32_t adc_battery_voltage() {
  uint32_t scaled_value =
      (battery_rolling_average * battery_channel_range_mv) >> 8;
  ESP_LOGI(TAG, "scaled %d", scaled_value);
  return scaled_value;
}

int16_t adc_hall_effect_sensor() {
  // This is really simple!
  return hall_sensor_read();
}
