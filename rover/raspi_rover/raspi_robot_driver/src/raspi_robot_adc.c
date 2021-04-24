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
#include "raspi_robot_led_pwm.h"

// Logging name.
static const char *TAG = "raspi_robot_adc";
//
#define DEFAULT_VREF (1100)
static esp_adc_cal_characteristics_t *m_adc_chars;
// Battery pin values.
static adc1_channel_t m_battery_channel = ADC1_CHANNEL_MAX;
static uint32_t m_battery_channel_range_mv = 0;
static uint16_t m_battery_rolling_average = 0;
static const uint16_t m_battery_num_samples = 8;

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
  // Configure ADC characteristics.
  m_adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12,
                           DEFAULT_VREF, m_adc_chars);
  // Configure ADC1 for use with hall effect sensor.
  adc1_config_width(ADC_WIDTH_BIT_12);
  // Configure battery ADC channel.
  m_battery_channel = pin_to_channel(battery_pin);
  if (m_battery_channel != ADC1_CHANNEL_MAX) {
    m_battery_channel_range_mv = battery_range_mv;
    adc1_config_channel_atten(m_battery_channel, ADC_ATTEN_DB_0);
    gpio_num_t gpio_num = 0;
    esp_err_t result = adc1_pad_get_io_num(m_battery_channel, &gpio_num);
    if (result == ESP_OK) {
      ESP_LOGI(TAG, "ADC using GPIO %d for battery voltage.", gpio_num);
    } else {
      ESP_LOGE(TAG, "ADC battery GPIO not setup.");
    }
  }
}

void adc_tick(void) {
  // Calling the ADC 10 times a second.
  if (m_battery_channel != ADC1_CHANNEL_MAX) {
    int32_t sample = adc1_get_raw(m_battery_channel);
    if (sample != -1) {
      m_battery_rolling_average =
          ((m_battery_rolling_average * (m_battery_num_samples - 1)) + sample) /
          m_battery_num_samples;
    }
    // ESP_LOGI(TAG, "sample %d, m_battery_num_samples %d, average %d", sample,
    //          m_battery_num_samples, m_battery_rolling_average);
  }
}

// This function scales the averaged 12 bit ADC value to mV.
uint32_t adc_battery_voltage() {
  uint32_t voltage_mv =
      esp_adc_cal_raw_to_voltage(m_battery_rolling_average, m_adc_chars);
  uint32_t scaled_voltage_mv = (voltage_mv * m_battery_channel_range_mv) / 1024;
  ESP_LOGI(TAG, "averaged ADC voltage %dmV, scaled %dmV", voltage_mv,
           scaled_voltage_mv);
  return scaled_voltage_mv;
}

int16_t adc_hall_effect_sensor() {
  // This is really simple!
  int16_t hall_sensor_reading = hall_sensor_read();
  ESP_LOGI(TAG, "hall_sensor_reading %d", hall_sensor_reading);
  return hall_sensor_reading;
}
