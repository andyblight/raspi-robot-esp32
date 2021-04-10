/*
Useful information for the GPIO functions was found here:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html
*/

#include "raspi_robot_leds.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "esp_log.h"

// Logging name.
static const char *TAG = "raspi_robot_leds";

// General defines.
#define ESP_INTR_FLAG_DEFAULT 0

// LED driver
// The ESP blue LED is on GPIO2.
#define GPIO_ESP_BLUE_LED 2
// RasPiRobot LED1 is on GPIO22.
#define GPIO_RASPI_ROBOT_LED1 22
// RasPiRobot LED2 is on GPIO22.
#define GPIO_RASPI_ROBOT_LED2 23

// The LEDs to configure.
#define MAX_LEDS 3
static const int led_gpios[MAX_LEDS] = {
    GPIO_ESP_BLUE_LED, GPIO_RASPI_ROBOT_LED1, GPIO_RASPI_ROBOT_LED2};

// Values for a single LED.
typedef struct {
  int tick_count;
  int gpio;
  int interval_ticks;
  int level;
} led_t;

// Array of all LEDs.
static led_t leds[MAX_LEDS];

void configure_led_gpios(int gpio_bits) {
  gpio_config_t io_conf = {// Bit mask.
                           .pin_bit_mask = gpio_bits,
                           // disable interrupt
                           .intr_type = GPIO_INTR_DISABLE,
                           // set as output mode
                           .mode = GPIO_MODE_OUTPUT,
                           // disable pull-down mode
                           .pull_down_en = 0,
                           // disable pull-up mode
                           .pull_up_en = 0};
  // Configure GPIOs.
  ESP_ERROR_CHECK(gpio_config(&io_conf));
}

/**** API FUNCTIONS ****/

void leds_init(void) {
  int gpio_bits = 0;
  for (int i = 0; i < MAX_LEDS; ++i) {
    int gpio = led_gpios[i];
    gpio_bits |= BIT64(gpio);
    led_t *led = &(leds[i]);
    led->tick_count = 0;
    led->gpio = gpio;
    led->interval_ticks = 0;
    led->level = 0;
  }
  configure_led_gpios(gpio_bits);
  ESP_LOGI(TAG, "LEDs initialised");
}

void leds_tick(void) {
  for (int i = 0; i < MAX_LEDS; ++i) {
    led_t *led = &(leds[i]);
    if (led->interval_ticks == 0) {
      // Should be off so make sure it is.
      gpio_set_level(led->gpio, 0);
    } else if (led->interval_ticks == -1) {
      // Should be on so make sure it is.
      gpio_set_level(led->gpio, 1);
    } else {
      // Should be flashing so update as needed.
      if (led->tick_count <= 0) {
        if (led->level == 0) {
          led->level = 1;
        } else {
          led->level = 0;
        }
        gpio_set_level(led->gpio, led->level);
        // ESP_LOGI(TAG, "LED %d, %d", led->gpio, led->level);
        led->tick_count = led->interval_ticks;
      }
      --led->tick_count;
    }
  }
}

void leds_set(const raspi_robot_led_t led_value,
              const raspi_robot_led_flash_t flash_rate) {
  if (led_value >= RASPI_ROBOT_LED_ESP_BLUE && led_value <= RASPI_ROBOT_LED_2) {
    led_t *led = &(leds[led_value]);
    led->tick_count = 0;
    led->level = 0;
    switch (flash_rate) {
      case RASPI_ROBOT_LED_FLASH_OFF:
        led->interval_ticks = 0;
        break;
      case RASPI_ROBOT_LED_FLASH_ON:
        led->interval_ticks = -1;
        break;
      case RASPI_ROBOT_LED_FLASH_1HZ:
        led->interval_ticks = 10;
        break;
      case RASPI_ROBOT_LED_FLASH_2HZ:
        led->interval_ticks = 5;
        break;
      case RASPI_ROBOT_LED_FLASH_5HZ:
        led->interval_ticks = 2;
        break;
      case RASPI_ROBOT_LED_FLASH_10HZ:
        led->interval_ticks = 1;
        break;
      default:
        ESP_LOGE(TAG, "Invalid flash rate %d", flash_rate);
        break;
    }
    ESP_LOGI(TAG, "Set LED %d, %d, %d", led_value, flash_rate,
             led->interval_ticks);
  } else {
    ESP_LOGE(TAG, "Invalid LED value %d", led_value);
  }
}
