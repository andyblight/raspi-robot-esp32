/*
Useful information for the GPIO functions was found here:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html
*/

#include "raspi_robot_switches.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "esp_log.h"

// Logging name.
static const char* TAG = "raspi_robot_switches";

#if USE_INTERRUPTS
// This worked once but is now broken.

// General defines.
#define ESP_INTR_FLAG_DEFAULT 0

// Switch state
#define GPIO_SWITCH_SW1 18
#define GPIO_SWITCH_SW2 19
// FIXME(AJB): Using variables like this is hacky.
// The last read state of the switches.  0 is low, 1 is high.
static int switch_state_sw1 = 0;
static int switch_state_sw2 = 0;

// Interrupt handler to update the switch state values.
// This function is the only thing that updates the switch states.
static void IRAM_ATTR gpio_isr_handler(void* arg) {
  uint32_t gpio_num = (uint32_t)arg;
  switch (gpio_num) {
    case GPIO_SWITCH_SW1:
      switch_state_sw1 = gpio_get_level(gpio_num);
      break;
    case GPIO_SWITCH_SW2:
      switch_state_sw2 = gpio_get_level(gpio_num);
      break;
    default:
      break;
  }
}

void init(void) {
  gpio_config_t io_conf = {
      // Bit mask for the switches.
      .pin_bit_mask = BIT64(GPIO_SWITCH_SW1) | BIT64(GPIO_SWITCH_SW2),
      // Set as input mode
      .mode = GPIO_MODE_INPUT,
      // Disable pull-down mode
      .pull_down_en = 0,
      // Enable pull-up mode
      .pull_up_en = 1,
      // Interrupt on any edge.
      .intr_type = GPIO_INTR_ANYEDGE};
  // Configure GPIOs with the given settings.
  ESP_ERROR_CHECK(gpio_config(&io_conf));

  // Set up the ISR handlers.
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(GPIO_SWITCH_SW1, gpio_isr_handler,
                       (void*)GPIO_SWITCH_SW1);
  gpio_isr_handler_add(GPIO_SWITCH_SW2, gpio_isr_handler,
                       (void*)GPIO_SWITCH_SW2);
  ESP_LOGI(TAG, "Switches initialised");
}

bool validate(uint8_t gpio_num) {
  bool result = false;
  switch (gpio_num) {
    case GPIO_SWITCH_SW1:
      // Drop through
    case GPIO_SWITCH_SW2:
      result = true;
      break;
  }
  return result;
}

/**
 * @brief Get the last state of the given GPIO.
 *
 * @param gpio_num The GPIO to use.
 * @return true The GPIO pin is high.
 * @return false The GPIO pin is low.
 */
bool switches_get(uint8_t gpio_num) {
  bool result = false;
  bool valid = validate(gpio_num);
  if (valid) {
    if (gpio_num == GPIO_SWITCH_SW1) {
      result = (switch_state_sw1 == 1) ? true : false;
    }
    if (gpio_num == GPIO_SWITCH_SW2) {
      result = (switch_state_sw2 == 1) ? true : false;
    }
  }
  return result;
}

#else
// Poll values

void switches_init(uint8_t gpio_num) {
  gpio_config_t io_conf = {// Bit mask for the switches.
                           .pin_bit_mask = BIT64(gpio_num),
                           // Set as input mode
                           .mode = GPIO_MODE_INPUT,
                           // Disable pull-down mode
                           .pull_down_en = 0,
                           // Enable pull-up mode
                           .pull_up_en = 1,
                           // Interrupt on any edge.
                           .intr_type = GPIO_INTR_ANYEDGE};
  // Configure GPIOs with the given settings.
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  ESP_LOGI(TAG, "Switch %d initialised", gpio_num);
}

bool switches_get(uint8_t gpio_num) {
  bool result = false;
  int switch_state = gpio_get_level(gpio_num);
  result = (switch_state == 1) ? true : false;
  return result;
}

#endif
