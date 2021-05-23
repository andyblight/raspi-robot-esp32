/*
Useful information for the GPIO functions was found here:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html

This driver uses interrupts to increment the encoder pulse count.
*/

#include "raspi_robot_encoders.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "esp_log.h"

#define MAX_ENCODERS (2)
#define ESP_INTR_FLAG_DEFAULT (0)

// Logging name.
static const char* TAG = "raspi_robot_encoders";

typedef struct {
  uint8_t gpio_num;
  uint16_t pulse_count;
} encoder_t;

static encoder_t encoders[MAX_ENCODERS] = {0};
static int encoder_count = 0;

// Interrupt handler to update the pulse counts.
// FIXME This could be simplified if the GPIOs were fixed and two ISRs were
// used, one for each GPIO pin.  This would remove the need for the loop.
static void IRAM_ATTR gpio_isr_handler(void* arg) {
  uint32_t gpio_num = (uint32_t)arg;
  for (int i = 0; i < MAX_ENCODERS; ++i) {
    if (encoders[i].gpio_num == gpio_num) {
      ++(encoders[i].pulse_count);
      break;
    }
  }
}

/**** API functions ****/

void encoders_init(uint8_t gpio_num) {
  if (encoder_count < MAX_ENCODERS) {
    // Configure GPIOs with the given settings.
    gpio_config_t io_conf = {// Bit mask for the encoder.
                             .pin_bit_mask = BIT64(gpio_num),
                             // Set as input mode
                             .mode = GPIO_MODE_INPUT,
                             // Disable pull-down mode
                             .pull_down_en = 0,
                             // Enable pull-up mode
                             .pull_up_en = 1,
                             // Interrupt on rising edge.
                             .intr_type = GPIO_INTR_POSEDGE};
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    // Set up the ISR handlers.
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // Use the same interrupt handler for both encoders.
    gpio_isr_handler_add(gpio_num, gpio_isr_handler, (void*)(uint32_t)gpio_num);
    // Initialise internal vars.
    encoders[encoder_count].gpio_num = gpio_num;
    encoders[encoder_count].pulse_count = 0;
    ++encoder_count;
    ESP_LOGI(TAG, "Encoder on GPIO %d initialised", gpio_num);
  } else {
    ESP_LOGE(TAG, "ERROR: No spare encoders.");
  }
}

void encoders_get(uint8_t gpio_num, uint16_t* count) {
  for (int i = 0; i < MAX_ENCODERS; ++i) {
    if (encoders[i].gpio_num == gpio_num) {
      // TODO Disable interrupts.
      *count = encoders[i].pulse_count;
      encoders[i].pulse_count = 0;
      // TODO Enable interrupts.
      // ESP_LOGI(TAG, "Encoder GPIO %d, count %d", gpio_num, *count);
      break;
    }
  }
}
