#ifndef RASPI_ROBOT_LED_PWM_H
#define RASPI_ROBOT_LED_PWM_H

#include <stdint.h>

typedef struct led_pwm_handle_t * led_pwm_handle_p;

/**
 * @brief Initialise the ESP32 LED PWM controller.
 * @param gpio The GPIO pin to use.
 */
void led_pwm_gpio_init(uint8_t gpio_pin);

/**
 * @brief Return the handle for the given GPIO pin.
 * @return Handle for the given GPIO pin or NULL if not found.
 */
led_pwm_handle_p led_pwm_get_handle(uint8_t gpio_pin);

/**
 * @brief Set the duty cycle of the PWM controller for the given GPIO pin.
 * @param handle The handle to use.
 * @param duty Range 0 (full off) to 255 (full on).
 */
void led_pwm_set_duty(led_pwm_handle_p handle, uint8_t duty);

#endif  // RASPI_ROBOT_LED_PWM_H
