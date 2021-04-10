#ifndef RASPI_ROBOT_LEDS_H
#define RASPI_ROBOT_LEDS_H

#include <stdint.h>

#include "raspi_robot_driver.h"

/**
 * @brief Initialises the RasPiRobot driver.
 */
void leds_init();

/**
 * @brief Call this regularly to keep 'things' happening.
 * @note Typical call rate is 10 per second.
 */
void leds_tick(void);

/**
 * @brief Controls the given LED.
 *
 * @param led_in The LED to control.
 * @param flash_rate The flash rate to set.
 */
void leds_set(const raspi_robot_led_t led_in,
              const raspi_robot_led_flash_t flash_rate);

#endif  // RASPI_ROBOT_LEDS_H
