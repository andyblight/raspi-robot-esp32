#ifndef RASPI_ROBOT_SWITCHES_H
#define RASPI_ROBOT_SWITCHES_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialise the given switch.
 *
 * @param gpio_num The GPIO to initialise.
 */
void switches_init(uint8_t gpio_num);

/**
 * @brief Get the last state of the given GPIO.
 *
 * @param gpio_num The GPIO to use.
 * @return true The GPIO pin is high.
 * @return false The GPIO pin is low.
 */
bool switches_get(uint8_t gpio_num);

#endif  // RASPI_ROBOT_SWITCHES_H
