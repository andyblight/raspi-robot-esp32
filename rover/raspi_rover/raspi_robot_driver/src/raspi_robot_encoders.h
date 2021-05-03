#ifndef RASPI_ROBOT_ENCODERS_H
#define RASPI_ROBOT_ENCODERS_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialise the given GPIO pin for use by the encoders.
 * @param gpio_num The GPIO to use.
 */
void encoders_init(uint8_t gpio_num);

/**
 * @brief Get counts of encoder pulses since the last call.
 * @param gpio_num The GPIO to use.
 * @param count Count of encoder pulses since last call.
 * @note This call resets the count of encoder pulses to 0.
 */
void encoders_get(uint8_t gpio_num, uint16_t *count);

#endif  // RASPI_ROBOT_ENCODERS_H
