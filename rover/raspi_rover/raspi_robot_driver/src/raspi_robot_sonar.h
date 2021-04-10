#ifndef RASPI_ROBOT_SONAR_H
#define RASPI_ROBOT_SONAR_H

#include <stdint.h>

/**
 * @brief Initialises the sonar driver.
 * @param output_pin The GPIO pin to use for output.
 * @param input_pin The GPIO pin to use for input.
 */
void sonar_init(uint8_t output_pin, uint8_t input_pin);

/**
 * @brief Terminates the task started by sonar_init().
 */
void sonar_term(void);

/**
 * @brief Call this regularly to keep 'things' happening.
 * @note Typical call rate is 10 per second.
 */
void sonar_tick(void);

/**
 * @brief Returns range of nearest object.
 * @return uint16_t Distance to nearest object in millimetres.  If 0, either
 * the distance is greater than 4 metres or there is a loose connection.
 */
uint16_t sonar_get();

#endif  // RASPI_ROBOT_SONAR_H
