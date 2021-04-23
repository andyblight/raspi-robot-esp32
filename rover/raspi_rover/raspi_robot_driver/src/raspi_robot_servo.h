#ifndef RASPI_ROBOT_SERVO_H
#define RASPI_ROBOT_SERVO_H

#include <stdint.h>

/**
 * @brief Initialse the given servo.
 *
 * @param gpio_num The GPIO to use for the servo.
 */
void servo_init(uint8_t gpio_num);

/**
 * @brief Get the last state of the given GPIO.
 *
 * @param gpio_num The GPIO of the servo to set.
 * @param angle The angle to set in whole degrees. 0 is centred, +180 to -180
 * degrees.
 * @return The angle set.  The implementation may limit the range.
 */
int16_t servo_set(uint8_t gpio_num, int16_t angle);

#endif  // RASPI_ROBOT_SERVO_H
