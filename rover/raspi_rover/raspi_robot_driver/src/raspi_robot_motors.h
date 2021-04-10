#ifndef RASPI_ROBOT_MOTORS_H
#define RASPI_ROBOT_MOTORS_H

#include <stdint.h>

/**
 * @brief Initialises the RasPiRobot motor driver.
 */
void motors_init();

/**
 * @brief Stop any running motors that have completed the requested duration.
 */
void motors_tick(void);

/**
 * @brief Drive the selected motors at the given speed for the given time.
 * @param speed_left Percentage of maximum speed. -ve reverse, +ve forward. 0 is
 * stop, 100 is maximum.
 * @param speed_right Percentage of maximum speed. -ve reverse, +ve forward. 0
 * is stop, 100 is maximum.
 * @param ticks Duration to drive the motors in ticks.
 */
void motors_drive(int8_t speed_left, int8_t speed_right, uint16_t ticks);

#endif  // RASPI_ROBOT_MOTORS_H
