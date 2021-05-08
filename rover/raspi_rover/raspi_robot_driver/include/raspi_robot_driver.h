#ifndef RASPI_ROBOT_DRIVER_H
#define RASPI_ROBOT_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  // The numbered values are used to access an internal array.
  RASPI_ROBOT_LED_ESP_BLUE = 0,
  RASPI_ROBOT_LED_1 = 1,
  RASPI_ROBOT_LED_2 = 2
} raspi_robot_led_t;

/**
 * @brief On, off, flash rates.
 * @note Flash rates are defined in number of ticks for on time. Off time = one
 * time.
 */
typedef enum {
  RASPI_ROBOT_LED_FLASH_OFF = 0,
  RASPI_ROBOT_LED_FLASH_ON,
  RASPI_ROBOT_LED_FLASH_1HZ,
  RASPI_ROBOT_LED_FLASH_2HZ,
  RASPI_ROBOT_LED_FLASH_5HZ,
  RASPI_ROBOT_LED_FLASH_10HZ
} raspi_robot_led_flash_t;

/**
 * @brief Status of switches and sonar.
 */
typedef struct {
  /// The status of the two switch inputs on the RasPiRobot board.
  /// These inputs use pull up resistors so open circuit is true, closed is
  /// false.
  bool switch1;
  bool switch2;
  /// The last value of the sonar in millimetres from the front of the unit.
  /// Maximum range is 4 metres.
  uint16_t sonar_mm;
} status_t;

/**
 * @brief Initialises the RasPiRobot driver.
 */
void raspi_robot_init(void);

/**
 * @brief Terminates any tasks started by the RasPiRobot driver.
 */
void raspi_robot_term(void);

/**
 * @brief Call this regularly to keep 'things' happening.
 * @note Typical call rate is 10 per second.
 */
void raspi_robot_tick(void);

/**
 * @brief Fill the given structure with the latest values of the switches and
 * sonar.
 *
 * @param status Pointer to the structure to fill out.
 */
void raspi_robot_get_status(status_t *status);

/**
 * @brief Controls the given LED.
 *
 * @param led The LED to control.
 * @param flash_rate The flash rate to set.
 */
void raspi_robot_set_led(const raspi_robot_led_t led_in,
                         const raspi_robot_led_flash_t flash_rate);

/**
 * @brief Drive the selected motors at the given speed for the given time.
 * @param speed_left Percentage of maximum speed. -ve reverse, +ve forward. 0 is
 * stop, 100 is maximum.
 * @param speed_right Percentage of maximum speed. -ve reverse, +ve forward. 0
 * is stop, 100 is maximum.
 * @param ticks Duration to drive the motors in ticks.
 */
void raspi_robot_motors_drive(int8_t percent_left, int8_t percent_right,
                              uint16_t ticks);

/**
 * @brief Returns the current battery voltage.
 *
 * @return The battery voltage in milli-volts.
 */
uint32_t raspi_robot_get_battery_voltage();

/**
 * @brief Gets the value of the hall effect sensor.
 *
 * @return int16_t TODO I have no idea what the value means.
 */
int16_t raspi_robot_get_hall_effect();

/**
 * @brief Set the servo position and returns the value actually set.
 * @param x The horizontal angle to set in degrees. -180 to +180, 0 centre.
 * @param y The vertical angle to set in degrees. -180 to +180, 0 horizontal.
 * @note The angle set may be limited by the implementation so check the values
 * that x and y are set to after this call.
 */
void raspi_robot_servo_set(int16_t *x, int16_t *y);

/**
 * @brief Get counts of encoder pulses since the last call.
 * @param left Count of encoder pulses since last call.
 * @param right Count of encoder pulses since last call.
 * @note This call resets the count of encoder pulses to 0.   Positive values
 * are forward.
 */
void raspi_robot_get_encoders(int16_t *left, int16_t *right);

#endif  // RASPI_ROBOT_DRIVER_H
