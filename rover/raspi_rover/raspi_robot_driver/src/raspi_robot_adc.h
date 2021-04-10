#ifndef RASPI_ROBOT_ADC_H
#define RASPI_ROBOT_ADC_H

#include <stdint.h>

/**
 * @brief Initialises the battery ADC pin and the hall effect sensor.
 * @param battery_pin The GPIO pin to measure battery voltage.
 * @param battery_range_mv Desired battery voltage range in mv.  Maximum value
 * is 16777215mv (although at this voltage sparks will literally fly!)
 * To read the battery voltage over in the range 0 to 10V, set
 * battery_range_mv to 10000.
 */
void adc_init(uint8_t battery_pin, uint32_t battery_range_mv);

/**
 * @brief Call this regularly to update the battery voltage.
 */
void adc_tick(void);

/**
 * @brief Read the battery voltage.
 * @return Value in mV.
 */
uint32_t adc_battery_voltage();

/**
 * @brief Reads the hall effect sensor.
 */
int16_t adc_hall_effect_sensor();

#endif  // RASPI_ROBOT_ADC_H
