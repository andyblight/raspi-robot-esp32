/*
Useful information for the GPIO functions was found here:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html
The LED PWM controller information was inspired by:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html
https://github.com/espressif/esp-idf/blob/master/examples/peripherals/ledc/main/ledc_example_main.c
*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "esp_log.h"

#include "raspi_robot_driver.h"
#include "raspi_robot_leds.h"
#include "raspi_robot_motors.h"
#include "raspi_robot_sonar.h"
#include "raspi_robot_switches.h"

// GPIO pins available according to ESP datasheet.
// input: 0-19, 21-23, 25-27, 32-39
// output: 0-19, 21-23, 25-27, 32-33
//
// Available on board:
// On board: 2, 4, 5,12,13,14,15,18,19,21,22,23,25,26,27,32,33,34,35
// Used:     L Se Se Ml Ml Ml    Sw Sw    L  L  Mr Mr Mr So So  E  E
//
// GPIO pins used.
// LEDS: 2,22,23 Note: 2 is for the on board blue LED.
// SWITCHES: 18,19
// MOTORS: 12,13,14,25,26,27
// SONAR: 32,33
// ENCODERS: 34,35
// SERVOS: 4,5

// Switch GPIOs
#define GPIO_SWITCH_SW1 18
#define GPIO_SWITCH_SW2 19
// Sonar GPIOs
#define GPIO_SONAR_OUT 32
#define GPIO_SONAR_IN 33


void raspi_robot_init(void)
{
    leds_init();
    switches_init(GPIO_SWITCH_SW1);
    switches_init(GPIO_SWITCH_SW2);
    motors_init();
    sonar_init(GPIO_SONAR_OUT, GPIO_SONAR_IN);
}

void raspi_robot_term(void)
{
    sonar_term();
}

void raspi_robot_tick(void)
{
    leds_tick();
    motors_tick();
    sonar_tick();
}

void raspi_robot_get_status(status_t *status) {
    // Switches.
    status->switch1 = switches_get(GPIO_SWITCH_SW1);
    status->switch2 = switches_get(GPIO_SWITCH_SW2);
    // Sonar.
    status->sonar_mm = sonar_get();
}

void raspi_robot_set_led(const raspi_robot_led_t led_value, const raspi_robot_led_flash_t flash_rate)
{
    leds_set(led_value, flash_rate);
}


void raspi_robot_motors_drive(int8_t speed_left, int8_t speed_right, uint16_t ticks)
{
    motors_drive(speed_left, speed_right, ticks);
}
