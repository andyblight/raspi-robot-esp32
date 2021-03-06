/*
Useful information for the GPIO functions was found here:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html
The LED PWM controller information was inspired by:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html
https://github.com/espressif/esp-idf/blob/master/examples/peripherals/ledc/main/ledc_example_main.c
*/

#include "raspi_robot_driver.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "raspi_robot_adc.h"
#include "raspi_robot_encoders.h"
#include "raspi_robot_leds.h"
#include "raspi_robot_motors.h"
#include "raspi_robot_servo.h"
#include "raspi_robot_sonar.h"
#include "raspi_robot_switches.h"

// GPIO pins available according to ESP datasheet.
// input: 0-19, 21-23, 25-27, 32-39
// output: 0-19, 21-23, 25-27, 32-33
// Only ADC1 can be used with Wi-FI enabled, pins 32-39.
//
// Available on board:
// On board: 2, 4, 5,12,13,14,15,18,19,21,22,23,25,26,27,32,33,34,35
// Used:     L    Se Ml Ml Ml    Se El Er L  L  Mr Mr Mr So So    B
//
// GPIO pins used.
// LEDS: 2,22,23 Note: 2 is for the on board blue LED.
// SWITCHES: 18,19
// MOTORS: 12,13,14,25,26,27
// SONAR:
// ENCODERS: 19, 21
// SERVOS: 4,5
// BATTERY: 15

// Switch GPIOs - NOT used but set ot valid values.
#define GPIO_SWITCH_SW1 38
#define GPIO_SWITCH_SW2 39
// Sonar GPIOs
#define GPIO_SONAR_OUT 32
#define GPIO_SONAR_IN 33
// Servo GPIOs
#define SERVO_X 5
#define SERVO_Y 18
// Encoder GPIOs
#define ENCODER_LEFT 19
#define ENCODER_RIGHT 21
// Battery voltage pin
#define BATTERY_VOLTAGE_PIN 35
#define BATTERY_VOLTAGE_RANGE_MV 11000

// Handy constants.
#define PI (3.1415f)
#define MS_PER_S (1000)
#define NS_PER_MS (1000 * 1000)
#define NS_PER_S (1000 * 1000 * 1000)

// Information about the robot.
// Wheel diameter.
#define WHEEL_DIAMETER_MM (68)
#define WHEEL_CIRCUMFERENCE_M (PI * WHEEL_DIAMETER_MM / 1000.0f)
// Distance between centres of wheels.
#define WHEEL_BASE_MM (160)
#define WHEEL_BASE_M (WHEEL_BASE_MM / 1000)

// Encoder ticks per revolution.
#define ENCODER_TICKS_PER_REV (12)
#define METRES_PER_ENCODER_TICK (WHEEL_CIRCUMFERENCE_M / ENCODER_TICKS_PER_REV)

// Motor constants.
// FIXME(AJB) Validate these.  Guesses for now.
#define MAXIMUM_SPEED_M_S (0.50)
#define MINIMUM_SPEED_M_S (0.10)
// The motors do not move the robot when less than this duty value.
// FIXME(AJB) Remove this when motor controller working.
#define MINIMUM_MOTOR_PERCENT (30)


/**** Private variables & functions ****/

static int8_t m_motor_percent_left;
static int8_t m_motor_percent_right;
static int16_t m_encoder_count_left;
static int16_t m_encoder_count_right;

static void update_encoder_counts(void) {
  uint16_t count_left = 0;
  uint16_t count_right = 0;
  // Get raw encoder counts.
  // These have no concept of direction.
  encoders_get(ENCODER_LEFT, &count_left);
  encoders_get(ENCODER_RIGHT, &count_right);
  // The motors are driven in opposite directions to move in the same direction,
  // i.e. Forward - left +ve, right -ve. Reverse - left -ve, right +ve.
  // Normalise the counts using motor directions.
  if (m_motor_percent_left > 0) {
    m_encoder_count_left += count_left;
  } else {
    m_encoder_count_left -= count_left;
  }
  if (m_motor_percent_right > 0) {
    m_encoder_count_right -= count_right;
  } else {
    m_encoder_count_right += count_right;
  }
}

/**** API functions ****/

void raspi_robot_init(void) {
  adc_init(BATTERY_VOLTAGE_PIN, BATTERY_VOLTAGE_RANGE_MV);
  leds_init();
  motors_init();
  encoders_init(ENCODER_LEFT);
  encoders_init(ENCODER_RIGHT);
  servo_init(SERVO_X);
  servo_init(SERVO_Y);
  sonar_init(GPIO_SONAR_OUT, GPIO_SONAR_IN);
  switches_init(GPIO_SWITCH_SW1);
  switches_init(GPIO_SWITCH_SW2);
}

void raspi_robot_term(void) { sonar_term(); }

void raspi_robot_tick(void) {
  adc_tick();
  leds_tick();
  motors_tick();
  sonar_tick();
  update_encoder_counts();
}

void raspi_robot_get_status(status_t *status) {
  // Switches.
  status->switch1 = switches_get(GPIO_SWITCH_SW1);
  status->switch2 = switches_get(GPIO_SWITCH_SW2);
  // Sonar.
  status->sonar_mm = sonar_get();
}

void raspi_robot_set_led(const raspi_robot_led_t led_value,
                         const raspi_robot_led_flash_t flash_rate) {
  leds_set(led_value, flash_rate);
}

void raspi_robot_motors_drive(int8_t percent_left, int8_t percent_right,
                              uint16_t ticks) {
  motors_drive(percent_left, percent_right, ticks);
  m_motor_percent_left = percent_left;
  m_motor_percent_right = percent_right;
  // Update the encoder counts in case there is a direction change.
  update_encoder_counts();
}

void raspi_robot_motors_set_velocities(float linear_m_s, float angular_r_s,
                                       uint16_t ticks) {
  // Convert float to percent within motor limits.
  int8_t left_percent = 0;
  int8_t right_percent = 0;
  // Do most calculations using absolute velocity.
  float abs_velocity_m_s = 0;
  if (linear_m_s < 0.0) {
    abs_velocity_m_s = -linear_m_s;
  } else {
    abs_velocity_m_s = linear_m_s;
  }
  // Clamp the maximum speed.
  if (abs_velocity_m_s > MAXIMUM_SPEED_M_S) {
    abs_velocity_m_s = MAXIMUM_SPEED_M_S;
  }
  // Convert the forward value to a motor percent.
  if (abs_velocity_m_s > MINIMUM_SPEED_M_S) {
    int8_t forward_percent =
        (int8_t)((float)(abs_velocity_m_s / MAXIMUM_SPEED_M_S) * 100);
    // Clamp minimum percent.
    if (forward_percent < MINIMUM_MOTOR_PERCENT) {
      forward_percent = MINIMUM_MOTOR_PERCENT;
    }
    // Add sign back in and convert to left and right values.
    // To go forward, one motor is +ve and the other is -ve.
    if (linear_m_s > 0.0) {
      // Forward.
      left_percent = forward_percent;
      right_percent = -forward_percent;
    } else {
      // Reverse.
      left_percent = -forward_percent;
      right_percent = forward_percent;
    }
  }  // Any value less than minimum speed is ignored.
  raspi_robot_motors_drive(left_percent, right_percent, MOTOR_TICKS);
}

void raspi_robot_motors_calibrate() {
  // Comment to keep formating sensible.
}

uint32_t raspi_robot_get_battery_voltage() {
  // Just return the result.
  return adc_battery_voltage();
}

int16_t raspi_robot_get_hall_effect() {
  // Just return the result.
  return adc_hall_effect_sensor();
}

void raspi_robot_servo_set(int16_t x, int16_t y) {
  // Invert the sense of the values to match function specification and to match
  // pitch and yaw values.
  servo_set(SERVO_X, -x);
  servo_set(SERVO_Y, -y);
}

void raspi_robot_get_encoders(int16_t *left, int16_t *right) {
  *left = m_encoder_count_left;
  *right = m_encoder_count_right;
  // Reset count so delta is returned next time.
  m_encoder_count_left = 0;
  m_encoder_count_right = 0;
}
