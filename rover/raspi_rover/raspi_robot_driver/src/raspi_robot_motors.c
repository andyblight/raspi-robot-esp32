/*
Useful information for the GPIO functions was found here:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html
The LED PWM controller information was inspired by:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html
https://github.com/espressif/esp-idf/blob/master/examples/peripherals/ledc/main/ledc_example_main.c
*/

#include "raspi_robot_motors.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

// Logging name.
static const char *TAG = "raspi_robot_motors";

// General defines.
#define ESP_INTR_FLAG_DEFAULT 0
#define TESTING 0

// Motor controllers
// Each motor needs three GPIOs, two ordinary to control direction and one PWM.
// I have used the notation used on the TB6612 device. A is left, B is right.
#define MOTOR_GPIO_AIN1 (12)  // RPI11
#define MOTOR_GPIO_AIN2 (13)  // RPI7
#define MOTOR_GPIO_PWMA (14)  // RPI18
#define MOTOR_GPIO_BIN1 (25)  // RPI19
#define MOTOR_GPIO_BIN2 (26)  // RPI22
#define MOTOR_GPIO_PWMB (27)  // RPI8
// Other defines for the motor PWMs.
#define MOTOR_COUNT (2)
// Voltages used to set may duty percent value in init.
#define MOTOR_BATTERY_VOLTAGE (9.0)
#define MOTOR_MOTOR_VOLTAGE (6.0)

typedef enum motor_mode_tag {
  MOTOR_MODE_STOP,
  MOTOR_MODE_CW,
  MOTOR_MODE_CCW,
  MOTOR_MODE_SHORT_BRAKE
} motor_mode_t;

// Values for a single motor.
typedef struct {
  motor_mode_t mode;
  uint8_t max_duty_value;
  uint16_t tick_count;
  int8_t speed;
  int gpio_in1;
  int gpio_in2;
  uint8_t gpio_pwm;
} motor_t;

// Motor instances.
#define MOTOR_INDEX_LEFT (0)
#define MOTOR_INDEX_RIGHT (1)
static motor_t motors[MOTOR_COUNT];

void motors_init_gpios() {
  // Configure the 4 GPIOs to control direction.
  gpio_config_t io_conf = {
      // Bit mask.
      .pin_bit_mask = BIT64(MOTOR_GPIO_AIN1) | BIT64(MOTOR_GPIO_AIN2) |
                      BIT64(MOTOR_GPIO_BIN1) | BIT64(MOTOR_GPIO_BIN2),
      // Disable interrupts.
      .intr_type = GPIO_INTR_DISABLE,
      // Output mode
      .mode = GPIO_MODE_OUTPUT,
      // Disable pull-up/down.
      .pull_down_en = 0,
      .pull_up_en = 0};
  // Configure GPIOs.
  ESP_ERROR_CHECK(gpio_config(&io_conf));
}

void motor_set_duty(motor_t *motor, int8_t speed_percent) {
  // Duty is in range 0 to 255 (8 bit range) limited to max_duty_value.
  uint8_t duty = (speed_percent * motor->max_duty_value) / 100;
  ESP_LOGI(TAG, "%s, speed%% %d, duty %d ", __FUNCTION__, speed_percent, duty);
  led_pwm_handle_t *handle = get_handle(motor->gpio_num);
  led_pwm_set_duty(handle, duty);
}

/**
 * @brief Set the given motor to the new mode.
 * @param motor The motor to change.
 * @param new_mode The new mode.
 */
void motor_set_mode(motor_t *motor, motor_mode_t new_mode) {
  // ESP_LOGI(TAG, "%s, %d, %d", __FUNCTION__, motor->gpio_in1, new_mode);
  int in1 = 0;
  int in2 = 0;
  switch (new_mode) {
    case MOTOR_MODE_STOP:
      // Already 00.
      break;
    case MOTOR_MODE_CCW:
      in2 = 1;
      break;
    case MOTOR_MODE_CW:
      in1 = 1;
      break;
    case MOTOR_MODE_SHORT_BRAKE:
      in1 = 1;
      in2 = 1;
      break;
  }
  gpio_set_level(motor->gpio_in1, in1);
  gpio_set_level(motor->gpio_in2, in2);
}

/**
 * @brief Sets the speed, direction and duration of the given motor.
 * @param motor The motor to control.
 * @param speed 0 is stop, +ve is forward (ccw), -ve is reverse (cw), value is
 * speed as percentage of maximum.
 * @param ticks Duration in ticks.
 */
void motor_drive(motor_t *motor, int8_t speed, uint32_t ticks) {
  ESP_LOGI(TAG, "%s called: motor %d, speed %d, ticks %d ", __FUNCTION__,
           motor->gpio_in1, speed, ticks);
  // Set the PWM duty cycle, limiting to MOTOR_max_duty_value.
  uint32_t speed_percent = abs(speed);
  if (speed_percent > 100) {
    speed_percent = 100;
  }
  motor_set_duty(motor, speed_percent);
  // Set the new mode.
  motor_mode_t new_mode = MOTOR_MODE_STOP;
  if (speed > 0) {
    new_mode = MOTOR_MODE_CCW;
  }
  if (speed < 0) {
    new_mode = MOTOR_MODE_CW;
  }
  motor_set_mode(motor, new_mode);
  // Now that the hardware has been updated, update the software copies.
  motor->tick_count = ticks;
  motor->speed = speed;
}

#if TESTING
// Test all combinations of motor_mode using both motors.
void motor_test_motor_set_mode(int state) {
  // Cycle through all four combinations on both motors.
  int mode_state = state % 4;
  motor_mode_t new_mode = MOTOR_MODE_STOP;
  switch (mode_state) {
    case 0:
      new_mode = MOTOR_MODE_SHORT_BRAKE;
      break;
    case 1:
      new_mode = MOTOR_MODE_CCW;
      break;
    case 2:
      new_mode = MOTOR_MODE_CW;
      break;
    case 3:
      // Drop through.
    default:
      // Use default (stop).
      break;
  }
  // Use motor 0 first, then 1 and repeat.
  int motor_select = (state % 8) / 4;
  motor_set_mode(&(motors[motor_select]), new_mode);
}

/**
 * @brief Test the calculations and the PWM output.
 * @note The motor_mode function has been tested elsewhere.
 * @param state The state value.
 */
void motor_test_motor_drive(int state) {
  uint32_t duration_ms = 1500;
  int8_t speed = 0;
  const int max_states = 7;
  int mode_state = state % max_states;
  switch (mode_state) {
    case 0:
      // Forward, 50%
      speed = +50;
      break;
    case 1:
      // Forward, 100%
      speed = +100;
      break;
    case 2:
      // Forward, 120%.  Should limit.
      speed = +120;
      break;
    case 3:
      // Reverse, 50%
      speed = -50;
      break;
    case 4:
      // Reverse, 100%
      speed = -100;
      break;
    case 5:
      // Reverse, 120%.  Should limit.
      speed = -120;
      break;
    case 6:
      // Drop through
    default:
      // 0%.  Should stop.
      break;
  }
  // Use motor 0 first, then 1 and repeat.
  int motor_select = (state % (max_states * 2)) / max_states;
  motor_drive(&(motors[motor_select]), speed, duration_ms);
}

void motor_test_tick() {
  static int tick_count = 0;
  static int state = 0;
  // Change state every 2 seconds (20 ticks).
  ++tick_count;
  if (tick_count % 20 == 0) {
    ESP_LOGI(TAG, "%s, %d", __FUNCTION__, state);
    // motor_test_motor_set_mode(state);
    motor_test_motor_drive(state);
    ++state;
  }
}
#endif  // TESTING

/********* API functions *********/

void motors_init() {
  led_pwm_gpio_init(MOTOR_GPIO_PWMA);
  led_pwm_gpio_init(MOTOR_GPIO_PWMB);
  motors_init_pwms();
  // Calculate the maximum duty cycle for the motors as a percentage.
  // Using 8 bit duty resolution.
  const int max_duty = 255;
  uint8_t max_duty_value =
      (uint8_t)((max_duty * MOTOR_MOTOR_VOLTAGE) / MOTOR_BATTERY_VOLTAGE);
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    motor_t *motor = &(motors[i]);
    motor->tick_count = 0;
    motor->max_duty_value = max_duty_value;
    motor->speed = 0;
    motor->speed_mode = MOTOR_PWM_SPEED_MODE;
    if (i == MOTOR_INDEX_LEFT) {
      motor->gpio_in1 = MOTOR_GPIO_AIN1;
      motor->gpio_in2 = MOTOR_GPIO_AIN2;
      motor->gpio_pwm = MOTOR_GPIO_PWMA;
    }
    if (i == MOTOR_INDEX_RIGHT) {
      motor->gpio_in1 = MOTOR_GPIO_BIN1;
      motor->gpio_in2 = MOTOR_GPIO_BIN2;
      motor->gpio_pwm = MOTOR_GPIO_PWMB;
    }
  }
  ESP_LOGI(TAG, "Motors initialised");
}

void motors_tick() {
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    motor_t *motor = &(motors[i]);
    if (motor->tick_count > 0) {
      --motor->tick_count;
      ESP_LOGI(TAG, "%s, motor %d, speed %d, tick_count %d", __FUNCTION__,
               motor->gpio_in1, motor->speed, motor->tick_count);
      if (motor->speed != 0 && motor->tick_count <= 0) {
        ESP_LOGI(TAG, "%s, stopping motor %d", __FUNCTION__, motor->gpio_in1);
        motor_set_mode(motor, MOTOR_MODE_STOP);
        motor_set_duty(motor, 0);
        motor->speed = 0;
        motor->tick_count = 0;
      }
    }
  }
#if TESTING
  motor_test_tick();
#endif
}

void motors_drive(int8_t speed_left, int8_t speed_right, uint16_t ticks) {
  ESP_LOGI(TAG, "%s, %d, %d, %d", __FUNCTION__, speed_left, speed_right, ticks);
  motor_drive(&(motors[MOTOR_INDEX_LEFT]), speed_left, ticks);
  motor_drive(&(motors[MOTOR_INDEX_RIGHT]), speed_right, ticks);
}
