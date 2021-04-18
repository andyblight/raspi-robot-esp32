#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_uros/options.h>
#include <std_msgs/msg/int32.h>
#include <stdio.h>
#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "raspi_robot_driver.h"
#include "raspi_robot_msgs/msg/leds.h"
#include "raspi_robot_msgs/msg/motors.h"
//#include "sensor_msgs/msg/battery_state.h"
#include "diagnostic_msgs/msg/diagnostic_array.h"
#include "diagnostics.h"
#include "std_msgs/msg/int32.h"

#define RCCHECK(fn)                                                 \
  {                                                                 \
    rcl_ret_t temp_rc = fn;                                         \
    if ((temp_rc != RCL_RET_OK)) {                                  \
      printf("Failed status on line %d: %d. Aborting.\n", __LINE__, \
             (int)temp_rc);                                         \
      vTaskDelete(NULL);                                            \
    }                                                               \
  }
#define RCSOFTCHECK(fn)                                               \
  {                                                                   \
    rcl_ret_t temp_rc = fn;                                           \
    if ((temp_rc != RCL_RET_OK)) {                                    \
      printf("Failed status on line %d: %d. Continuing.\n", __LINE__, \
             (int)temp_rc);                                           \
    }                                                                 \
  }

// Tick definitions.
#define TICK_RATE_HZ (10)
#define MS_PER_TICK (1000 / TICK_RATE_HZ)
#define US_PER_TICK (MS_PER_TICK * 1000)

// Number of executor handles: one timer, two subscribers.
#define EXECUTOR_HANDLE_COUNT (3)

rcl_publisher_t publisher_battery_state;
rcl_publisher_t publisher_diagnostics;
rcl_subscription_t subscriber_leds;
rcl_subscription_t subscriber_motors;

// Logging name.
static const char *TAG = "raspi_rover";

static void publish_battery_state(void) {
  // ESP_LOGI(TAG, "Timer - Pub pointer: %p", &publisher_battery_state);
  // status_t status;
  // raspi_robot_get_status(&status);
  // float voltage = 1.05;
  // sensor_msgs__msg__BatteryState msg;
  // msg.voltage = voltage;
  // msg.power_supply_technology =
  //     sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LIPO;
  // msg.present = (voltage > 7.0);
  // ESP_LOGI(TAG, "Sending msg: %f, %d, %d", msg.voltage,
  //          msg.power_supply_technology, msg.present);
  std_msgs__msg__Int32 msg;
  msg.data = raspi_robot_get_battery_voltage();
  rcl_ret_t rc = rcl_publish(&publisher_battery_state, &msg, NULL);
  RCLC_UNUSED(rc);
}

static void publish_diagnositics(void) {
  // Create and initialise instance.
  diagnostic_msgs__msg__DiagnosticArray *msg =
      diagnostic_msgs__msg__DiagnosticArray__create();
  // Fill out message.
  diagnostics_populate(msg);
  // Publish and tidy up.
  rcl_ret_t rc = rcl_publish(&publisher_diagnostics, msg, NULL);
  RCLC_UNUSED(rc);
  diagnostic_msgs__msg__DiagnosticArray__destroy(msg);
}

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  ESP_LOGI(TAG, "Timer called.");
  if (timer != NULL) {
    publish_battery_state();
    publish_diagnositics();
  }
}

static void subscription_callback_leds(const void *msgin) {
  const raspi_robot_msgs__msg__Leds *msg =
      (const raspi_robot_msgs__msg__Leds *)msgin;
  ESP_LOGI(TAG, "Received: %d, %d", msg->led, msg->flash_rate);
  raspi_robot_set_led(msg->led, msg->flash_rate);
}

static void subscription_callback_motors(const void *msgin) {
  const raspi_robot_msgs__msg__Motors *msg =
      (const raspi_robot_msgs__msg__Motors *)msgin;
  ESP_LOGI(TAG, "Received: %d, %d, %d", msg->left_percent, msg->right_percent,
           msg->duration_ms);
  // Call motor functions.
  uint16_t ticks = msg->duration_ms / MS_PER_TICK;
  raspi_robot_motors_drive(msg->left_percent, msg->right_percent, ticks);
}

void appMain(void *arg) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  rcl_node_t node = rcl_get_zero_initialized_node();
  RCCHECK(rclc_node_init_default(&node, "raspi_rover", "", &support));

  // Create publishers.
  ESP_LOGI(TAG, "Creating publishers");

  RCCHECK(rclc_publisher_init_default(
      &publisher_battery_state, &node,
      // ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "battery_state"));

  RCCHECK(rclc_publisher_init_default(
      &publisher_diagnostics, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray),
      "diagnostics"));

  // Create subscribers.
  ESP_LOGI(TAG, "Creating subscribers");
  RCCHECK(rclc_subscription_init_default(
      &subscriber_leds, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(raspi_robot_msgs, msg, Leds),
      "raspi_robot_leds"));

  RCCHECK(rclc_subscription_init_default(
      &subscriber_motors, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(raspi_robot_msgs, msg, Motors),
      "raspi_robot_motors"));

  // Create timer.
  ESP_LOGI(TAG, "Creating timers");
  rcl_timer_t timer = rcl_get_zero_initialized_timer();
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout),
                                  timer_callback));

  // Create executor.
  ESP_LOGI(TAG, "Creating executor");
  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, EXECUTOR_HANDLE_COUNT,
                             &allocator));

  unsigned int rcl_wait_timeout = 1000;  // in ms
  RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  ESP_LOGI(TAG, "Adding subs");
  raspi_robot_msgs__msg__Leds leds_msg;
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_leds, &leds_msg,
                                         &subscription_callback_leds,
                                         ON_NEW_DATA));
  raspi_robot_msgs__msg__Motors motors_msg;
  RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber_motors, &motors_msg, &subscription_callback_motors,
      ON_NEW_DATA));

  raspi_robot_init();
  raspi_robot_set_led(RASPI_ROBOT_LED_ESP_BLUE, RASPI_ROBOT_LED_FLASH_2HZ);

  ESP_LOGI(TAG, "Spinning...");
  while (1) {
    rclc_executor_spin_some(&executor, 100);
    usleep(US_PER_TICK);
    raspi_robot_tick();
  }

  // free resources
  ESP_LOGI(TAG, "Free resources");
  RCCHECK(rcl_subscription_fini(&subscriber_motors, &node));
  RCCHECK(rcl_subscription_fini(&subscriber_leds, &node));
  RCCHECK(rcl_publisher_fini(&publisher_battery_state, &node))
  RCCHECK(rcl_publisher_fini(&publish_diagnositics, &node))
  RCCHECK(rcl_node_fini(&node))

  vTaskDelete(NULL);
}
