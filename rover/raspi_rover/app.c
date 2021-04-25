#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/error_handling.h>
#include <rmw_uros/options.h>
#include <std_msgs/msg/int32.h>
#include <stdio.h>
#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "geometry_msgs/msg/twist_stamped.h"
#include "geometry_msgs/msg/twist.h"
#include "raspi_robot_driver.h"
#include "raspi_robot_msgs/msg/leds.h"
#include "raspi_robot_msgs/msg/motors.h"
#include "raspi_robot_msgs/srv/sonar_position.h"
#include "sensor_msgs/msg/battery_state.h"
#include "sensor_msgs/msg/range.h"

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

// Number of executor handles: one timer, three subscribers, one service.
// Publishers don't count as they are driven by the timer.
#define EXECUTOR_HANDLE_COUNT (4)

rcl_publisher_t publisher_battery_state;
rcl_publisher_t publisher_range;
rcl_subscription_t subscriber_cmd_vel;
rcl_subscription_t subscriber_leds;
// rcl_subscription_t subscriber_motors;
rcl_service_t service_sonar_position;

// Logging name.
static const char *TAG = "raspi_rover";
// Messages to publish.
static sensor_msgs__msg__BatteryState *battery_state_msg = NULL;
static sensor_msgs__msg__Range *range_msg = NULL;

static void publish_battery_state(void) {
  status_t status;
  raspi_robot_get_status(&status);
  // Fill message.
  range_msg->radiation_type = 0;   // Ultrasound.
  range_msg->field_of_view = 0.5;  // radians (about 30 degrees).
  range_msg->min_range = 0.05;     // metres
  range_msg->min_range = 4.0;      // metres
  range_msg->range = (float)(status.sonar_mm) / 1000;  // metres
  ESP_LOGI(TAG, "Sending msg: %f", range_msg->range);
  rcl_ret_t rc = rcl_publish(&publisher_range, range_msg, NULL);
  RCLC_UNUSED(rc);
}

static void publish_range(void) {
  // Fill message.
  battery_state_msg->voltage = raspi_robot_get_battery_voltage();
  // Convert from milli-Volts to Volts.
  battery_state_msg->voltage /= 1000;
  battery_state_msg->power_supply_technology =
      sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LIPO;
  battery_state_msg->present = true;
  ESP_LOGI(TAG, "Sending msg: %f", battery_state_msg->voltage);
  rcl_ret_t rc = rcl_publish(&publisher_battery_state, battery_state_msg, NULL);
  RCLC_UNUSED(rc);
}

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  ESP_LOGI(TAG, "Timer called.");
  if (timer != NULL) {
    publish_battery_state();
    publish_range();
  }
}

static void subscription_callback_cmd_vel(const void *msg_in) {
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msg_in;
  float forward = msg->linear.x;
  float rotate = msg->angular.z;
  ESP_LOGI(TAG, "Received: forward %f, rotate %f", forward, rotate);
}

static void subscription_callback_leds(const void *msg_in) {
  const raspi_robot_msgs__msg__Leds *msg =
      (const raspi_robot_msgs__msg__Leds *)msg_in;
  ESP_LOGI(TAG, "Received: %d, %d", msg->led, msg->flash_rate);
  raspi_robot_set_led(msg->led, msg->flash_rate);
}

// static void subscription_callback_motors(const void *msg_in) {
//   const raspi_robot_msgs__msg__Motors *msg =
//       (const raspi_robot_msgs__msg__Motors *)msg_in;
//   ESP_LOGI(TAG, "Received: %d, %d, %d", msg->left_percent, msg->right_percent,
//            msg->duration_ms);
//   // Call motor functions.
//   uint16_t ticks = msg->duration_ms / MS_PER_TICK;
//   raspi_robot_motors_drive(msg->left_percent, msg->right_percent, ticks);
// }

void service_sonar_position_callback(const void *req, void *res) {
  raspi_robot_msgs__srv__SonarPosition_Request *request =
      (raspi_robot_msgs__srv__SonarPosition_Request *)req;
  raspi_robot_msgs__srv__SonarPosition_Response *response =
      (raspi_robot_msgs__srv__SonarPosition_Response *)res;
  int16_t x = request->x;
  int16_t y = request->y;
  ESP_LOGI(TAG, "Requested: x %d, y %d", x, y);
  raspi_robot_servo_set(&x, &y);
  response->x = x;
  response->y = y;
}

void appMain(void *arg) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  // Initialise messages.
  battery_state_msg = sensor_msgs__msg__BatteryState__create();
  range_msg = sensor_msgs__msg__Range__create();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  rcl_node_t node = rcl_get_zero_initialized_node();
  RCCHECK(rclc_node_init_default(&node, "raspi_rover", "", &support));

  // Create publishers.
  ESP_LOGI(TAG, "Creating publishers");

  RCCHECK(rclc_publisher_init_default(
      &publisher_battery_state, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      "battery_state"));

  RCCHECK(rclc_publisher_init_default(
      &publisher_range, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "range"));

  // Create subscribers.
  ESP_LOGI(TAG, "Creating subscribers");
  RCCHECK(rclc_subscription_init_default(
      &subscriber_cmd_vel, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "raspi_robot_cmd_vel"));

  RCCHECK(rclc_subscription_init_default(
      &subscriber_leds, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(raspi_robot_msgs, msg, Leds),
      "raspi_robot_leds"));

  // RCCHECK(rclc_subscription_init_default(
  //     &subscriber_motors, &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(raspi_robot_msgs, msg, Motors),
  //     "raspi_robot_motors"));

  // Create services.
  ESP_LOGI(TAG, "Creating services");
  RCCHECK(rclc_service_init_default(
      &service_sonar_position, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(raspi_robot_msgs, srv, SonarPosition),
      "raspi_robot_sonar_position"));

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
  geometry_msgs__msg__Twist twist_msg;
  RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber_cmd_vel, &twist_msg,
      &subscription_callback_cmd_vel, ON_NEW_DATA));

  ESP_LOGI(TAG, "Adding led subs");
  raspi_robot_msgs__msg__Leds leds_msg;
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_leds, &leds_msg,
                                         &subscription_callback_leds,
                                         ON_NEW_DATA));

  // ESP_LOGI(TAG, "Adding motor subs");
  // raspi_robot_msgs__msg__Motors motors_msg;
  // RCCHECK(rclc_executor_add_subscription(
  //     &executor, &subscriber_motors, &motors_msg, &subscription_callback_motors,
  //     ON_NEW_DATA));

  ESP_LOGI(TAG, "Adding services");
  raspi_robot_msgs__srv__SonarPosition_Request request;
  raspi_robot_msgs__srv__SonarPosition_Response response;
  RCCHECK(rclc_executor_add_service(&executor, &service_sonar_position,
                                    &request, &response,
                                    &service_sonar_position_callback));

  // Flash the blue LED when running.
  raspi_robot_init();
  raspi_robot_set_led(RASPI_ROBOT_LED_ESP_BLUE, RASPI_ROBOT_LED_FLASH_2HZ);

  ESP_LOGI(TAG, "Spinning...");
  while (1) {
    rclc_executor_spin_some(&executor, 100);
    usleep(US_PER_TICK);
    raspi_robot_tick();
  }

  // Free resources.
  ESP_LOGI(TAG, "Free resources");
  RCCHECK(rcl_subscription_fini(&subscriber_cmd_vel, &node));
  RCCHECK(rcl_subscription_fini(&subscriber_leds, &node));
  // RCCHECK(rcl_subscription_fini(&subscriber_motors, &node));
  RCCHECK(rcl_publisher_fini(&publisher_battery_state, &node))
  RCCHECK(rcl_publisher_fini(&publisher_range, &node))
  RCCHECK(rcl_service_fini(&service_sonar_position, &node));
  RCCHECK(rcl_node_fini(&node))
  sensor_msgs__msg__BatteryState__destroy(battery_state_msg);
  sensor_msgs__msg__Range__destroy(range_msg);

  vTaskDelete(NULL);
}
