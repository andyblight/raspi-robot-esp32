#include "app_messages.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "geometry_msgs/msg/twist.h"
#include "nav_msgs/msg/odometry.h"
#include "raspi_robot_driver.h"
#include "rosidl_runtime_c/string.h"
#include "sensor_msgs/msg/battery_state.h"
#include "sensor_msgs/msg/range.h"
#include "std_msgs/msg/header.h"

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
// Ticks is preset to 1 second, 10 ticks.
#define MOTOR_TICKS (10)

// FIXME(AJB) Hack based on knowledge of system at time of writing.
#define ODOMETRY_CALL_INTERVAL_MS (1000)
#define ODOMETRY_CALL_INTERVAL_S (ODOMETRY_CALL_INTERVAL_MS / MS_PER_S)

// Logging name.
static const char *TAG = "app_messages";

/**** Private functions ****/
static void get_now(int32_t *secs, uint32_t *nanosecs) {
  static int32_t last_secs = 0;
  static int32_t last_nanosecs = 0;
  // Get num ns since last call.
  //   Get tick count and convert to nanoseconds since last call.
  static uint32_t last_tick_count = 0;
  uint32_t tick_count_diff = 0;
  uint32_t tick_count = (uint32_t)xTaskGetTickCount();
  // Handle tick count wrap around.
  if (last_tick_count > tick_count) {
    if (configUSE_16_BIT_TICKS == 1) {
      tick_count_diff = UINT16_MAX - last_tick_count;
    } else {
      tick_count_diff = UINT32_MAX - last_tick_count;
    }
    tick_count_diff += tick_count;
  } else {
    tick_count_diff = tick_count - last_tick_count;
  }
  // Convert tick count diff to ns.
  uint32_t diff_ns = (tick_count_diff / portTICK_RATE_MS) * NS_PER_MS;
  // Handle nanosecond overflow.
  last_nanosecs += diff_ns;
  if (last_nanosecs > NS_PER_S) {
    ++last_secs;
    last_nanosecs -= NS_PER_S;
  }
  // Set results.
  *secs = last_secs;
  *nanosecs = last_nanosecs;
}

static void set_rosidl_c_string(rosidl_runtime_c__String *string,
                                const char *new_string) {
  // Make sure allocated space is big enough. +1 for null terminator.
  size_t new_string_length = strlen(new_string) + 1;
  if (new_string_length > string->capacity) {
    free(string->data);
    string->data = (char *)malloc(new_string_length);
  }
  // Set new value.
  snprintf(string->data, new_string_length, "%s", new_string);
  string->size = new_string_length;
}

// Fill the header stamp variable.
// Contents defined in builtin_interfaces/msg/detail/time__struct.h
void set_message_header(const char *frame_id, std_msgs__msg__Header *header) {
  int32_t secs = 0;
  uint32_t nanosecs = 0;
  get_now(&secs, &nanosecs);
  // int32_t
  header->stamp.sec = secs;
  // uint32_t
  header->stamp.nanosec = nanosecs;
  set_rosidl_c_string(&header->frame_id, frame_id);
}

// Based on code from here:
// https://answers.ros.org/question/241602/get-odometry-from-wheels-encoders/
static void calculate_odometry(const float delta_time_s, float *pose_x_m,
                               float *pose_y_m, float *pose_theta,
                               float *velocity_x_m_s, float *velocity_y_m_s,
                               float *velocity_theta) {
  // Get the encoder values since last call.
  int16_t delta_left = 0;
  int16_t delta_right = 0;
  raspi_robot_get_encoders(&delta_left, &delta_right);
  ESP_LOGI(TAG, "Odom encoders: left %d, right %d", delta_left, delta_right);
  // Convert encoder ticks to speed for each wheel in m/s.
  float speed_left_m_s = delta_left * METRES_PER_ENCODER_TICK / delta_time_s;
  float speed_right_m_s = delta_right * METRES_PER_ENCODER_TICK / delta_time_s;
  ESP_LOGI(TAG, "Odom speed: left %f, right %f", speed_left_m_s,
           speed_left_m_s);
  // Calculate velocities.
  *velocity_x_m_s = (speed_left_m_s + speed_right_m_s) / 2.0f;
  *velocity_y_m_s = 0.0f;
  *velocity_theta = (speed_left_m_s + speed_right_m_s) / WHEEL_BASE_M;
  ESP_LOGI(TAG, "Odom velocity: x %f, y %f, theta %f", *velocity_x_m_s,
           *velocity_y_m_s, *velocity_theta);
  // Calculate the delta pose values and add (cumulative).
  *pose_x_m += (*velocity_x_m_s * cos(*velocity_theta)) * delta_time_s;
  *pose_y_m += (*velocity_x_m_s * sin(*velocity_theta)) * delta_time_s;
  *pose_theta += *velocity_theta * delta_time_s;
  ESP_LOGI(TAG, "Odom pose: x %f, y %f, theta %f", *pose_x_m,
           *pose_y_m, *pose_theta);
}

/**** API functions ****/

void messages_battery_state(sensor_msgs__msg__BatteryState *msg) {
  set_message_header("battery_state", &msg->header);
  // Fill message.
  msg->voltage = raspi_robot_get_battery_voltage();
  // Convert from milli-Volts to Volts.
  msg->voltage /= 1000;
  msg->power_supply_technology =
      sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LIPO;
  msg->present = true;
}

void messages_range(sensor_msgs__msg__Range *msg) {
  // Get distance reported by sonar.
  status_t status;
  raspi_robot_get_status(&status);
  // Fill message.
  set_message_header("range", &msg->header);
  msg->radiation_type = 0;   // Ultrasound.
  msg->field_of_view = 0.5;  // radians (+/- 15 degrees ish).
  msg->min_range = 0.05;     // metres
  msg->min_range = 2.0;      // metres
  msg->range = (float)(status.sonar_mm) / 1000;  // metres
}

void messages_odometry(nav_msgs__msg__Odometry *msg) {
  // Pose values are cumulative.
  static float pose_x_m = 0.0f;
  static float pose_y_m = 0.0f;
  static float pose_theta = 0.0f;
  // Velocities are instantaneous.
  float velocity_x_m_s = 0.0f;
  float velocity_y_m_s = 0.0f;
  float velocity_theta = 0.0f;
  // Get the odometry values.
  calculate_odometry(ODOMETRY_CALL_INTERVAL_S, &pose_x_m, &pose_y_m,
                     &pose_theta, &velocity_x_m_s, &velocity_y_m_s,
                     &velocity_theta);
  // Fill the message.
  set_message_header("odom", &msg->header);
  set_rosidl_c_string(&msg->child_frame_id, "base_link");
  // Pose position.
  msg->pose.pose.position.x = pose_x_m;
  msg->pose.pose.position.y = pose_y_m;
  msg->pose.pose.position.z = 0.0f;
  // TODO
  //   msg->pose.pose.orientation =
  //       tf::createQuaternionMsgFromYaw(pose_theta);
  // Velocities in the Twist message.
  msg->twist.twist.linear.x = velocity_x_m_s;
  msg->twist.twist.linear.y = velocity_y_m_s;
  msg->twist.twist.angular.z = velocity_theta;
}

void messages_cmd_vel(const geometry_msgs__msg__Twist *msg) {
  // Only use two variables in the message for differential drive.
  float forward = msg->linear.x;
  float rotate = msg->angular.z;
  ESP_LOGI(TAG, "Received: forward %f, rotate %f", forward, rotate);
  // Convert float to percent within motor limits.
  int8_t left_percent = 0;
  int8_t right_percent = 0;
  // Do most calculations using absolute velocity.
  if (forward < 0.0) {
    forward = -forward;
  }
  // Clamp the maximum speed.
  if (forward > MAXIMUM_SPEED_M_S) {
    forward = MAXIMUM_SPEED_M_S;
  }
  // Convert the forward value to a motor percent.
  if (forward > MINIMUM_SPEED_M_S) {
    int8_t forward_percent =
        (int8_t)((float)(forward / MAXIMUM_SPEED_M_S) * 100);
    // Clamp minimum percent.
    if (forward_percent < MINIMUM_MOTOR_PERCENT) {
      forward_percent = MINIMUM_MOTOR_PERCENT;
    }
    // Add sign back in and convert to left and right values.
    // To go forward, one motor is +ve and the other is -ve.
    if (forward_percent > 0) {
      left_percent = forward_percent;
      right_percent = -forward_percent;
    } else {
      left_percent = -forward_percent;
      right_percent = forward_percent;
    }
  }  // Any value less than minimum speed is ignored.
  raspi_robot_motors_drive(left_percent, right_percent, MOTOR_TICKS);
}
