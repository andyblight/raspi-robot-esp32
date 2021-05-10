#include "app_messages.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "geometry_msgs/msg/twist.h"
#include "nav_msgs/msg/odometry.h"
#include "raspi_robot_driver.h"
#include "sensor_msgs/msg/battery_state.h"
#include "sensor_msgs/msg/range.h"
#include "std_msgs/msg/header.h"

// Information about the robot.
// Wheel
#define WHEEL_CIRCUMFERENCE_M (0.10f)
// Distance between centres of wheels.
#define WHEEL_BASE_M (0.10f)
// FIXME(AJB) Hack based on knowledge of system at time of writing.
#define MS_PER_S (1000)
#define NS_PER_MS (1000 * 1000)
#define NS_PER_S (1000 * 1000 * 1000)
#define ODOMETRY_CALL_INTERVAL_MS (1000)
#define ODOMETRY_CALL_INTERVAL_S (ODOMETRY_CALL_INTERVAL_MS / MS_PER_S)
// Motor constants.
#define MAXIMUM_SPEED_M_S (0.50)
#define MINIMUM_SPEED_M_S (0.10)
// The motors do not move the robot when less than this value.
#define MINIMUM_MOTOR_PERCENT (30)
// Ticks is preset to 1 second, 10 ticks.
#define MOTOR_TICKS (10)

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

// Fill the header stamp variable.
// TODO(AJB) Use frame_id string properly.
// Contents defined in builtin_interfaces/msg/detail/time__struct.h
void set_message_header(std_msgs__msg__Header *header) {
  int32_t secs = 0;
  uint32_t nanosecs = 0;
  get_now(&secs, &nanosecs);
  // int32_t
  header->stamp.sec = secs;
  // uint32_t
  header->stamp.nanosec = nanosecs;
  sprintf(header->frame_id.data, "To do!");
}

static void calculate_odometry(const float *delta_time, float *x, float *y,
                               float *theta) {
  // Get the encoder values.
  int16_t left = 0;
  int16_t right = 0;
  raspi_robot_get_encoders(&left, &right);
  ESP_LOGI(TAG, "Encoder counts: left %d, right %d", left, right);
#if 1
  *x = -0.1f;
  *y = 0.1f;
  *theta = 0.2f;
#else
  // Divide the relative encoder counts by 11 for the gear ratio
  double left_wheel_speed = encoder1 / 11.0f;
  double right_wheel_speed = encoder2 / 11.0f;
  // Calculate the wheel speed from the delta_time
  left_wheel_speed /= delta_time;
  right_wheel_speed /= delta_time;
  // Divide by 1000 to convert from encoder_ticks_per_second to rps
  left_wheel_speed /= 1000.0f;
  right_wheel_speed /= 1000.0f;
  // Convert rps to mps for each wheel
  double wheel_circumference = this->wheel_radius_ * 2.0f * M_PI;
  left_wheel_speed *= wheel_circumference;
  right_wheel_speed *= wheel_circumference;
  // Calculate velocities
  double velocity = 0.0f;
  velocity += this->wheel_radius_ * right_wheel_speed;
  velocity += this->wheel_radius_ * left_wheel_speed;
  velocity /= 2.0f;
  double angular_velocity = 0.0f;
  angular_velocity -=
      this->wheel_radius_ / this->track_width_ * left_wheel_speed;
  angular_velocity +=
      this->wheel_radius_ / this->track_width_ * right_wheel_speed;
  // Calculate new positions
  x += delta_time * velocity *
       cos(theta + (angular_velocity / 2.0f) * delta_time);
  y += delta_time * velocity *
       sin(theta + (angular_velocity / 2.0f) * delta_time);
  theta += angular_velocity * delta_time;
#endif
}

/**** API functions ****/

void messages_battery_state(sensor_msgs__msg__BatteryState *battery_state_msg) {
  // Fill message.
  battery_state_msg->voltage = raspi_robot_get_battery_voltage();
  // Convert from milli-Volts to Volts.
  battery_state_msg->voltage /= 1000;
  battery_state_msg->power_supply_technology =
      sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LIPO;
  battery_state_msg->present = true;
}

void messages_range(sensor_msgs__msg__Range *range_msg) {
  status_t status;
  raspi_robot_get_status(&status);
  // Fill message.
  range_msg->radiation_type = 0;   // Ultrasound.
  range_msg->field_of_view = 0.5;  // radians (+/- 15 degrees ish).
  range_msg->min_range = 0.05;     // metres
  range_msg->min_range = 4.0;      // metres
  range_msg->range = (float)(status.sonar_mm) / 1000;  // metres
}

void messages_odometry(nav_msgs__msg__Odometry *odometry_msg) {
  static float previous_x = 0.0f;
  static float previous_y = 0.0f;
  static float previous_theta = 0.0f;
  // Get the odometry values.
  float delta_time = ODOMETRY_CALL_INTERVAL_S;
  float x = 0.0f;
  float y = 0.0f;
  float theta = 0.0f;
  calculate_odometry(&delta_time, &x, &y, &theta);
  // Fill the message.
  set_message_header(&odometry_msg->header);
  sprintf(odometry_msg->child_frame_id.data, "odom");
  // Pose position.
  odometry_msg->pose.pose.position.x = x;
  odometry_msg->pose.pose.position.y = y;
  odometry_msg->pose.pose.position.z = 0.0f;
  // Pose orientation.
  // TODO
  //   odometry_msg->pose.pose.orientation =
  //       tf::createQuaternionMsgFromYaw(this->odometry_theta);
  // Twist linear.
  odometry_msg->twist.twist.linear.x = (x - previous_x) / delta_time;
  odometry_msg->twist.twist.linear.y = (y - previous_y) / delta_time;
  odometry_msg->twist.twist.linear.z = 0.0f;
  // Twist angular
  odometry_msg->twist.twist.angular.z = (theta - previous_theta) / delta_time;
  // Update previous_* values for next time.
  previous_x = x;
  previous_y = y;
  previous_theta = theta;
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
