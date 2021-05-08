#include "app_messages.h"

#include "esp_log.h"
#include "geometry_msgs/msg/twist.h"
#include "nav_msgs/msg/odometry.h"
#include "raspi_robot_driver.h"
#include "sensor_msgs/msg/battery_state.h"
#include "sensor_msgs/msg/range.h"

// Information about the robot.
// Wheel
#define WHEEL_CIRCUMFERENCE_M (0.10f)
// Distance between centres of wheels.
#define WHEEL_BASE_M (0.10f)

// Logging name.
static const char *TAG = "app_messages";

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
#if 0
  float velocity = 0.0f;
  float angular = 0.0f;
  raspi_robot_get_odometry(&velocity, &angular);
  // Fill message.
  // now?
  // odometry_msg->header.stamp = now;
  sprintf(odometry_msg->child_frame_id.data, "odom");
  // Pose position.
  odometry_msg->pose.pose.position.x = this->odometry_x;
  odometry_msg->pose.pose.position.y = this->odometry_y;
  odometry_msg->pose.pose.position.z = 0.0f;
  // Pose orientation.
  odometry_msg->pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(this->odometry_theta);
  // Twist linear.
    this->odom_msg.twist.twist.linear.x =
      (this->odometry_x - previous_x)/delta_time;
    this->odom_msg.twist.twist.linear.y =
      (this->odometry_y - previous_y)/delta_time;
    this->odom_msg.twist.twist.linear.z = 0.0f;
  // Twist angular
    this->odom_msg.twist.twist.angular.z =
      (this->odometry_theta - previous_theta)/delta_time;
#endif
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
