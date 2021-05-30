#ifndef APP_MESSAGES_H
#define APP_MESSAGES_H

#include "nav_msgs/msg/odometry.h"
#include "sensor_msgs/msg/battery_state.h"
#include "sensor_msgs/msg/range.h"

void messages_battery_state(sensor_msgs__msg__BatteryState *msg);
void messages_range(sensor_msgs__msg__Range *msg);
void messages_odometry(nav_msgs__msg__Odometry *msg);
void messages_cmd_vel(const void *msg_in);
void messages_motors_test(const void *msg_in);
void messages_sonar_position(const void *msg_in);

#endif  // APP_MESSAGES_H