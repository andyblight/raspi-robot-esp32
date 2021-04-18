#include "diagnostics.h"

#include <rclc/rclc.h>

#include "diagnostic_msgs/msg/diagnostic_array.h"
#include "diagnostic_msgs/msg/diagnostic_status.h"
#include "diagnostic_msgs/msg/key_value.h"
#include "rosidl_runtime_c/string_functions.h"

#define NUM_STATUS (2)

static void fill_robot_status(diagnostic_msgs__msg__DiagnosticStatus *status) {
  diagnostic_msgs__msg__DiagnosticStatus__init(&status);
  // Serial number or similar.
  rosidl_runtime_c__String__assign(&status.hardware_id, "RPR1");
  // `level` is one of:
  // diagnostic_msgs__msg__DiagnosticStatus__OK
  // diagnostic_msgs__msg__DiagnosticStatus__WARN
  // diagnostic_msgs__msg__DiagnosticStatus__ERROR
  // diagnostic_msgs__msg__DiagnosticStatus__STALE
  status.level = diagnostic_msgs__msg__DiagnosticStatus__OK;
  // A descriptive message.
  rosidl_runtime_c__String__assign(&status.message, "AOK");
  // Robot name.
  rosidl_runtime_c__String__assign(&status.name, "RasPiRover");
  // TODO Add key value pairs here.
}

static void add_wifi_status(diagnostic_msgs__msg__DiagnosticStatus *status) {
  RCLC_UNUSED(status);
}

diagnostic_msgs__msg__DiagnosticArray *diagnositics_init(void) {
  diagnostic_msgs__msg__DiagnosticArray *msg =
      diagnostic_msgs__msg__DiagnosticArray__create();
  return msg;
}
  // Create and initialise instance.
  // Fill out message.
  diagnostics_populate(msg);
  // Publish and tidy up.
  rcl_ret_t rc = rcl_publish(&publisher_diagnostics, m_diagnostic_array, NULL);
  RCLC_UNUSED(rc);
  diagnostic_msgs__msg__DiagnosticArray__destroy(msg);



void diagnostics_populate(diagnostic_msgs__msg__DiagnosticArray *msg) {
  diagnostic_msgs__msg__DiagnosticStatus robot_status;
  diagnostic_msgs__msg__DiagnosticStatus wifi_status;
  fill_robot_status(&robot_status);
  fill_wifi_status(&wifi_status);
}
