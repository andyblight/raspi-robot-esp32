# Micro-ROS ESP32 Build System

One of the most difficult challenges I had to overcome was to work out how to add more source files to the project.

The firmware project directory structure is (only interesting bits shown):

```text
~/ws/firmware/
    dev_ws/
    freertos_apps/
        apps/
            int32_publisher/
                app-colcon.meta
                app.c
            ...
            raspi_rover/
                CMakeLists.txt
                app
                app-colcon.meta
                app.c
                raspi_robot_driver/
                    CMakeLists.txt
                    include/
                        raspi_robot_driver.h
                    src/
                        raspi_robot_driver.c
                        raspi_robot_leds.c
                        raspi_robot_leds.h
                        ...
        microros_esp32_extensions/
            CMakeLists.txt
            build/
            esp32_toolchain.cmake
            esp32_toolchain.cmake.in
            libmicroros.mk
            main/
                CMakeLists.txt
                Kconfig.projbuild
                app.h
                main.c
                microros_transports.c
                microros_transports.h
            sdkconfig
            sdkconfig.defaults
            sdkconfig.old
    mcu_ws/
    toolchain/
```

To start with, I copied the `int32_publisher` files into the `raspi_rover` directory, ran `ros2 run configure_firmware.sh raspi_rover ...` and it built without problems.  I wanted to keep the micro-ROS code separate from the driver code, so I added the directory `raspi_robot_driver` with a header file to expose the driver functions.  This is where I had to take a deep dive into the build system as I could find no instructions on how to add more files.

## The build system

The top level `CMakeLists.txt` file for the project is `firmware/freertos_apps/microros_esp32_extensions/CMakeLists.txt`.  This sets the project value to `UROS_APP`.  The value `UROS_APP` is set by the command

```bash
ros2 run micro_ros_setup configure_firmware.sh <application name> -t udp -i [your local machine IP] -p 8888
```

using the value `<application name>`.  The top level file then includes `firmware/toolchain/esp-idf/tools/cmake/project.cmake` that does "a lot of stuff" and eventually includes the file `firmware/freertos_apps/microros_esp32_extensions/main/CMakeLists.txt`.

Finally figured out how to make it work.  For some reason, you can't just add a subdirectory and use CMakeLists.txt files as you would expect.  This is something to do with the way the ESP build system works (looks like it was coded by a Python programmer who does not know how make works).  So there is a work around where a file `app.cmake` is included from the app directory that contains the list of files to build.  There is also a change to the top of file `firmware/freertos_apps/microros_esp32_extensions/main/CMakeLists.txt` that looks like this:

```cmake
# message("AJB: UROS_APP_FOLDER: " ${UROS_APP_FOLDER})
if(DEFINED UROS_APP_FOLDER)
    include(${UROS_APP_FOLDER}/app.cmake)
endif()
# message("AJB: UROS_APP_INCLUDES: " ${UROS_APP_INCLUDES})
# message("AJB: UROS_APP_SRCS: " ${UROS_APP_SRCS})

idf_component_register(SRCS main.c microros_transports.c ${UROS_APP_SRCS}
                       INCLUDE_DIRS "." ${UROS_APP_INCLUDES}
)

```

## Building the rover code

First time, start the docker and run `./setup_rover.sh`.

To rebuild and flash the code for the rover inside the docker,

```bash
cd ~/ws
. ./install/local_setup.bash
cp -r ~/code/raspi-robot-esp32/rover/raspi_rover/ ~/ws/firmware/freertos_apps/apps/
ros2 run micro_ros_setup build_firmware.sh
ros2 run micro_ros_setup flash_firmware.sh
```

## Building the agent

To test the custom messages, we need to re-build the agent so that the new messages are used.

In shell 1:

```bash
# Copy the message package into the agent workspace.
cp -r ~/ws/firmware/mcu_ws/raspi_robot_msgs/ ~/ws/src
# Build the messages.
cd ~/ws
colcon build --packages-select raspi_robot_msgs
. install/local_setup.bash
# Build the agent.
ros2 run micro_ros_setup build_agent.sh
# Run the agent.
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

In shell 2, test the custom messages and services:

```bash
cd ~/ws
. install/local_setup.bash

ros2 topic pub /raspi_robot/leds raspi_robot_msgs/msg/Leds "{led: 1, flash_rate: 4}"

ros2 topic pub /raspi_robot/motors raspi_robot_msgs/msg/Motors "{left_percent: 30, right_percent: 30, duration_ms: 1000}"

ros2 service call /raspi_robot/sonar_position raspi_robot_msgs/srv/SonarPosition "{x: 10, y: 20}"

ros2 topic pub /raspi_robot_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: -1.0}}"

ros2 topic pub /raspi_robot_cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 1.0}, angular: {z: -1.0}}}"

# Test joystick
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' joy_dev:='/dev/ros2-joystick'
# Teleop with keyboard.
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Changing the custom messages

Changes to the messages need to be copied and built twice, firmware and then agent.

### Firmware

```bash
. ~/ws/install/local_setup.bash
rm -rf ~/ws/firmware/mcu_ws/raspi_robot_msgs/
cp -r ~/code/raspi-robot-esp32/raspi_robot_msgs/ ~/ws/firmware/mcu_ws/
ros2 run micro_ros_setup configure_firmware.sh raspi_rover -t udp -i 192.168.1.1 -p 8888
ros2 run micro_ros_setup build_firmware.sh
ros2 run micro_ros_setup flash_firmware.sh
```

Remember to change the IP address to match your server IP.

The build takes ages after `configure_firmware.sh` as the entire firmware has to be rebuilt.

### Agent

```bash
rm -rf ~/ws/src/raspi_robot_msgs/
cp -r ~/code/raspi-robot-esp32/raspi_robot_msgs/ ~/ws/src
cd ~/ws
colcon build --packages-select raspi_robot_msgs
. install/local_setup.bash
ros2 run micro_ros_setup build_agent.sh
```

## Updating app-colcon.meta

When adding publishers etc. you need to update the `app-colcon.meta` file to tell the build system that number of publishers etc.  This change takes effect when the command `ros2 run micro_ros_setup build_firmware.sh` is run.

Not having the correct numbers in `app-colcon.meta` and the value of `EXECUTOR_HANDLE_COUNT` causes a failure during intialisation.  Check the numbers carefully.

## Adding a common interface message

This is pretty straightforward but there are quirks that you need to bear in mind when using micro-ROS.

### Finding the header files

It is useful to know where to find the header files for the common interface messages so you can see what messages are available.  The key directory to look in is `./firmware/mcu_ws/install/include/` which is only present after a successful build.  It is also worth taking a look at the numerous files that are used for a single message to get an idea of how the messages are intended to be used.

### Diagnostic message

micro-ROS uses a special version of the `common_interfaces/diagnostic_msgs` as variable length fields don't work with micro-ROS.  Instead use `micro_ros_diagnostic_msgs`.

### Message functions

Each message needs to be created and destroyed.  There are several other functions for each message, but the only functions I have needed are create and destroy functions.

The example below shows how the `range` message might be used.

```c
...
#include "sensor_msgs/msg/range.h"

static sensor_msgs__msg__Range *range_msg = NULL;

static void publish_range(void) {
  // Fill message.
  range_msg->radiation_type = 0;   // Ultrasound.
  ...
  rcl_ret_t rc = rcl_publish(&publisher_range, range_msg, NULL);
  RCLC_UNUSED(rc);
}

appMain (void *arg) {
  ...
  // Create messages
  range_msg = sensor_msgs__msg__Range__create();
  ...
  while (1) {
    rclc_executor_spin_some(&executor, 100);
    usleep(US_PER_TICK);
  }
  // Free resources
  sensor_msgs__msg__Range__destroy(range_msg);
  ...
}
```
