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
