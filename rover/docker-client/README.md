# Docker

This directory has scripts that allow you to setup a docker that allows you to build and run the examples in the [micro-ROS component for ESP-IDF repo](https://github.com/micro-ROS/micro_ros_espidf_component).  NOTE: The repo `micro_ros_espidf_component` is cloned as part of the setup process.

## Basic operation

The script `./build.bash` creates the docker container.  Do this just once!  This process took about 5 minutes on my PC, so get on with something else while the image is built.

To start the container, use `./start.bash`.  This script starts the container and leaves it running until `./stop.bash` is called. The script `./start.bash` also mounts a workspace directory using the pattern `$HOME/<docker container name>_ws`.

If you need to change the mounted directory on the container, you need to destroy the current container by stopping the container using `./stop.bash`, remove the container using `./rm_container.bash` and then start a new container using `.start.bash`.

When the container is running, you can get a Bash user prompt attached to the container using `./attach.bash`.  The `./attach.bash` script can be used to open multiple terminal sessions on the docker container by calling the script as many times as needed.

__IMPORTANT:__ The micro-ROS setup scripts install lots of packages into the the docker container.  Use the `commit.bash` script to save the state of the container after you have successfully built the firmware.  If you don't commit your changes, you will have to run the setup scripts again.

## Verifying the docker

You should do this the first time you setup the docker and after every change to the docker or related scripts.

Build and run the example `int32_publisher` by following these instructions.

Open a new shell on the docker (use `attach.bash`), then enter the commands:

```bash
ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32
ros2 run micro_ros_setup configure_firmware.sh int32_publisher -t udp -i [your local machine IP] -p 8888
ros2 run micro_ros_setup build_firmware.sh menuconfig

# Now go to the micro-ROS Transport Settings → WiFi Configuration menu and fill your WiFi SSID and password. Save your changes, exit the interactive menu, and run:
ros2 run micro_ros_setup build_firmware.sh

# Connect your ESP32 to the computer with a micro-USB cable, and run:
ros2 run micro_ros_setup flash_firmware.sh
```

Then in another shell, run the pre-build micro-ROS agent using this command:

```bash
docker run -it --rm --net=host microros/micro-ros-agent:foxy udp4 --port 8888 -v6
```

If you examine the output from the agent, you should see messages being sent and received.

## Building the RasPiRobot ESP32 software

This is I managed to get a project to build.

```bash
cd ~/ws/micro_ros_espidf_component/examples
cp -r int32_publisher raspi_rover
cd raspi_rover/
nano CMakeLists.txt
# Changed name of project to 'raspi_rover'
idf.py set-target esp32
idf.py menuconfig
# Set server address and Wi-Fi SSID and password
idf.py build
idf.py flash
```

Now I have to create scripts to copy the files to and from my repo.

### Building and testing the rover code

TODO

#### Start the ESP IDF monitor

```bash
export IDF_PATH=~/ws/firmware/toolchain/esp-idf/
. firmware/toolchain/esp-idf/export.sh

```

Adding ESP-IDF tools to PATH...
/usr/bin/env: ‘python’: No such file or directory

