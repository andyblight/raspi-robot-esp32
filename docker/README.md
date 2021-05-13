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
```

Change the IP address of your host PC, edit the file `firmware/mcu_ws/colcon.meta` and modify the option `-DRMW_UXRCE_DEFAULT_UDP_IP=192.168.1.1`.

Now go to the micro-ROS Transport Settings → WiFi Configuration menu and fill your WiFi SSID and password. Save your changes, exit the interactive menu, and run:

```bash
ros2 run micro_ros_setup build_firmware.sh
```

Connect your ESP32 to the computer with a micro-USB cable, and run:

```bash
ros2 run micro_ros_setup flash_firmware.sh
```

Then in another shell, run the pre-built micro-ROS agent using this command:

```bash
docker run -it --rm --net=host microros/micro-ros-agent:foxy udp4 --port 8888 -v6
```

If you examine the output from the agent, you should see messages being sent and received.  You may need to reboot the ESP32 (press and release the EM button) for the device to connect.
