# Docker

This directory has scripts that allow you to setup a docker that allows you to build and run the examples in the [micro-ROS component for ESP-IDF repo](https://github.com/micro-ROS/micro_ros_espidf_component).  NOTE: The repo `micro_ros_espidf_component` is cloned as part of the setup process.

NOTE: If you are using a joystick to teleop the rover, then you need to add a udev rule to your host Linux PC to allow the joystick to be consistently passed through to the docker.  See ["Set up joystick"](#set-up-joystick).

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

## Set up joystick

The joystick driver used by ROS2 expects the joystick to appear on a device node named like this `/dev/input/eventXX` where `XX` is the number of the event.  The reason for this is something to do with using `SDL`, the [Simple DirectMedia Layer](https://www.libsdl.org/).  This is a bit of pain as the number of the event changes based on what hardware has been added.  To get around this problem and allow us to pass a consistent joystick node into the docker, we need to add a `udev` rule to the host PC. Note about it is [here.](https://github.com/ros-drivers/joystick_drivers/tree/ros2/joy#technical-note-about-interfacing-with-the-joystick-on-linux)

As this is a one off per host, I'm going to document the process and not script it.  The basic concept is described [here](https://github.com/ros-drivers/joystick_drivers/tree/ros2/joy/udev) but I have added the actual steps I took to make it work.

I identified the event being used by the joystick by listing the directory `/dev/input` before and after plugging in the joystick.

```bash
$ ls /dev/input
by-id  by-path  event0  event1  event10  event11  event12  event13  event14  event15  event2  event3  event4  event5  event6  event7  event8  event9  mice  mouse0
$ ls /dev/input
by-id  by-path  event0  event1  event10  event11  event12  event13  event14  event15  event16  event2  event3  event4  event5  event6  event7  event8  event9  js0  mice  mouse0
```

You can see that the device nodes `event16` and `js0` are created when the joystick is plugged in.  The next thing to do was to find out the information needed to create the `udev` rule.

```bash
 udevadm info -a -n /dev/input/event16

Udevadm info starts with the device specified by the devpath and then
walks up the chain of parent devices. It prints for every device
found, all possible attributes in the udev rules key format.
A rule to match, can be composed by the attributes of the device
and the attributes from one single parent device.

  looking at device '/devices/pci0000:00/0000:00:14.0/usb3/3-1/3-1:1.0/input/input20/event16':
    KERNEL=="event16"
    SUBSYSTEM=="input"
    DRIVER==""

  looking at parent device '/devices/pci0000:00/0000:00:14.0/usb3/3-1/3-1:1.0/input/input20':
    KERNELS=="input20"
    SUBSYSTEMS=="input"
    DRIVERS==""
    ATTRS{phys}=="usb-0000:00:14.0-1/input0"
    ATTRS{name}=="Microsoft X-Box 360 pad"
    ATTRS{uniq}==""
    ATTRS{properties}=="0"
...
```

We need this value `ATTRS{name}=="Microsoft X-Box 360 pad"`.  I then created the `udev` rule file, `99-xbox360-wired.rules` and copied in the contents of the [example file](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/udev/99-logitech-f710.rules).  I changed the `ATTRS{name}` value and the `SYMLINK` value and ended up with this:

```rules
KERNEL=="event[0-9]*", SUBSYSTEM=="input", ATTRS{name}=="Microsoft X-Box 360 pad", ACTION=="add", SYMLINK+="ros2_joystick", MODE="666"
```

Then I tested it to see where the `SYMLINK` value appeared under `/dev`.

```bash
sudo cp docker/99-xbox360-wired.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

After reloading the rules, you need to unplug and plug in the joystick for the rule to take effect.  The new device node appears here `/dev/ros-joystick`.  I then modified the `start.bash` script to attach the `/dev/ros2-joystick` node to the docker and the node appears in my docker as expected.

```bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' joy_dev:='/dev/ros2-joystick'
```

When I started the `joy` node, nothing happened!

Tried various things for 2 hours.  Still can't get it working so using keyboard instead.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
