# Docker

This directory has scripts that allow you to setup a docker that allows you to run the Espressif IDF tool.  The main reason to use this docker is to run `idf.py monitor`.  The monitor program is the only way to view

## Basic operation

The script `./build.bash` creates the docker container.  Do this just once!

To start the container, use `./start.bash`.  This script starts the container and leaves it running until `./stop.bash` is called. The script `./start.bash` also mounts a workspace directory using the pattern `$HOME/<docker container name>_ws`.

If you need to change the mounted directory on the container, you need to destroy the current container by stopping the container using `./stop.bash`, remove the container using `./rm_container.bash` and then start a new container using `.start.bash`.

When the container is running, you can get a Bash user prompt attached to the container using `./attach.bash`.  The `./attach.bash` script can be used to open multiple terminal sessions on the docker container by calling the script as many times as needed.

## Start the ESP IDF monitor

```bash
idf.py monitor
```

## Verifying the docker

You should do this the first time you setup the docker and after every change to the docker or related scripts.

Build and run the example `int32_publisher` by following these instructions.

Open a new shell on the docker (use `attach.bash`), then enter the commands:

```bash
git clone https://github.com/micro-ROS/micro_ros_espidf_component.git
cd micro_ros_espidf_component/examples/int32_publisher
# Set target board [esp32|esp32s2|esp32c3]
idf.py set-target esp32
idf.py menuconfig
# Set your micro-ROS configuration and WiFi credentials under micro-ROS Settings
idf.py build
idf.py flash
idf.py monitor
```

Then in another shell, run the pre-build micro-ROS agent using this command:

```bash
docker run -it --rm --net=host microros/micro-ros-agent:foxy udp4 --port 8888 -v6
```

If you examine the output from the agent, you should see messages being sent and received.
