# ESP32 Rover

The ESP-WROOM-32 module was chosen as it was directly supported by the
Micro-ROS project.  The unit I had was bought from
[Amazon](https://smile.amazon.co.uk/gp/product/B071JR9WS9).  There are many
similar products but they have different pin layouts so beware!  There are
several variants of the ESP32 available.  The good news is that most have are
fitted with at least 4MB of Flash, and the rest is all pretty similar.  The
ESP32-CAM modules I have don't have enough GPIO pins for this robot but
might be useful for other projects.

![ESP-WROOM-32](resources/esp32-wroom-thumb.jpg "ESP-WROOM-32")

[Larger image](resources/esp32-wroom.jpg)

The [RasPiRobot Rover Robot Kit](https://cpc.farnell.com/monkmakes/sku00049/raspirobot-rover-robot-kit/dp/SC14457)
was chosen as I had it lying around, it was simple and I had all the
documentation.

![RasPiRobot Rover Robot](resources/RasPiRobot.jpg "RasPiRobot Rover Robot")

This kit uses the RasPiRobot V3f board to control the motors and implement the sonar interface.

![RasPiRobot v3f Board](resources/RasPiRobotv3f-thumb.jpg "RasPiRobot V3f board")

[Larger image](resources/RasPiRobotv3f.jpg)

and the HC-SR04 sonar unit.

![HC-SRO4](resources/HC-SR04.jpg "HC-SRO4")

The wiring schedule is [here](wiring.md).

The encoders used for the wheels were obtained from [here](http://www.pridopia.co.uk/pi-motor-encoder-2p.html).  There seems to be no documentation for the hardware, so I'm not sure how it works.

The `resources` directory contains the data sheets used during development of the
drivers.

## Development Setup

The [docker README.md file](docker-client/README.md) explains how to setup the environment for the rover.

## References

Useful links and the pin out of the
[DOIT ESP32S](https://github.com/playelek/pinout-doit-32devkitv1).

[ESP32 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)

[vfRasPiRobot board schematics](https://github.com/simonmonk/raspirobotboard3).

[Details of the ESP32 board used](https://github.com/Nicholas3388/LuaNode).

## Notes

This way of working is slow and painful.  The Arduino method is much quicker.

Can the Arduino method be used with the ESP32?  The difficulty is micro-ROS.
