# Motor Calibration Driver

In an ideal world, I would be able to use continuous servo motors to control the wheels of my robot.  Instead, I'm using DC motors and optical encoders, very much cheaper but much more difficult to control using software.

Most motor controllers are designed around a PID controller that are designed to set the motor speed over a number of iterations to a constant speed.  This approach is only useful when a robot is travelling over long distances.  This robot has a wheel circumference of about 21cm, so when making small adjustments, say 5cm, the wheel needs to be turned about a quarter of a turn and that corresponds to 3 slots on the encoder wheel.

The other problem is that many cheap DC motors do not start rotating with the same input power.  For instance, with the wheels suspended, the left motor of this robot starts turning forward at 16% duty cycle and the right motor starts turning forward at 23%.  So using bare duty cycle is going to cause problems.  This is where the idea of motor calibration comes in.

We also need to consider the battery voltage being used to drive the motors.  In our case, the motors are rated at 6V but we are driving them from a 2S LiPo battery with an output voltage of between 6.4V and 8.4V.  To prevent overheating the motors, the amount of power being applied to the motors is the important value to consider.  Assuming the resistance of the motor is constant, the amount of power being applied to the motor is determined by the voltage being applied times the duty cycle (in the range of 0 to 1).  The battery voltage is something that is easy to measure so can be used as an input to the control algorithm.

Basically, the idea is to implement a look up table for each motor that can be used to convert the desired rotational speed into a duty cycle value to drive the motor.  The use of optical encoders makes it possible to test different duty cycle values and record the effect on the output shaft.

The motors will be calibrated by sending a command to the robot that then performs a series of maneuvers and records the results.  The results are then converted into a look up table and stored in NVM.  The saved look up table is then used to control the motors.  The calibration can be performed as often as desired.

## Motor calibration

The first question that comes to mind is what values should be stored in the look up table.  The simplest implementation is to go though the duty cycle of each motor from 1 to 100 forward and reverse and record the encoder counts per unit time.  This info is then stored in a look up table.

Once this has been done, if a certain speed is requested, the speed can be converted into encoder ticks per second, the look up table searched for the nearest encoder ticks per second value and the duty cycle applied.

https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/nvs_flash.html




## To do list

1. Send message to start calibration.  The ROS self test message is suitable. However, when tested only one service at a time works with micro-ROS, so using a subscriber and publisher instead.
1. Run tests several times, capturing results for analysis to improve what is done.
1. Test various implementations.
1.

The motors being used are similar to this: https://www.aliexpress.com/item/32434622843.html

Description:
Operating voltage: 3V~12VDC (recommended operating voltage of about 6 to 8V)
Maximum torque: 800gf cm min (3V)
No-load speed: 1:48 (3V time)
The load current: 70mA (250mA MAX) (3V)
Size: 7x2.2x1.8cm(approx)

