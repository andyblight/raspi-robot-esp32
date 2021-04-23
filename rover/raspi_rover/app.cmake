# This file sets the variables UROS_APP_INCLUDES and UROS_APP_SRCS

# Set paths to include directories.
set(UROS_APP_INCLUDES
  ${UROS_APP_FOLDER}/raspi_robot_driver/include
)

# Set paths to files you want to build.
set(UROS_APP_SRCS
  ${UROS_APP_FOLDER}/app.c
  # ${UROS_APP_FOLDER}/diagnostics.c
  ${UROS_APP_FOLDER}/raspi_robot_driver/src/raspi_robot_adc.c
  ${UROS_APP_FOLDER}/raspi_robot_driver/src/raspi_robot_driver.c
  ${UROS_APP_FOLDER}/raspi_robot_driver/src/raspi_robot_leds.c
  ${UROS_APP_FOLDER}/raspi_robot_driver/src/raspi_robot_motors.c
  # ${UROS_APP_FOLDER}/raspi_robot_driver/src/raspi_robot_servo.c
  ${UROS_APP_FOLDER}/raspi_robot_driver/src/raspi_robot_sonar.c
  ${UROS_APP_FOLDER}/raspi_robot_driver/src/raspi_robot_switches.c
)

# message("AJB: INCLUDES: " ${UROS_APP_INCLUDES})
# message("AJB: SRCS: " ${UROS_APP_SRCS})
