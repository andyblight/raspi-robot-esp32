#include <stdio.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_uros/options.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include <raspi_robot_msgs/msg/leds.h>
#include <raspi_robot_msgs/msg/motors.h>
// #include <raspi_rover_msgs/msg/status.h>


#include "raspi_rover_driver.h"

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

// Tick definitions.
#define TICK_RATE_HZ (10)
#define MS_PER_TICK (1000 / TICK_RATE_HZ)
#define US_PER_TICK (MS_PER_TICK * 1000)

// Number fo executor handles: one timer, two subscribers.
#define EXECUTOR_HANDLE_COUNT (3)

rcl_publisher_t publisher_status;
rcl_subscription_t subscriber_leds;
rcl_subscription_t subscriber_motors;

// Logging name.
static const char *TAG = "raspi_rover";

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        // status_t status;
        // raspi_rover_get_status(&status);
        // raspi_rover_msgs__msg__Status msg;
        // msg.switch1 = status.switch1;
        // msg.switch2 = status.switch2;
        // msg.sonar_mm = status.sonar_mm;
        // RCSOFTCHECK(rcl_publish(&publisher_status, &msg, NULL));
        // ESP_LOGI(TAG, "Sent msg: %d, %d, %d", msg.switch1, msg.switch2, msg.sonar_mm);
    }
}

static void subscription_callback_leds(const void *msgin)
{
    const pipebot_msgs__msg__Leds *msg = (const pipebot_msgs__msg__Leds *)msgin;
    ESP_LOGI(TAG, "Received: %d, %d", msg->led, msg->flash_rate);
    raspi_rover_set_led(msg->led, msg->flash_rate);
}

static void subscription_callback_motors(const void *msgin)
{
    const raspi_rover_msgs__msg__Motors *msg = (const raspi_rover_msgs__msg__Motors *)msgin;
    ESP_LOGI(TAG, "Received: %d, %d, %d", msg->left, msg->right, msg->duration_ms);
    // Call motor functions.
    uint16_t ticks = msg->duration_ms / MS_PER_TICK;
    raspi_rover_motors_drive(msg->left, msg->right, ticks);
}

void appMain(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Create node
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "raspi_rover", "", &support));

    // Create status publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher_status,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(raspi_rover_msgs, msg, Status),
        "raspi_rover_status_p"));

    // Create subscribers.
    RCCHECK(rclc_subscription_init_default(
        &subscriber_leds,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(pipebot_msgs, msg, Leds),
        "raspi_rover_leds_s"));

    RCCHECK(rclc_subscription_init_default(
        &subscriber_motors,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(raspi_rover_msgs, msg, Motors),
        "raspi_rover_motors_s"));

    // Create timer.
    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // create executor
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, EXECUTOR_HANDLE_COUNT, &allocator));

    unsigned int rcl_wait_timeout = 1000; // in ms
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    pipebot_msgs__msg__Leds leds_msg;
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_leds, &leds_msg, &subscription_callback_leds, ON_NEW_DATA));
    raspi_rover_msgs__msg__Motors motors_msg;
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_motors, &motors_msg, &subscription_callback_motors, ON_NEW_DATA));

    raspi_rover_init();
    raspi_rover_set_led(raspi_rover_LED_ESP_BLUE, raspi_rover_LED_FLASH_2HZ);

    while (1)
    {
        rclc_executor_spin_some(&executor, 100);
        usleep(US_PER_TICK);
        raspi_rover_tick();
    }

    // free resources
    RCCHECK(rcl_subscription_fini(&subscriber_motors, &node));
    RCCHECK(rcl_subscription_fini(&subscriber_leds, &node));
    RCCHECK(rcl_publisher_fini(&publisher_status, &node))
    RCCHECK(rcl_node_fini(&node))

    vTaskDelete(NULL);
}
