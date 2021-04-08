/*
Useful information for the GPIO functions was found here:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html
*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"


#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "raspi_robot_sonar.h"

// General defines.
#define ESP_INTR_FLAG_DEFAULT 0
#define MAX_TIMEOUTS (5)

// Logging name.
static const char *TAG = "raspi_robot_sonar";

// GPIO pins.
static uint8_t trigger_pin = 0;
static uint8_t echo_pin = 0;

// Debugging vars.
static int debug_ping_count = 0;
static int debug_timeout_count = 0;

// Queue variables and types.
// 80ms queue timeout.
const TickType_t QUEUE_TIMEOUT_TICKS = pdMS_TO_TICKS(80);
typedef enum {
    EVENT_ISR_STOP = 0,
    EVENT_ISR_START = 1,
    EVENT_PING
} event_types_t;
typedef struct {
    int64_t time_us;
    event_types_t event;
} isr_event_t;
static xQueueHandle event_queue = NULL;
// Mutex protects distance_mm.
static const TickType_t MUTEX_TIMEOUT_TICKS = (TickType_t)10;
static SemaphoreHandle_t distance_mutex = NULL;
static uint16_t distance_mm = 0;

// Task variables.
TaskHandle_t *pxCreatedTask = NULL;


static void IRAM_ATTR sonar_isr_handler(void* arg)
{
    // Record time.
    isr_event_t event;
    event.time_us = esp_timer_get_time();
    // Work out which edge.
    int gpio_level = gpio_get_level(echo_pin);
    if (gpio_level == 1) {
        event.event = EVENT_ISR_START;
    } else {
        event.event = EVENT_ISR_STOP;
    }
    // Write values to queue.
    xQueueSendToBackFromISR(event_queue, &event, NULL);
}

static void configure_trigger_gpio(uint8_t gpio_pin)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = BIT64(gpio_pin),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    // Configure GPIO.
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    trigger_pin = gpio_pin;
    gpio_set_level(trigger_pin, 0);
}

static void configure_echo_gpio(uint8_t gpio_pin)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = BIT64(gpio_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 0,
        .pull_up_en = 1,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    // Configure GPIOs.
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    // Add ISR.
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(gpio_pin, sonar_isr_handler, (void*) &gpio_pin);
    echo_pin = gpio_pin;
}

static void send_ping()
{
    // Trigger pulse.  Needs >10us.
    int64_t esp_time_us = esp_timer_get_time();
    int64_t trigger_end_time_us = esp_time_us + 10;
    gpio_set_level(trigger_pin, 1);
    // 10us delay to trigger pulse.
    do {
        esp_time_us = esp_timer_get_time();
    } while (esp_time_us < trigger_end_time_us);
    gpio_set_level(trigger_pin, 0);
    // 100us delay to allow for ping to be sent.
    trigger_end_time_us = esp_time_us + 100;
    do {
        esp_time_us = esp_timer_get_time();
    } while (esp_time_us < trigger_end_time_us);
    // ESP_LOGI(TAG, "Trigger sent");
}

static void set_distance(int64_t start_us, int64_t end_us)
{
    // Calculate distance using formula:
    // distance (m) = (elapsed time (s) x speed of sound (343m/s)) / 2s.
    const int32_t speed_of_sound_m_s = 343;
    int32_t elapsed_time_us = end_us - start_us;
    uint16_t sonar_distance_mm = (elapsed_time_us * speed_of_sound_m_s) / 2000;
    // ESP_LOGI(TAG, "elapsed_time_us %dus", elapsed_time_us);
    if (distance_mutex) {
        BaseType_t mutex_obtained = xSemaphoreTake(distance_mutex, MUTEX_TIMEOUT_TICKS);
        if (mutex_obtained == pdTRUE) {
            distance_mm = sonar_distance_mm;
            xSemaphoreGive(distance_mutex);
        }
    }
    // ESP_LOGI(TAG, "Nearest object %dmm", sonar_distance_mm);
}

static void sonar_task(void * arg)
{
    ESP_LOGI(TAG, "Sonar task started");
    BaseType_t queue_result = pdFALSE;
    isr_event_t event;
    int64_t start_time_us = 0;
    bool ping_sent = false;
    // This is used to detect when the sonar has malfunctioned.
    // If the count exceeds a threshold, the distance is set to 0mm.
    int timeout_count = 0;
    while (1) {
        queue_result = xQueueReceive(event_queue, &event, QUEUE_TIMEOUT_TICKS);
        if (queue_result == pdTRUE) {
            // ESP_LOGI(TAG, "Sonar task event %lld", event.time_us);
            // { // Logging
            //     int event_type = 0;
            //     if (event.event == EVENT_PING) event_type = 2;
            //     if (event.event == EVENT_ISR_START) event_type = 1;
            //     ESP_LOGI(TAG, "Sonar task event %d, %lld",
            //         event_type, event.time_us);
            // }
            if (event.event == EVENT_PING) {
                start_time_us = 0;
                send_ping();
                ping_sent = true;
                ++debug_ping_count;
            } else {
                if (ping_sent) {
                    if (event.event == EVENT_ISR_START) {
                        // Record start time.
                        start_time_us = event.time_us;
                    }
                    if (event.event == EVENT_ISR_STOP && start_time_us != 0) {
                        set_distance(start_time_us, event.time_us);
                        start_time_us = 0;
                        ping_sent = false;
                        timeout_count = 0;
                    }
                }
                // Else ignore.
            }
        } else {
            if (ping_sent) {
                // Timeout occurred.
                start_time_us = 0;
                ping_sent = false;
                ++debug_timeout_count;
                // ESP_LOGI(TAG, "Debug: sent %d, timeouts %d", debug_ping_count, debug_timeout_count);
                ++timeout_count;
                if (timeout_count > MAX_TIMEOUTS) {
                    set_distance(0, 0);
                }
            }
            // else do nothing as waiting for ping.
        }
    }
}

void sonar_init(uint8_t output_pin, uint8_t input_pin)
{
    configure_trigger_gpio(output_pin);
    configure_echo_gpio(input_pin);
    // Create queue, mutex and task to handle process the ping requests and the ISR events.
    event_queue = xQueueCreate(10, sizeof(isr_event_t));
    if (!event_queue) {
        ESP_LOGE(TAG, "No sonar queue");
    } else {
        distance_mutex = xSemaphoreCreateMutex();
        if (distance_mutex) {
            BaseType_t xReturned = xTaskCreate(sonar_task, "sonar_task", 6*2048, NULL, 5, pxCreatedTask);
            if (xReturned == pdPASS) {
                ESP_LOGI(TAG, "Sonar initialised");
            } else {
                ESP_LOGE(TAG, "No sonar task");
            }
        } else {
            ESP_LOGE(TAG, "No sonar mutex");
        }
    }
}

void sonar_term(void)
{
    vTaskDelete(pxCreatedTask);
}

void sonar_tick()
{
    // Set quite fast so that any timeouts have minimal effect.
    const int sonar_tick_interval = 2;
    static int counter = 0;
    ++counter;
    // Once per interval.
    if (counter % sonar_tick_interval == 0) {
        // Queue a ping request.
        isr_event_t event;
        event.time_us = esp_timer_get_time();
        event.event = EVENT_PING;
        xQueueSendToBackFromISR(event_queue, &event, NULL);
    }
}

uint16_t sonar_get()
{
    uint16_t result = 0;
    if (distance_mutex) {
        BaseType_t mutex_obtained = xSemaphoreTake(distance_mutex, MUTEX_TIMEOUT_TICKS);
        if (mutex_obtained == pdTRUE) {
            result = distance_mm;
            xSemaphoreGive(distance_mutex);
        }
    }
    return result;
}
