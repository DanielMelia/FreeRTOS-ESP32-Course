#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
//#include "FreeRTOSConfig.h"

#include "driver/uart.h"
#include "driver/gpio.h"

//#define configTIMER_TASK_STACK_DEPTH 4096
//#define CONFIG_FREERTOS_TIMER_TASK_STACK_DEPTH 4096


// NOTE: To use timers you need to go to FreeRTOSConfig.h and set:
// #define configUSE_TIMERS                1
//In our case it was already set by default
//#include <FreeRTOSConfig.h>

const uart_port_t uart_num = UART_NUM_0;
int baud = 115200;
#define TXD_PIN (GPIO_NUM_1)
#define RXD_PIN (GPIO_NUM_3)

void UART_init(void){
   const uart_config_t uart_config = {
      .baud_rate = baud,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB
   };

   // Configure UART parameters
   ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

   // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
   ESP_ERROR_CHECK(uart_set_pin(uart_num, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

   // Setup UART buffered IO with event queue
   const int uart_buffer_size = (1024 * 2);
   //QueueHandle_t uart_queue;
   // Install UART driver using an event queue here
   //ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
   ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, 0, 0, NULL, 0));

}

static TimerHandle_t one_shot_timer = NULL;
static TimerHandle_t auto_reload_timer = NULL;

// **********************************************
// CALLBACKS

// called when one of the timers expires
void myTimerCallback(TimerHandle_t xTimer){

    // Print message if timer 0 expired
    if((uint32_t)pvTimerGetTimerID(xTimer) == 0){
        printf("One-Shot Timer Expired!\r\n");
    }
    
    // Print message if timer 1 expired
    if((uint32_t)pvTimerGetTimerID(xTimer) == 1){
        printf("Auto-reload Timer Expired!\r\n");
    }
}

void app_main() {
    UART_init();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("\r\n --- FreeRTOS Timer Demo! ---\r\n");

    // Create a one-shot timer
    one_shot_timer = xTimerCreate(
        "one shot timer",           // name of timer
        2000 / portTICK_PERIOD_MS,  // period of timer in ticks (1ms minimum)
        pdFALSE,                    // Auto-reload (set FALSE to execute onyl once)
        (void *)0,                  // Timer ID
        myTimerCallback             // Callback function
    );

    // Create a one-shot timer
    auto_reload_timer = xTimerCreate(
        "auto reload timer",           // name of timer
        1000 / portTICK_PERIOD_MS,  // period of timer in ticks (1ms minimum)
        pdTRUE,                    // Auto-reload (set FALSE to execute onyl once)
        (void *)1,                  // Timer ID
        myTimerCallback             // Callback function
    );

    // Check to make sure timers was created
    if (one_shot_timer == NULL || auto_reload_timer == NULL){
        printf("Could not create one of the timers!!\r\n");
    }

    // Wait and then print out a message that we're starting the timer
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("Starting timer...\r\n");

    // Start timers (max block time if command queue is full)
    xTimerStart(one_shot_timer, portMAX_DELAY);
    xTimerStart(auto_reload_timer, portMAX_DELAY);

    while(1){}
}