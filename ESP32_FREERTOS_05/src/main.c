#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "driver/gpio.h"

const uart_port_t uart_num = UART_NUM_0;
int baud = 115200;
#define TXD_PIN (GPIO_NUM_1)
#define RXD_PIN (GPIO_NUM_3)

static int shared_var = 0;
static SemaphoreHandle_t mutex; // FreeRTOS generalises semaphores and mutex as just semaphores

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

void incTask(void *parameters){
	int local_var;
	
	while(1){

        // Take mutex before entering a critical code section
        if(xSemaphoreTake(mutex, 0) == pdTRUE){ // second parameter is timeout in ticks
        //if the function cannot obtain the mutex in that many ticks it will return pdFalse
        // 0 timeout creates a non-blocking task

            local_var = shared_var;
            local_var++;
            vTaskDelay(200 / portTICK_PERIOD_MS);
            shared_var = local_var;

            // Give mutex after critical section
            xSemaphoreGive(mutex);

            printf("%d\r\n",shared_var);
        }else{
            // Do something else
        }
	}
}

void app_main() {
    UART_init();

    // Create semaphore before starting task
    mutex = xSemaphoreCreateMutex();

    // Start task 1
    xTaskCreate(incTask, "Increment task 1", 1024, NULL, 1, NULL);
    // Start task 2
    xTaskCreate(incTask, "Increment task 2", 1024, NULL, 1, NULL);

}