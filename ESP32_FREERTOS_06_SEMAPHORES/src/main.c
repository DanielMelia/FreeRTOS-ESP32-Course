#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "string.h"
#include "stdio.h"

const uart_port_t uart_num = UART_NUM_0;
int baud = 115200;
#define TXD_PIN (GPIO_NUM_1)
#define RXD_PIN (GPIO_NUM_3)

#define BUILT_IN_LED 2

static SemaphoreHandle_t bin_sem;
// static SemaphoreHandle_t mutex;

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

//*****************************************************************************
// Tasks

// Blink LED based on rate passed by parameter
void blinkLED(void *parameters) {

  // Copy the parameter into a local variable
  int num = *(int *)parameters;

  // Release semaphore so that the creating function can finish
  xSemaphoreGive(bin_sem);  // add 1 to semaphore
  // xSemaphoreGive(mutex);

  // Print the parameter
  printf("\r\nReceived: %d\r\n", num);

    /* Reset an gpio to default state (enable pullup, disable
       interrupts, ...) */
    gpio_reset_pin(BUILT_IN_LED);
    // Set output
    gpio_set_direction(BUILT_IN_LED, GPIO_MODE_OUTPUT);

  // Blink forever and ever
  while (1) {
        gpio_set_level(BUILT_IN_LED, 1);        // Set high level
        vTaskDelay(num / portTICK_PERIOD_MS);
        gpio_set_level(BUILT_IN_LED, 0);        // Set low level
        vTaskDelay(num / portTICK_PERIOD_MS);
  }
}

void app_main() {
    UART_init();
    long int delay_arg;

    // Wait a moment to start (so we don't miss Serial output)
    vTaskDelay(1000 / portTICK_PERIOD_MS);    

    printf("\r\n---FreeRTOS Mutex Challenge---\r\n");
    printf("Enter a number for delay (milliseconds): ");
    fflush(stdout);

    char c;
    uint8_t buf_len = 10;
    char buf[buf_len];
    uint8_t idx = 0;    

    // Clear whole buffer
    memset(buf, 0, buf_len);

    while (1) {
        // Read characters from serial
        int len = uart_read_bytes(uart_num, &c, 1, portMAX_DELAY);
        if (len > 0) {
            // Store received character to buffer if not over buffer limit
            if (idx < buf_len - 1) {
                buf[idx] = c;
                idx++;
            }
            // Print newline and check input on 'enter'
            if ((c == '\n') || (c == '\r')) {
                buf[idx] = '\0';  // Null-terminate the string
                delay_arg = atoi(buf);
                break;
            }else{
                putchar(c);
                fflush(stdout);
            }           
        }
    }

    // Create mutex before starting tasks
    //mutex = xSemaphoreCreateMutex();

    // Take the mutex
    //xSemaphoreTake(mutex, portMAX_DELAY);

    // Create binary semaphore before starting tasks
    bin_sem = xSemaphoreCreateBinary(); // initialises to 0 by default
    // take semaphore -> decreases semaphore value
    // give semaphore -> increases semaphore value

    // Start task 1
    xTaskCreate(blinkLED, "Blink LED", 4096, (void *)&delay_arg, 1, NULL);

    // Do nothing until the binary semaphore has been returned
    xSemaphoreTake(bin_sem, portMAX_DELAY);
    //xSemaphoreTake(mutex, portMAX_DELAY);

    printf("Done!\r\n");

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);  
    }
    

}