#include "stdio.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

const uart_port_t uart_num = UART_NUM_0;
int baud = 115200;
#define TXD_PIN (GPIO_NUM_1)
#define RXD_PIN (GPIO_NUM_3)

// Settings
static const uint8_t buf_len = 255;

// Globals
static char *msg_ptr = NULL;
static volatile uint8_t msg_flag = 0;

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

// Task: read message from Serial buffer
void readSerial(void *parameters) {
  char c;
  char buf[buf_len];
  uint8_t idx = 0;

  // Clear whole buffer
  memset(buf, 0, buf_len);
  
  // Loop forever
  while (1) {
    // Read cahracters from serial
    int len = uart_read_bytes(uart_num, &c, 1, portMAX_DELAY);
    if (len > 0) {
        if (c == '\n') {
            buf[idx] = '\0';  // Null-terminate the string

            // Try to allocate memory and copy over message. If message buffer is
            // still in use, ignore the entire message.
            if (msg_flag == 0) {
                msg_ptr = (char *)pvPortMalloc(idx * sizeof(char));

                // If malloc returns 0 (out of memory), throw an error and reset
                configASSERT(msg_ptr);

                // Copy message
                memcpy(msg_ptr, buf, idx);

                // Notify other task that message is ready
                msg_flag = 1;

                printf("\r\nFlag set!\r\n");
            }

            // Reset receive buffer and index counter
            memset(buf, 0, buf_len);
            idx = 0;


        } else if (c != '\r') {  // Ignore carriage return
            putchar(c);
            fflush(stdout);
            // Store received character to buffer if not over buffer limit
            if (idx < buf_len - 1) {
                buf[idx] = c;
                idx++;
            }
        }
    }
  }    
}

// Task: print message whenever flag is set and free buffer
void printMessage(void *parameters) {
  while (1) {

    // Wait for flag to be set and print message
    if (msg_flag == 1) {
        printf("%s\r\n", msg_ptr);

        // Give amount of free heap memory (uncomment if you'd like to see it)
        printf("Free heap (bytes): %d\r\n", xPortGetFreeHeapSize());

        // Free buffer, set pointer to null, and clear flag
        vPortFree(msg_ptr);
        msg_ptr = NULL;
        msg_flag = 0;
    }
  }
}

// Task : perfom some mundane task
void testTask(void *parameter){
    while(1){
        int a = 1;
        int b[100]; // 400 bytes of memory. Aroung 768 bytes are task overhead

        // do something with array so it's not optimised out by the compiler
        for (int i = 0; i < 100; i++){
            b[i] = a + 1;
        }
        putchar(b[0]);
        fflush(stdout);
        printf("\r\n");

        // Print out remaining stack memory (word = 4 bytes)
        printf("High water mark (words): %d\r\n", uxTaskGetStackHighWaterMark(NULL));
        
        // Print out number of free heap memory bytes before malloc
        printf("Heap before malloc (bytes): %d\r\n", xPortGetFreeHeapSize());
 
        // Thread-safe malloc operation
        int *ptr = (int*)pvPortMalloc(1024 * sizeof(int));

        // One way to prevent heap overflow is to check the malloc output
        // checks that memory was allocated
        if (ptr == NULL){
            printf("Not enough heap!\r\n");
        }else{
            // do something with array so it's not optimised out by the compiler
            for (int i = 0; i < 1024; i++){
                ptr[i] = 3;
            }
        }



        // Print out number of free heap memory bytes before malloc
        printf("Heap after malloc (bytes): %d\r\n", xPortGetFreeHeapSize()); 

        // Free up our allocated memory
        vPortFree(ptr);
        
        // small delay
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

//*****************************************************************************
// Main (runs as its own task with priority 1 on core 1)

void app_main() {
    UART_init();

    printf("\r\n--- Program Start ---\r\n");

    //xTaskCreate(testTask, "test task", 4096, NULL, 1, NULL);
    xTaskCreate(readSerial, "read Serial", 4096, NULL, 1, NULL);
    xTaskCreate(printMessage, "print Message", 4096, NULL, 0, NULL);

    // Delete main app
    //vTaskDelete(NULL);

}