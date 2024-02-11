#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "string.h"
#include "stdio.h"

const uart_port_t uart_num = UART_NUM_0;
int baud = 115200;
#define TXD_PIN (GPIO_NUM_1)
#define RXD_PIN (GPIO_NUM_3)

// Settings
static const uint8_t msg_queue_len = 5;

// Globals
static QueueHandle_t msg_queue;

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

// ---- RTOS PART 5 CHALLENGE --------------------------------
#define BUILT_IN_LED 2
static const uint8_t buf_len = 255;     // Size of buffer to look for command
static const int delay_queue_len = 5;   // Size of delay_queue
static const int msg_queue_len = 5;     // Size of msg_queue
static QueueHandle_t delay_queue;
static QueueHandle_t msg_queue;
static const char command[] = "delay "; // Note the space!
static const uint8_t blink_max = 100;   // Num times to blink before message

// Message struct: used to wrap strings (not necessary, but it's useful to see
// how to use structs here)
typedef struct Message {
  char body[20];
  int count;
} Message;
// Task: command line interface (CLI)
void taskA(void *parameters){
    Message rcv_msg;
    char c;
    char buf[buf_len];
    uint8_t idx = 0;    
    uint8_t cmd_len = strlen(command);
    int led_delay;

    // Clear whole buffer
    memset(buf, 0, buf_len);

      // Loop forever
    while (1) {
        // See if there's a message in the queue (do not block)
        if (xQueueReceive(msg_queue, (void *)&rcv_msg, 0) == pdTRUE) {
            printf("Msg body: %s, Msg Count: %s\r\n", rcv_msg.body, rcv_msg.count);
        }

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
                // Print newline to terminal
                printf("\r\n");

                // Check if the first 6 characters are "delay "
                if (memcmp(buf, command, cmd_len) == 0) {
                    // Convert last part to positive integer (negative int crashes)
                    //char tail_str[4];
                    //strncpy ( tail_str, command+cmd_len, sizeof(tail_str));
                    //tail_str[4] = '\0';

                    // Extract the substring after "delay " (sets a pointer to the position in the string after "delay ")
                    char* tail = buf + cmd_len;
                    led_delay = atoi(tail);
                    led_delay = abs(led_delay);

                    // Send integer to other task via queue
                    if (xQueueSend(delay_queue, (void *)&led_delay, 10) != pdTRUE) {
                        printf("ERROR: Could not put item on delay queue.");
                    }
                }

                // Reset receive buffer and index counter
                memset(buf, 0, buf_len);
                idx = 0;
            // Otherwise, echo character back to serial terminal    
            }else{
                putchar(c);
                fflush(stdout);
            }

        }
        
    }

}

// Task: flash LED based on delay provided, notify other task every 100 blinks
void taskB(void *parameters){
    Message msg;
    int led_delay = 500;
    uint8_t counter = 0;

    // Loop forever
    while (1) {

        // See if there's a message in the queue (do not block)
        if (xQueueReceive(delay_queue, (void *)&led_delay, 0) == pdTRUE) {

            // Best practice: use only one task to manage serial comms
            strcpy(msg.body, "Message received ");
            msg.count = 1;
            xQueueSend(msg_queue, (void *)&msg, 10);
        }

        // Blink
        gpio_set_level(BUILT_IN_LED, 1);        // Set high level
        vTaskDelay(led_delay / portTICK_PERIOD_MS);
        gpio_set_level(BUILT_IN_LED, 0);        // Set low level
        vTaskDelay(led_delay / portTICK_PERIOD_MS);

        // If we've blinked 100 times, send a message to the other task
        counter++;
        if (counter >= blink_max) {
        
            // Construct message and send
            strcpy(msg.body, "Blinked: ");
            msg.count = counter;
            xQueueSend(msg_queue, (void *)&msg, 10);

            // Reset counter
            counter = 0;
        }


    }
}
// -----------------------------------------------------------

// *****************************************************
// Tasks

// Wait for item on queue and print it
void printMessages(void *parameters){

    int item;

    // Loop forever
    while(1){
        // See if there's a message in the queue (do not block)
        // 1st param - handle to the queue
        // 2nd param - address where the queue item will be copied to. Address is casted into void pointer (not necessary)
        // 3rd param - TimeOut in number of ticks to wait for something to appear in the queue
        // returs - pdTRUE if something was read from the queue. pdFALSE otherwise
        // Reads an item from msg_queue and copied it into item. 
        if (xQueueReceive(msg_queue, (void*)&item, 0) == pdTRUE){
            // print to the console
            //printf("\r\n%d", item);
        }

        printf("\r\n%d", item);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main() {
    UART_init();

    /* Reset an gpio to default state (enable pullup, disable
       interrupts, ...) */
    gpio_reset_pin(BUILT_IN_LED);
    // Set output
    gpio_set_direction(BUILT_IN_LED, GPIO_MODE_OUTPUT);

    printf("\r\n --- FreeRTOS Queue Demo --- \r\n");

    // Create Queue
    msg_queue = xQueueCreate(msg_queue_len, sizeof(int));

    // Start print task
    xTaskCreate(printMessages, "printMessages", 4096, NULL, 1, NULL);

    while(1){
        static int num = 0;

        // Try to add number to the queue for 10 ticks, fail if queue is full
        if (xQueueSend(msg_queue, (void*)&num, 10) != pdTRUE){
            printf("\r\nQueue full!");
        }

        num++;

        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Generally, it is a good practice to assign one hardware peripheral per task. 
        // The same task should handle all serial inputs and outputs, and don't have serial commands in other tasks
        // In these series, for demo purposes UART is shared between tasks
         

    }
}