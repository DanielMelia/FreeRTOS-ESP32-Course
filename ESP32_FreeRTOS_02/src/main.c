#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "string.h"

// SOURCE: https://www.youtube.com/watch?v=5JcMtbA9QEE&list=PLEBQazB0HUyQ4hAPU1cJED6t3DU0h34bz&index=7


// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

const uart_port_t uart_num = UART_NUM_0;
int baud = 115200;
#define TXD_PIN (GPIO_NUM_1)
#define RXD_PIN (GPIO_NUM_3)
void UART_init(void);

#define BUILT_IN_LED 2
// Some string to print
const char msg[] = "Barkadeer brig Arr booty rum.";
int blink_delay = 500;

//*******************************************************************
// Tasks

// Task: print to Serial Terminal with lower priority
void startTask1(void *parameter){
    // Count number of characters in string
    //int msg_len = strlen(msg);
    char reply[25];
    int index = 0;
    // Print string to terminal
    while(1){
        printf("\r\nEnter blink delay in ms: ");
        fflush(stdout);  // Flush the output buffer to ensure the message is displayed

         while(1){
            uint8_t ch;
            int len = uart_read_bytes(uart_num, &ch, 1, portMAX_DELAY);
            if (len > 0) {
                  if (ch == '\n') {
                     reply[index] = '\0';  // Null-terminate the string
                     blink_delay = atoi(reply);
                     break;
                  }
                else if (ch != '\r'){
                    reply[index] = ch;
                    index++;
                    putchar(ch);
                    fflush(stdout);  // Flush the output buffer
                }
            }
         }
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // for(int i = 0; i < msg_len; i++){
        //     putchar(msg[i]);
        //     fflush(stdout);
        // }
        // printf("\r\n");
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
}

// Task 2: print to serial terminal with higher priority

void startTask2(void *parameter){

    while(1){
      gpio_set_level(BUILT_IN_LED, 1);        // Set high level
      vTaskDelay(blink_delay / portTICK_PERIOD_MS);   // Delay 500 milliseconds
      gpio_set_level(BUILT_IN_LED, 0);        // Set low level
      vTaskDelay(blink_delay / portTICK_PERIOD_MS);   // Delay 500 milliseconds
    }

    // while(1){
    //     putchar('*');
    //     fflush(stdout);   
    //     vTaskDelay(100 / portTICK_PERIOD_MS);     
    // }
}

// void serialPrintTask(void *parameter){
//     while(1){
//         printf("Test\r\n");
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
// }

// Task handles (to allow control of these tasks from a 3rd task)
static TaskHandle_t task_1 = NULL;
static TaskHandle_t task_2 = NULL;

void app_main() {
    UART_init();
    /* Reset an gpio to default state (enable pullup, disable
       interrupts, ...) */
    gpio_reset_pin(BUILT_IN_LED);
    // Set output
    gpio_set_direction(BUILT_IN_LED, GPIO_MODE_OUTPUT);



    printf("\r\n--- FreeRTOS Task Demo ---\r\n");


    xTaskCreatePinnedToCore(    // Use xTaskCreate() in vanilla FreeRTOS (you don't specify the core)
        startTask2,      // Function to be called
        "task 2",   // Name of task
        1024,           // Stack size (bytes in ESP32, words in FreeRTOS) 768bytes is the smallest
        NULL,           // Parameter to pass to function
        2,              // Higher the number the higher the priority (0 to configMAX_PRIORITIES - 1)
        &task_2,           // Task Handle
        app_cpu         // Run on one core for demo purposes (ESP32 only)
    ); 

    //char c;
    //char msgIn[25];
    //int idx = 0;

    // while(1){
    //     char reply[25];
    //     int index = 0;
    //     printf("\r\nEnter blink delay in ms: ");
    //     fflush(stdout);


    //      while(1){
    //         uint8_t ch;
    //         int len = uart_read_bytes(uart_num, &ch, 1, portMAX_DELAY);
    //         if (len > 0) {
    //               if (ch == '\n') {
    //                 reply[index] = '\0';  // Null-terminate the string
    //                 printf("\r\nHello, %s!\n", reply);
    //                 blink_delay = atoi(reply);
    //                 printf("\r\nDelay: %d!\n", blink_delay);
    //                 break;
    //               }
    //             else if (ch != '\r'){
    //                 reply[index] = ch;
    //                 index++;
    //                 putchar(ch);
    //                 fflush(stdout);  // Flush the output buffer
    //             }
    //         }
    //      }


    //     // while(1){           
    //     //     c = getchar();
    //     //     if (c > 20 && c < 120){
    //     //         if (c == '\n') {
    //     //             msgIn[idx] = '\0';  // Null-terminate the string
    //     //             printf(msgIn);
    //     //             printf("\r\n");
    //     //             break;
    //     //         }
    //     //         else if (c != '\r'){
    //     //             msgIn[idx] = c;
    //     //             idx++;
    //     //             putchar(c);
    //     //             fflush(stdout);  // Flush the output buffer
    //     //         }            
    //     //         vTaskDelay(50 / portTICK_PERIOD_MS);
    //     //     }

    //     // }
    //     //scanf("%d", &blink_delay);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }


    xTaskCreatePinnedToCore(    // Use xTaskCreate() in vanilla FreeRTOS (you don't specify the core)
        startTask1,      // Function to be called
        "task 1",   // Name of task
        4096,           // Stack size (bytes in ESP32, words in FreeRTOS) 768bytes is the smallest
        NULL,           // Parameter to pass to function
        10,              // Higher the number the higher the priority (0 to configMAX_PRIORITIES - 1)
        &task_1,           // Task Handle
        app_cpu         // Run on one core for demo purposes (ESP32 only)
    );    

  

    // while(1){
    //     // Suspend the higher priority tasks for some intervals
    //     for(int i = 0; i < 3; i++){
    //         vTaskSuspend(task_2); // control task using handle
    //         vTaskDelay(1000 / portTICK_PERIOD_MS);
    //         vTaskResume(task_2); // control task using handle
    //         vTaskDelay(1000 / portTICK_PERIOD_MS);
    //     }

    //     // Delete the lower priority task (IMPORTANT to check that the task exists, and then delete it)
    //     if (task_1 != NULL){
    //         vTaskDelete(task_1);
    //         task_1 = NULL;
    //     }
    // }

    // xTaskCreatePinnedToCore(    // Use xTaskCreate() in vanilla FreeRTOS (you don't specify the core)
    //     serialPrintTask,      // Function to be called
    //     "serial print",   // Name of task
    //     1024,           // Stack size (bytes in ESP32, words in FreeRTOS) 768bytes is the smallest
    //     NULL,           // Parameter to pass to function
    //     1,              // Higher the number the higher the priority (0 to configMAX_PRIORITIES - 1)
    //     NULL,           // Task Handle
    //     app_cpu         // Run on one core for demo purposes (ESP32 only)
    // );
}

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