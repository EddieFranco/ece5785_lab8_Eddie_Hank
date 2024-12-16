#include <can2040.h>
#include <hardware/regs/intctrl.h>
#include <stdio.h>
#include <pico/stdlib.h>
#include "pico/time.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#define MAIN_TASK_PRIORITY      ( tskIDLE_PRIORITY + 1UL )
#define BLINK_TASK_PRIORITY     ( tskIDLE_PRIORITY + 2UL )
#define MAIN_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define BLINK_TASK_STACK_SIZE configMINIMAL_STACK_SIZE



QueueHandle_t msgs;
static struct can2040 cbus;

//callback that runs when a CAN message is received or another CAN event occurs.
// cd: Pointer to the CAN object (cbus), giving access to CAN state.
// notify: Contains notification flags for specific CAN events, such as received messages or errors.
// msg: Structure holding the actual CAN message, which includes fields like message ID and data.
static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
     xQueueSendToBack(msgs, msg, portMAX_DELAY);
}


// This interrupt handler is triggered when a CAN interrupt occurs.
static void PIOx_IRQHandler(void)
{
    can2040_pio_irq_handler(&cbus); //It calls can2040_pio_irq_handler, which processes the interrupt and handles CAN events in the cbus object.
}


// Example CAN message to transmit
struct can2040_msg message = {
    .id = 0x12,         // CAN ID
    .dlc = 4,          // Data length code
    .data = {'H', 'o'}   // Example data
};


void transmit_can(void)
{
    int transmit_status;

        // Attempt to queue the message for transmission
        transmit_status = can2040_transmit(&cbus, &message);
        
        if (transmit_status == 0) {
            printf("Message scheduled for transmission\n");
        } else {
            printf("Transmission queue is full\n");
        }
    
}


// This function is where the CAN bus is initialized and configured.
void canbus_setup(void)
{
    uint32_t pio_num = 0;
    uint32_t sys_clock = 125000000, bitrate = 500000;
    uint32_t gpio_rx = 4, gpio_tx = 5;

    // Setup canbus
    can2040_setup(&cbus, pio_num); //configures cbus with pio_num, The pio_num should be either 0 or 1 to use either the PIO0 or PIO1 rp2040 hardware block
    can2040_callback_config(&cbus, can2040_cb); // Register Callback, links cbus to the can2040_cb callback for handling received messages.

    // Enable irqs
    irq_set_exclusive_handler(PIO0_IRQ_0, PIOx_IRQHandler); //  Assigns the interrupt handler PIOx_IRQHandler to PIO0_IRQ_0 (the interrupt for PIO0)
    irq_set_priority(PIO0_IRQ_0, PICO_DEFAULT_IRQ_PRIORITY - 1); // Sets the priority of PIO0_IRQ_0 just above the default, so it can respond quickly without overriding critical tasks
    irq_set_enabled(PIO0_IRQ_0, 1); // Enables PIO0_IRQ_0, allowing interrupts from this source

    // Start canbus
    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
}

void main_task(__unused void *params)
{
  printf("starting rx loop\n");

  while(1){
  struct can2040_msg received;
  //printf("Got message\n");
  xQueueReceive(msgs, &received, portMAX_DELAY);
  printf("data: %s", received.data);
  }

}



int main( void )
{
    stdio_init_all();
    canbus_setup();
    
    sleep_ms(3000);
    msgs = xQueueCreate(100, sizeof(struct can2040_msg));
    TaskHandle_t task;
    transmit_can();

    // while (true) {
    //     transmit_can();  // Attempt to transmit every second
    //     sleep_ms(1000);
    // }
   
    xTaskCreate(main_task, "MainThread",
                MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, &task);
    vTaskStartScheduler();


     return 0;
   
}