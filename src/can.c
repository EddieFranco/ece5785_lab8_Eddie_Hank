#include <can2040.h>
#include <hardware/regs/intctrl.h>
#include <stdio.h>
#include <pico/stdlib.h>

static struct can2040 cbus;

//callback that runs when a CAN message is received or another CAN event occurs.
// cd: Pointer to the CAN object (cbus), giving access to CAN state.
// notify: Contains notification flags for specific CAN events, such as received messages or errors.
// msg: Structure holding the actual CAN message, which includes fields like message ID and data.
static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    // Put your code here....
}


// This interrupt handler is triggered when a CAN interrupt occurs.
static void PIOx_IRQHandler(void)
{
    can2040_pio_irq_handler(&cbus); //It calls can2040_pio_irq_handler, which processes the interrupt and handles CAN events in the cbus object.
}

// This function is where the CAN bus is initialized and configured.
void canbus_setup(void)
{
    uint32_t pio_num = 0;
    uint32_t sys_clock = 125000000, bitrate = 500000;
    uint32_t gpio_rx = 4, gpio_tx = 5;

    // Setup canbus
    can2040_setup(&cbus, pio_num); //configures cbus with pio_num
    can2040_callback_config(&cbus, can2040_cb); // Register Callback, links cbus to the can2040_cb callback for handling received messages.

    // Enable irqs
    irq_set_exclusive_handler(PIO0_IRQ_0, PIOx_IRQHandler);
    irq_set_priority(PIO0_IRQ_0, PICO_DEFAULT_IRQ_PRIORITY - 1);
    irq_set_enabled(PIO0_IRQ_0, 1);

    // Start canbus
    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
}