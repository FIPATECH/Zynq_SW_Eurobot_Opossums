#include "main.h"

// #define COMM_VAL (*(volatile unsigned long *)(0xFFFF0000))
extern u32 MMUTable;


typedef struct {
    u32 flag_position_valid; // CORE0 -> CORE1: 1 if position is valid, 0 otherwise
    u32 flag_position_ack;   // CORE1 -> CORE0: 1 new position taken into account, 0 otherwise
    int timer_ms1; // Timer value in ms
} sharedCommand; 

#define SHARED_MEMORY_BASEADDR 0xFFFF0000 // Base address for shared memory
sharedCommand *shared_mem = (sharedCommand *)SHARED_MEMORY_BASEADDR;

int old_timer_ms1 = 0;

int main()
{
    init_platform();
    print("CPU1: init_platform\n\r");

    int Status = SetupInterruptSystem(&InterruptController);

    if (Status != XST_SUCCESS) {
        xil_printf("Interrupt Setup Failed\r\n");
    } else {
        xil_printf("Interrupt Setup Done\r\n");
    }

    // initialise timer
    Status = Init_Timer_ms1();
    if (Status != XST_SUCCESS) {
        xil_printf("CPU1: Timer initialization failed\n\r");
        return XST_FAILURE;
    }
    
    // initialise shared memory
    shared_mem->flag_position_valid = 0;
    shared_mem->flag_position_ack = 0;
    
    while(1){
        if(Timer_ms1 - old_timer_ms1 >= 1000) {
            old_timer_ms1 = Timer_ms1;
            xil_printf("ARM1: Timer value: %d ms\n\r", Timer_ms1);
        }


        // shared_mem->flag_position_ack = 0; // Reset the acknowledgment flag
        // // Check if the timer has been updated
        // if (Timer_ms1 != old_timer_ms1) {
        //     old_timer_ms1 = Timer_ms1;

        //     // Update the shared memory with the new timer value
        //     shared_mem->timer_ms1 = Timer_ms1;

        //     // Set the flag to indicate that the position is valid
        //     shared_mem->flag_position_valid = 1;

        //     // Print the timer value for debugging
        //     xil_printf("CPU1: Timer value updated: %d ms\n\r", Timer_ms1);
        // }
        // while (shared_mem->flag_position_ack == 0) {
        //     // Wait for the acknowledgment from CORE0
        //     // This loop will block until CORE0 acknowledges the new position
        // }
    }

    cleanup_platform();
    return 0;
}
