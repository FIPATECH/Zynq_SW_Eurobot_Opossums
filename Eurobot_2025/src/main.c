#include "main.h"
#include "lib_asserv/Lib_Asserv.h"


#define sev() __asm__("sev")
#define ARM1_STARTADR 0xFFFFFFF0
#define ARM1_BASEADDR 0x10080000
// #define COMM_VAL (*(volatile unsigned long *)(0xFFFF0000))

int old_timer_ms1 = 0;
int Status = 0;

typedef struct {
    u32 flag_position_valid; // CORE0 -> CORE1: 1 if position is valid, 0 otherwise
    u32 flag_position_ack;   // CORE1 -> CORE0: 1 new position taken into account, 0 otherwise
    int timer_ms1; // Timer value in ms
} sharedCommand; 

#define SHARED_MEMORY_BASEADDR 0xFFFF0000 // Base address for shared memory
sharedCommand *shared_mem = (sharedCommand *)SHARED_MEMORY_BASEADDR;

int main()
{
    u8 c;
    init_platform();

    //Disable cache on OCM    
    // S=b1 TEX=b100 AP=b11, Domain=b1111, C=b0, B=b0
    Xil_SetTlbAttributes(0xFFFF0000,0x14de2); 

    print("ARM0: writing startaddress for ARM1\n\r");
    Xil_Out32(ARM1_STARTADR, ARM1_BASEADDR);
    dmb(); //waits until write has finished

    print("ARM0: sending the SEV to wake up ARM1\n\r");
    sev();

    Status = SetupInterruptSystem(&InterruptController);

    if (Status != XST_SUCCESS) {
        xil_printf("Interrupt Setup Failed\r\n");
    } else {
        xil_printf("Interrupt Setup Done\r\n");
    }

    Status = UART_Init();
    if (Status != XST_SUCCESS) {
        xil_printf("UART init failed\n\r");
        Status = 0;
    } else {
        xil_printf("UART init done\n\r");
        Status = 0;
    }

    Status = Init_Timer_ms1();
    if (Status != XST_SUCCESS) {
        xil_printf("Timer init failed\n\r");
        Status = 0;
    } else {
        xil_printf("Timer init done\n\r");
        Status = 0;
    }

   Status = init_CAN();
   if (Status != XST_SUCCESS) {
       xil_printf("CAN init failed\n\r");
       Status = 0;
   } else {
       xil_printf("CAN init done\n\r");
       Status = 0;
   }

    // init_QEI();
    // init_CAN_MOTOR_variables();
    // PWM_Init();
    // Std_Com_Init();
    // init_AU();
    // ws2812b_init();
    // init_switch();
    // Init_Pump();
    // Init_Valve();
    // Init_Asserv();
    // Init_Stepper();
    xil_printf("Init done\n\r");
    int counter = 0;
    while(1){
        if (Timer_ms1 - old_timer_ms1 >= 1000) {
            old_timer_ms1 = Timer_ms1;
            counter++;
            printf("ARM0: counter: %d\n\r", counter);
        }

        // Check if the timer has been updated
        // if (shared_mem->flag_position_valid == 1) {
        //     // Read the timer value from shared memory
        //     int timer_value = shared_mem->timer_ms1;

        //     // Print the timer value for debugging
        //     xil_printf("ARM0: Timer value updated: %d ms\n\r", timer_value);

        //     // Reset the flag after reading
        //     shared_mem->flag_position_valid = 0;

        //     // Set the acknowledgment flag to indicate that the position has been processed
        //     shared_mem->flag_position_ack = 1;
        // }

        // if (Get_Std_In(&c)) {
        //     Interp(c);
        // }

        

        // AU_Loop();
        // LED_loop();
        // Std_Com_Loop();

        // if(AU_state == 1){
        //     LED_AU();
        //     motion_free();
        //     init_CAN_MOTOR_variables();
        //     Init_Pump();
        //     PWM_Init();
        //     Init_Asserv();
        // }else{
        //     // stepper functions
        //     Init_Stepper_Loop();
        //     Stepper_Loop();

        //     LED_CLASSIC_MODE();
        //     MaP_Asserv_Loop();
        //     Asserv_Loop();
        //     PWM_Loop();

        //     Pump_Loop();
        //     Valve_Loop();
        // }
        // IHM_loop();

        // Asserv_test_loop();
    }
    cleanup_platform();
    return 0;
}
