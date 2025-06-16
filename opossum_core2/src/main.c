#include "main.h"
#include "lib_asserv/Lib_Asserv.h"

// #define COMM_VAL (*(volatile unsigned long *)(0xFFFF0000))
extern u32 MMUTable;

int old_timer_us1 = 0;
int old_timer_ms1 = 0;

int main()
{
    Xil_SetTlbAttributes(0xFFFF0000,0x14de2); 

    init_platform();
    print("CPU1: init_platform\n\r");


    int Status = SetupInterruptSystem(&InterruptController);

    if (Status != XST_SUCCESS) {
        xil_printf("Interrupt Setup Failed\r\n");
    } else {
        xil_printf("Interrupt Setup Done\r\n");
    }

    // initialise timer
    Status = Init_Timer_us1();
    if (Status != XST_SUCCESS) {
        xil_printf("CPU1: Timer initialization failed\n\r");
        return XST_FAILURE;
    }
    
    // initialise shared memory
    init_shared_memory();


    Init_CAN();
    Init_AU();
    Init_Asserv();
    

    while(1){
        if(Timer_ms1 - old_timer_ms1 >= 1000) {
            old_timer_ms1 = Timer_ms1;
            printf("CPU1: Timer_ms1: %d\n\r", Timer_ms1);
            // shared_mem->Timer_ms = Timer_ms1;
            // __asm__ volatile("dsb sy" ::: "memory");
            // shared_mem->flag_Timer_ms_valid = 1;
            // shared_mem->flag_Timer_ms_ack = 0;
            // sharedCommand local_data;
            // local_data.Timer_ms = Timer_ms1;
            // SEND_FIELD_BLOCKING(&local_data, Timer_ms);
        }

        AU_Loop();
        check_for_cmd_loop();
        if(AU_state == 1){
            motion_free();
            Init_CAN_MOTOR_variables();
            Init_Asserv();
        }else{
            Asserv_Loop();
        }
    }

    cleanup_platform();
    return 0;
}
