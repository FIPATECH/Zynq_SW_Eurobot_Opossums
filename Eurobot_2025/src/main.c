#include "main.h"

int old_timer_ms1 = 0;
int Status = 0;

int main()
{

    init_platform();

    Status = SetupInterruptSystem(&InterruptController);
    if (Status != XST_SUCCESS) {
        xil_printf("Interrupt Setup Failed\r\n");
    } else {
        xil_printf("Interrupt Setup Done\r\n");
    }

    Status = Init_Timer_ms1();
    if (Status != XST_SUCCESS) {
        print("Timer init failed\n\r");
        Status = 0;
    } else {
        print("Timer init done\n\r");
        Status = 0;
    }

    // Status = init_CAN();
    // if (Status != XST_SUCCESS) {
    //     print("CAN init failed\n\r");
    //     Status = 0;
    // } else {
    //     print("CAN init done\n\r");
    //     Status = 0;
    // }

    // init_QEI();
    // PWM_Init();
    print("Init done\n\r");

    while(1){
        if (Timer_ms1 - old_timer_ms1 >= 1000){
            old_timer_ms1 = Timer_ms1;
            xil_printf("Timer_ms1 = %d\n", Timer_ms1);
        }
        // Asserv_Loop();
        PWM_Loop();
        // Can_Loop();
    }
    cleanup_platform();
    return 0;
}
