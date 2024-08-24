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

    // Status = init_CAN();
    // if (Status != XST_SUCCESS) {
    //     xil_printf("CAN init failed\n\r");
    //     Status = 0;
    // } else {
    //     xil_printf("CAN init done\n\r");
    //     Status = 0;
    // }

    // init_QEI();
    // PWM_Init();
    xil_printf("Init done\n\r");

    // Send_Uart_Cmd('b');

    while(1){
        if (Timer_ms1 - old_timer_ms1 >= 1000) {
            old_timer_ms1 = Timer_ms1;
            // Send_Uart_Cmd('a');
            // xil_printf("Timer_ms1: %d\n\r", Timer_ms1);
        }
        // Asserv_Loop();
        // PWM_Loop();
        Std_Com_Loop();
        // Can_Loop();
    }
    cleanup_platform();
    return 0;
}
