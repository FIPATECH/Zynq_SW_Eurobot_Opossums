#include "main.h"

#define COMM_VAL (*(volatile unsigned long *)(0xFFFF0000))
extern u32 MMUTable;


int main()
{
    init_platform();
    print("CPU1: init_platform\n\r");

    

    while(1){
        while(COMM_VAL == 0){
        };

        xil_printf("Hello World - ARM1\n\r");
        sleep(1);
        COMM_VAL = 0;
    }

    cleanup_platform();
    return 0;
}
