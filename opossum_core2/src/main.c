#include <stdio.h>
#include <sleep.h>
#include "xil_io.h"
#include "xil_mmu.h"
#include "platform.h"
#include "xil_cache.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xpseudo_asm.h"
#include "xil_exception.h"

#define COMM_VAL (*(volatile unsigned long *)(0xFFFF0000))
extern u32 MMUTable;


int main()
{
    init_platform();
    print("CPU1: init_platform\n\r");

    //Disable cache on OCM
    // S=b1 TEX=b100 AP=b11, Domain=b1111, C=b0, B=b0
    Xil_SetTlbAttributes(0xFFFF0000,0x14de2);

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
