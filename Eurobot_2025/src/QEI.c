#include "main.h"

int qei_0 = 0; 
int qei_1 = 0;
XGpio QEI_0;
XGpio QEI_1;
u32 Old_Timer_ms1 = 0;

int read_channel(int base_addr){
	volatile int *channel_ptr =(volatile int *)base_addr;
	return *channel_ptr;
}

void init_QEI(void){
	// Init QEI
    XGpio_Initialize(&QEI_0, XPAR_AXI_GPIO_0_DEVICE_ID);
    XGpio_Initialize(&QEI_1, XPAR_AXI_GPIO_1_DEVICE_ID);

	XGpio_SetDataDirection(&QEI_0, 1, 1);
    XGpio_SetDataDirection(&QEI_1, 1, 1);
}

void read_qei(void){
	qei_0 = XGpio_DiscreteRead(&QEI_0, 1);
	qei_1 = XGpio_DiscreteRead(&QEI_1, 1);

	#ifdef DEBUG
	xil_printf("QEI: %d, %d\n", qei_0, qei_1);
	#endif
	Old_Timer_ms1 = Timer_ms1;
}
