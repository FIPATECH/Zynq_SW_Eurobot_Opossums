#define INTC_DEVICE_ID          XPAR_SCUGIC_SINGLE_DEVICE_ID

#define TIMER_IRPT_INTR         XPAR_SCUTIMER_INTR
#define CAN_IRPT_INTR           XPAR_XCANPS_1_INTR
#define UART_IRPT_INTR		    XPAR_XUARTPS_0_INTR

extern XScuGic InterruptController;

int SetupInterruptSystem(XScuGic *GicInstancePtr);