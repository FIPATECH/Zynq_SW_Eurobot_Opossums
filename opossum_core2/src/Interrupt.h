#define INTC_DEVICE_ID          XPAR_SCUGIC_SINGLE_DEVICE_ID

#define TIMER_IRPT_INTR         XPAR_SCUTIMER_INTR

extern XScuGic InterruptController;

int SetupInterruptSystem(XScuGic *GicInstancePtr);