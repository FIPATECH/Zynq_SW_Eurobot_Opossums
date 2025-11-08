#ifndef INTERRUPT_H
#define INTERRUPT_H

#define INTC_DEVICE_ID          XPAR_SCUGIC_SINGLE_DEVICE_ID

#define TIMER_IRPT_INTR         XPAR_SCUTIMER_INTR
#define UART_IRPT_INTR		    XPAR_XUARTPS_0_INTR
#define UARTLITE_IRPT_INTR      XPAR_FABRIC_UARTLITE_0_VEC_ID

extern XScuGic InterruptController;

int SetupInterruptSystem(XScuGic *GicInstancePtr);

#endif // INTERRUPT_H