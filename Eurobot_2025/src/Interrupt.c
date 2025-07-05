#include "main.h"

XScuGic InterruptController;


int SetupInterruptSystem(XScuGic *GicInstancePtr) {
    int Status;
    XScuGic_Config *GicConfig;

    /*
     * Initialize the interrupt controller driver so that it is ready to
     * use.
    */
    GicConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
    if (NULL == GicConfig) {
        return XST_FAILURE;
    }

    Status = XScuGic_CfgInitialize(GicInstancePtr, GicConfig, 
                        GicConfig->CpuBaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // --------------------------------------------------------
    // ---------------------- Interrupts ----------------------
    // --------------------------------------------------------

    // ---------------------- Timer ----------------------
    // Connect and enable Timer interrupt
    Status = XScuGic_Connect(GicInstancePtr, TIMER_IRPT_INTR,
                    (Xil_InterruptHandler)TimerIntrHandler, &TimerInstance);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    XScuGic_Enable(GicInstancePtr, TIMER_IRPT_INTR);


    // ---------------------- UART PS ----------------------
    // Connect and enable UART interrupt
    Status = XScuGic_Connect(GicInstancePtr, UART_IRPT_INTR,
                    (Xil_InterruptHandler)XUartPs_InterruptHandler, &UartInstance);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    XScuGic_Enable(GicInstancePtr, UART_IRPT_INTR);

    // ---------------------- UART PL ----------------------
    XScuGic_SetPriorityTriggerType(GicInstancePtr, UARTLITE_IRPT_INTR, 0xA0, 0x3);
    // Connect and enable UART PL interrupt
    Status = XScuGic_Connect(GicInstancePtr, UARTLITE_IRPT_INTR,
                            (Xil_InterruptHandler)XUartLite_InterruptHandler, &UartLite);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    XScuGic_Enable(GicInstancePtr, UARTLITE_IRPT_INTR);
    // --------------------------------------------------------


    /*
     * Connect the interrupt controller interrupt handler to the hardware
     * interrupt handling logic in the processor.
    */
    Xil_ExceptionInit();

    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                                (Xil_ExceptionHandler)XScuGic_InterruptHandler,
                                GicInstancePtr);

    Xil_ExceptionEnable();
    
    return XST_SUCCESS;
}