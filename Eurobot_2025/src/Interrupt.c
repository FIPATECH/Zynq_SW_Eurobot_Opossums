#include "main.h"

XScuGic InterruptController;

int SetupInterruptSystem(XScuGic *GicInstancePtr) {
    int Status;
    XScuGic_Config *GicConfig;

    Xil_ExceptionInit();
    GicConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
    if (NULL == GicConfig) {
        return XST_FAILURE;
    }

    Status = XScuGic_CfgInitialize(GicInstancePtr, GicConfig, GicConfig->CpuBaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
        (Xil_ExceptionHandler)XScuGic_InterruptHandler,
        GicInstancePtr);


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

    // ---------------------- CAN ----------------------
    // Connect and enable CAN interrupt
    Status = XScuGic_Connect(GicInstancePtr, CAN_IRPT_INTR,
        (Xil_InterruptHandler)XCanPs_IntrHandler, &CanInstance);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    XScuGic_Enable(GicInstancePtr, CAN_IRPT_INTR);

    // ---------------------- UART ----------------------
    // Connect and enable UART interrupt
    Status = XScuGic_Connect(GicInstancePtr, UART_IRPT_INTR,
        (Xil_InterruptHandler)XUartPs_InterruptHandler, &UartInstance);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }
    XScuGic_Enable(GicInstancePtr, UART_IRPT_INTR);

    // --------------------------------------------------------

    // Enable interrupts in the Processor.
    Xil_ExceptionEnable();

    return XST_SUCCESS;
}
