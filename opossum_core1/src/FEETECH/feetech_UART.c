#include "../main.h"


XUartPs Uart1_Instance;

u8 UART1_SendBuffer[UART1_BUFFER_SIZE];
u16 i_TX1_CMD_Buff_TODO = 0;
u16 i_TX1_CMD_Buff_DONE = 0;

u8 UART1_RecvBuffer[UART1_BUFFER_SIZE];
u16 i_RX1_CMD_Buff_TODO = 0;
u16 i_RX1_CMD_Buff_DONE = 0;

/*
 * The following counters are used to determine when the entire buffer has
 * been sent and received.
 */
volatile int TotalReceivedCount;
volatile int TotalSentCount;
int TotalErrorCount;

int UART1_Init(void) {
    XUartPs_Config *ConfigPtr;
    XUartPs *UartInstPtr = &Uart1_Instance;

    int Status;
    int Index;

	/*
     * Initialize the send buffer bytes and
     * the receive buffer.
     */
    for (Index = 0; Index < UART1_BUFFER_SIZE; Index++) {
        UART1_SendBuffer[Index] = 0;
        UART1_RecvBuffer[Index] = 0;
    }
	i_RX1_CMD_Buff_TODO = 0;
	i_RX1_CMD_Buff_DONE = 0;


	// Initialize the UART driver
    ConfigPtr = XUartPs_LookupConfig(UART1_DEVICE_ID);
    if (ConfigPtr == NULL) {
		xil_printf("Unable to find UART device\r\n");
        return XST_FAILURE;
    }else{
		xil_printf("UART device found\r\n");
	}

    XUartPs_CfgInitialize(UartInstPtr, ConfigPtr, ConfigPtr->BaseAddress);
	Status = XUartPs_SelfTest(UartInstPtr);
    if (Status != XST_SUCCESS) {
		xil_printf("Error: Unable to initialize UART device\r\n");
        return XST_FAILURE;
    }else{
		xil_printf("UART device initialized\r\n");
	}

	/* Set Baud Rate*/
    XUartPs_SetBaudRate(UartInstPtr, BAUDRATE_UART1);

	/* Set FIFO Trigger level (1 bytes)*/
	XUartPs_SetFifoThreshold(UartInstPtr, 1);
    
	/* Set receiver Timeout (wait 8 bit-tmes before triggering timeout interrupt)*/
	XUartPs_SetRecvTimeout(UartInstPtr, 8);

	/* Set Handler */
	XUartPs_SetHandler(UartInstPtr, (XUartPs_Handler)UART1_Handler, UartInstPtr); 

    /*
	 * Enable the interrupt of the UART so interrupts will occur, setup
	 * a local loopback so data that is sent will be received.
	 */
    XUartPs_SetInterruptMask(UartInstPtr, XUARTPS_IXR_TOUT | XUARTPS_IXR_PARITY | XUARTPS_IXR_FRAMING |
										XUARTPS_IXR_OVER | XUARTPS_IXR_TXEMPTY | XUARTPS_IXR_RXFULL |
										XUARTPS_IXR_RXOVR);


	/* Set the UART in Normal Mode */
	XUartPs_SetOperMode(UartInstPtr, XUARTPS_OPER_MODE_NORMAL);

	return XST_SUCCESS;

}


/**************************************************************************/
/**
*
* This function is the handler which performs processing to handle data events
* from the device.  It is called from an interrupt context. so the amount of
* processing should be minimal.
*
* This handler provides an example of how to handle data for the device and
* is application specific.
*
* @param	CallBackRef contains a callback reference from the driver,
*		in this case it is the instance pointer for the XUartPs driver.
* @param	Event contains the specific kind of event that has occurred.
* @param	EventData contains the number of bytes sent or received for sent
*		and receive events.
*
* @return	None.
*
* @note		None.
*
***************************************************************************/
void UART1_Handler(void *CallBackRef, u32 Event, unsigned int EventData)
{
	/* forward to FEETECH module so it can handle TX-complete and RX-timeout */
	FEETECH_Uart_EventHandler(Event, EventData);
	XUartPs *Uart1_InstancePtr = (XUartPs *)CallBackRef;

	while (XUartPs_IsReceiveData(Uart1_InstancePtr->Config.BaseAddress)) {
		UART1_RecvBuffer[i_RX1_CMD_Buff_TODO] = XUartPs_ReadReg(Uart1_InstancePtr->Config.BaseAddress, XUARTPS_FIFO_OFFSET);
		i_RX1_CMD_Buff_TODO++;
		if (i_RX1_CMD_Buff_TODO == UART1_BUFFER_SIZE) {  // Check buffer overflow
			i_RX1_CMD_Buff_TODO = 0;
		}
	}
	
	if (Event == XUARTPS_EVENT_RECV_ERROR || 
			Event == XUARTPS_EVENT_PARE_FRAME_BRKE ||
			Event == XUARTPS_EVENT_RECV_ORERR ) {
		TotalErrorCount++;
	}
}

u16 Place_In_Uart1_Cmd(void) {
	u16 In_Buff = (UART1_BUFFER_SIZE + i_TX1_CMD_Buff_TODO - i_TX1_CMD_Buff_DONE) % UART1_BUFFER_SIZE;
	return (UART1_BUFFER_SIZE - 1 - In_Buff);
}

void Send_Uart1_Cmd(uint8_t symbole) {
	XUartPs_Send(&Uart1_Instance, &symbole, 1);
}

void Send_Uart1_Buff_Cmd(uint8_t Buff[], uint8_t Len) {
	XUartPs_Send(&Uart1_Instance, Buff, Len);
}

uint8_t Get_Uart1_Cmd(uint8_t *c) {
	if (i_RX1_CMD_Buff_DONE != i_RX1_CMD_Buff_TODO) { 
		*c = UART1_RecvBuffer[i_RX1_CMD_Buff_DONE];
		i_RX1_CMD_Buff_DONE++;
		if (i_RX1_CMD_Buff_DONE >= UART1_BUFFER_SIZE)
			i_RX1_CMD_Buff_DONE = 0;
		return 1;
	} else {
		return 0;
	}
}