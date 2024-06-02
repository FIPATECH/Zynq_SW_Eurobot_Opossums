#include "main.h"

XUartPs UartInstance;

static u8 SendBuffer[UART_BUFFER_SIZE];
static u8 RecvBuffer[UART_BUFFER_SIZE];

/*
 * The following counters are used to determine when the entire buffer has
 * been sent and received.
 */
volatile int TotalReceivedCount;
volatile int TotalSentCount;
int TotalErrorCount;

int UART_Init(void) {
    XUartPs_Config *ConfigPtr;
    XUartPs *UartInstPtr = &UartInstance;
    int Status;
    int Index;
	u32 IntrMask;
	int BadByteCount = 0;

    ConfigPtr = XUartPs_LookupConfig(UART_DEVICE_ID);
    if (NULL == ConfigPtr) {
        return XST_FAILURE;
    }

    Status = XUartPs_CfgInitialize(UartInstPtr, ConfigPtr, ConfigPtr->BaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    /* Check hardware build */
	Status = XUartPs_SelfTest(UartInstPtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

    XUartPs_SetBaudRate(UartInstPtr, BAUDRATE);
    
    /*
	 * Setup the handlers for the UART that will be called from the
	 * interrupt context when data has been sent and received, specify
	 * a pointer to the UART driver instance as the callback reference
	 * so the handlers are able to access the instance data
	 */
    XUartPs_SetHandler(UartInstPtr, (XUartPs_Handler)Handler, UartInstPtr);

    /*
	 * Enable the interrupt of the UART so interrupts will occur, setup
	 * a local loopback so data that is sent will be received.
	 */
	IntrMask =
		XUARTPS_IXR_TOUT | XUARTPS_IXR_PARITY | XUARTPS_IXR_FRAMING |
		XUARTPS_IXR_OVER | XUARTPS_IXR_TXEMPTY | XUARTPS_IXR_RXFULL |
		XUARTPS_IXR_RXOVR;

    XUartPs_SetInterruptMask(UartInstPtr, IntrMask);

    XUartPs_SetOperMode(UartInstPtr, XUARTPS_OPER_MODE_LOCAL_LOOP);

    /*
	 * Set the receiver timeout. If it is not set, and the last few bytes
	 * of data do not trigger the over-water or full interrupt, the bytes
	 * will not be received. By default it is disabled.
	 *
	 * The setting of 8 will timeout after 8 x 4 = 32 bit periods.
	 * Increase the time out value if baud rate is high, decrease it if
	 * baud rate is low.
	 */
	XUartPs_SetRecvTimeout(UartInstPtr, 8);

    /*
     * Initialize the send buffer bytes and
     * the receive buffer.
     */
    for (Index = 0; Index < UART_BUFFER_SIZE; Index++) {
        SendBuffer[Index] = Index;
        RecvBuffer[Index] = 0;
    }
    /*
	 * Start receiving data before sending it since there is a loopback,
	 * ignoring the number of bytes received as the return value since we
	 * know it will be zero
	 */
	XUartPs_Recv(UartInstPtr, RecvBuffer, UART_BUFFER_SIZE);

	/*
	 * Send the buffer using the UART and ignore the number of bytes sent
	 * as the return value since we are using it in interrupt mode.
	 */
	XUartPs_Send(UartInstPtr, SendBuffer, UART_BUFFER_SIZE);

	/*
	 * Wait for the entire buffer to be received, letting the interrupt
	 * processing work in the background, this function may get locked
	 * up in this loop if the interrupts are not working correctly.
	 */
	while (1) {
		if ((TotalSentCount == UART_BUFFER_SIZE) &&
		    (TotalReceivedCount == UART_BUFFER_SIZE)) {
			break;
		}
	}

	/* Verify the entire receive buffer was successfully received */
	for (Index = 0; Index < UART_BUFFER_SIZE; Index++) {
		if (RecvBuffer[Index] != SendBuffer[Index]) {
			BadByteCount++;
		}
	}



	/* Set the UART in Normal Mode */
	XUartPs_SetOperMode(UartInstPtr, XUARTPS_OPER_MODE_NORMAL);


    /* If any bytes were not correct, return an error */
	if (BadByteCount != 0) {
		return XST_FAILURE;
	}
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
void Handler(void *CallBackRef, u32 Event, unsigned int EventData)
{
	/* All of the data has been sent */
	if (Event == XUARTPS_EVENT_SENT_DATA) {
		TotalSentCount = EventData;
	}

	/* All of the data has been received */
	if (Event == XUARTPS_EVENT_RECV_DATA) {
		TotalReceivedCount = EventData;
	}

	/*
	 * Data was received, but not the expected number of bytes, a
	 * timeout just indicates the data stopped for 8 character times
	 */
	if (Event == XUARTPS_EVENT_RECV_TOUT) {
		TotalReceivedCount = EventData;
	}

	/*
	 * Data was received with an error, keep the data but determine
	 * what kind of errors occurred
	 */
	if (Event == XUARTPS_EVENT_RECV_ERROR) {
		TotalReceivedCount = EventData;
		TotalErrorCount++;
	}

	/*
	 * Data was received with an parity or frame or break error, keep the data
	 * but determine what kind of errors occurred. Specific to Zynq Ultrascale+
	 * MP.
	 */
	if (Event == XUARTPS_EVENT_PARE_FRAME_BRKE) {
		TotalReceivedCount = EventData;
		TotalErrorCount++;
	}

	/*
	 * Data was received with an overrun error, keep the data but determine
	 * what kind of errors occurred. Specific to Zynq Ultrascale+ MP.
	 */
	if (Event == XUARTPS_EVENT_RECV_ORERR) {
		TotalReceivedCount = EventData;
		TotalErrorCount++;
	}
}
