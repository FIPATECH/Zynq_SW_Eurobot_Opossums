#include "main.h"

XUartLite UartLite;

u8 SendBuffer[UART_PL_BUFFER_SIZE];
u16 i_TX_PL_CMD_Buff_TODO = 0;
u16 i_TX_PL_CMD_Buff_DONE = 0;

u8 ReceiveBuffer[UART_PL_BUFFER_SIZE];
u16 i_RX_PL_CMD_Buff_TODO = 0;
u16 i_RX_PL_CMD_Buff_DONE = 0;


int UART_PL_Init(void) {
    // Initialize the UART Lite driver
    XUartLite_Config *ConfigPtr;

    int Status;
    u32 Index;

    ConfigPtr = XUartLite_LookupConfig(UARTLITE_DEVICE_ID);
    if (ConfigPtr == NULL) {
        xil_printf("Unable to find UART PL device\n\r");
        return XST_FAILURE;
    } else {
        xil_printf("UART PL device found\n\r");
    }
    Status = XUartLite_Initialize(&UartLite, ConfigPtr->DeviceId);
    if (Status != XST_SUCCESS) {
        xil_printf("UART PL initialization failed\n\r");
        return XST_FAILURE;
    } else {
        xil_printf("UART PL initialized\n\r");
    }

    Status = XUartLite_SelfTest(&UartLite);
    if (Status != XST_SUCCESS) {
        xil_printf("UART PL self-test failed\n\r");
        return XST_FAILURE;
    } else {
        xil_printf("UART PL self-test passed\n\r");
    }

    XUartLite_SetSendHandler(&UartLite, SendHandler, &UartLite);
    XUartLite_SetRecvHandler(&UartLite, RecvHandler, &UartLite);
    XUartLite_EnableInterrupt(&UartLite);

    for (Index = 0; Index < UART_PL_BUFFER_SIZE; Index++) {
        SendBuffer[Index] = Index;
    }

    return XST_SUCCESS;
}




/*****************************************************************************/
/**
*
* This function is the handler which performs processing to send data to the
* UartLite. It is called from an interrupt context such that the amount of
* processing performed should be minimized. It is called when the transmit
* FIFO of the UartLite is empty and more data can be sent through the UartLite.
*
* This handler provides an example of how to handle data for the UartLite,
* but is application specific.
*
* @param	CallBackRef contains a callback reference from the driver.
*		In this case it is the instance pointer for the UartLite driver.
* @param	EventData contains the number of bytes sent or received for sent
*		and receive events.
*
* @return	None.
*
* @note		None.
*
****************************************************************************/
void SendHandler(void *CallBackRef, unsigned int EventData)
{
    // This function is called when the UartLite is ready to send more data.
}

/****************************************************************************/
/**
*
* This function is the handler which performs processing to receive data from
* the UartLite. It is called from an interrupt context such that the amount of
* processing performed should be minimized.  It is called data is present in
* the receive FIFO of the UartLite such that the data can be retrieved from
* the UartLite. The size of the data present in the FIFO is not known when
* this function is called.
*
* This handler provides an example of how to handle data for the UartLite,
* but is application specific.
*
* @param	CallBackRef contains a callback reference from the driver, in
*		this case it is the instance pointer for the UartLite driver.
* @param	EventData contains the number of bytes sent or received for sent
*		and receive events.
*
* @return	None.
*
* @note		None.
*
****************************************************************************/
void RecvHandler(void *CallBackRef, unsigned int EventData)
{
    XUartLite *UartLitePtr = (XUartLite *)CallBackRef;
    u8 buffer[UART_PL_BUFFER_SIZE];
    int bytesRead;
    bytesRead = XUartLite_Recv(UartLitePtr, buffer, EventData);

    if (bytesRead > 0) {
        for (int i = 0; i < bytesRead; i++) {
            ReceiveBuffer[i_RX_PL_CMD_Buff_TODO] = buffer[i];
            i_RX_PL_CMD_Buff_TODO++;
            if (i_RX_PL_CMD_Buff_TODO >= UART_PL_BUFFER_SIZE) {
                i_RX_PL_CMD_Buff_TODO = 0; // Reset to avoid overflow
            }
        }
    }
}

void Send_Uart_PL_Cmd(uint8_t symbole) {
    XUartLite_Send(&UartLite, &symbole, 1);
}

void Send_Uart_PL_Buff_Cmd(uint8_t Buff[], uint8_t Len) {
    XUartLite_Send(&UartLite, Buff, Len);
}

uint8_t Get_Uart_PL_Cmd(uint8_t *c) {
    if (i_RX_PL_CMD_Buff_TODO != i_RX_PL_CMD_Buff_DONE) {
        *c = ReceiveBuffer[i_RX_PL_CMD_Buff_DONE];
        i_RX_PL_CMD_Buff_DONE++;
        if (i_RX_PL_CMD_Buff_DONE >= UART_PL_BUFFER_SIZE) {
            i_RX_PL_CMD_Buff_DONE = 0; // Reset to avoid overflow
        }
        return 1; // Command successfully retrieved
    } 
    return 0; // No command available
}