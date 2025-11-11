#ifndef UART_PL_H
#define UART_PL_H

#define UARTLITE_DEVICE_ID    XPAR_UARTLITE_0_DEVICE_ID

#define UART_PL_BUFFER_SIZE 100

extern XUartLite UartLite;

void SendHandler(void *CallBackRef, unsigned int EventData);

void RecvHandler(void *CallBackRef, unsigned int EventData);

int UART_PL_Init(void);

void Send_Uart_PL_Cmd(uint8_t symbole);
void Send_Uart_PL_Buff_Cmd(uint8_t Buff[], uint8_t Len);
uint8_t Get_Uart_PL_Cmd(uint8_t *c);

#endif // UART_PL_H