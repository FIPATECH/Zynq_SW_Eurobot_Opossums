#ifndef UART_DEVICE_ID
#define UART_DEVICE_ID		XPAR_XUARTPS_0_DEVICE_ID
#endif

#define UART_BUFFER_SIZE	100

#define BAUDRATE            115200

extern XUartPs UartInstance;

int UART_Init(void);
void Handler(void *CallBackRef, u32 Event, unsigned int EventData);
