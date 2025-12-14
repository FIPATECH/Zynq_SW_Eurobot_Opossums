#ifndef FEETECH_UART_H
#define FEETECH_UART_H


#ifndef UART1_DEVICE_ID
#define UART1_DEVICE_ID		XPAR_XUARTPS_1_DEVICE_ID
#endif

#define UART1_BUFFER_SIZE	1000
#define BAUDRATE_UART1      1000000

extern XUartPs Uart1_Instance;

/**
 * @brief Initialize the UART1 interface
 * 
 * @return int 
 */
int UART1_Init(void);

/**
 * @brief UART1 interrupt handler
 * 
 * @param CallBackRef Pointer to the UART instance
 * @param Event Event type
 * @param EventData Event data
 */
void UART1_Handler(void *CallBackRef, u32 Event, unsigned int EventData);

/**
 * @brief Send a command via UART1
 * 
 * @param symbole Command symbol to send
 */
void Send_Uart1_Cmd(uint8_t symbole);

/**
 * @brief Send a buffer of commands via UART1
 * 
 * @param Buff Pointer to the buffer containing commands
 * @param Len Length of the buffer
 */
void Send_Uart1_Buff_Cmd(uint8_t Buff[], uint8_t Len);

/**
 * @brief Get a command from the UART1 receive buffer
 * 
 * @param c Pointer to store the received command
 * @return uint8_t Status of the operation (0 if successful, non-zero if error)
 */
uint8_t Get_Uart1_Cmd(uint8_t *c);

/**
 * @brief Calculate the number of commands that can be placed in the UART1 command buffer
 * 
 * @return u16 Number of commands that can be placed in the buffer
 */
u16 Place_In_Uart1_Cmd(void);
#endif