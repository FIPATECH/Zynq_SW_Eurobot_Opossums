#define CAN_DEVICE_ID	XPAR_XCANPS_0_DEVICE_ID

#define INTC		XScuGic

#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define CAN_INTR_VEC_ID		XPAR_XCANPS_1_INTR

#define BRPR_BAUD_PRESCALAR	9

#define BTR_SYNCJUMPWIDTH		0
#define BTR_SECOND_TIMESEGMENT	0
#define BTR_FIRST_TIMESEGMENT	7

#define CAN_MOTOR_FILTER_MASK 0x203

#define CAN_MOTOR_1_ID 0x201
#define CAN_MOTOR_2_ID 0x202
#define CAN_MOTOR_3_ID 0x203
#define CAN_MOTOR_4_ID 0x204

/* Maximum CAN frame length in word */
#define XCANPS_MAX_FRAME_SIZE_IN_WORDS (XCANPS_MAX_FRAME_SIZE / sizeof(u32))
#define FRAME_DATA_LENGTH	8 /* Frame Data field length */
#define ESC_TX_MESSAGE_ID 0x200

// #define DEBUG_CAN

extern XCanPs CanInstance; 

extern int motor1_current_order;
extern int motor2_current_order;
extern int motor3_current_order;

extern int angle_motor_1;
extern int angle_motor_2;
extern int angle_motor_3;
 
extern int torque_motor_1;
extern int torque_motor_2;
extern int torque_motor_3;
 
extern int speed_motor_1;
extern int speed_motor_2;
extern int speed_motor_3;

typedef struct {
    float motor1;   
    float motor2;
    float motor3;
} ESC_Torque;

typedef struct {
    float motor1;   
    float motor2;
    float motor3;
} ESC_Speed;

typedef struct {
    float motor1;   
    float motor2;
    float motor3;
} ESC_Angle;

typedef struct {
    ESC_Torque torque;
    ESC_Speed speed;
    ESC_Angle angle;
} ESC_Info;

typedef struct {
    uint16_t id;             // the 11-bit message ID
    uint8_t buffer;          // the buffer the message is stored in
    uint8_t payload[8];      // the 8 bytes of data
    uint8_t valid_bytes;     // the number of valid bytes in the payload
} CAN_Message;

/**
 * @brief Initializes the CAN controller.
 * 
 * @return Returns XST_SUCCESS on success, or an error code on failure.
 */
int Init_CAN(void);

/**
 * @brief Configures the CAN acceptance filters.
 * 
 * @return Returns XST_SUCCESS on success, or an error code on failure.
 */
int CAN_configure_filters(void);

/**
 * @brief Creates a CAN message.
 * 
 * @param message Pointer to the CAN_Message structure to be filled.
 * @param id The 11-bit message ID.
 * @param buffer The buffer number for the message.
 * @param payload Pointer to the data payload of the message.
 * @param valid_bytes The number of valid bytes in the payload.
 */
void CAN_create_message(CAN_Message *message, uint16_t id, uint8_t buffer, uint8_t *payload, uint8_t valid_bytes);

/**
 * @brief Transmits a CAN frame.
 * 
 * @param InstancePtr Pointer to the XCanPs instance.
 */
void CAN_transmit(XCanPs *InstancePtr);

/**
 * This function configures the CAN controller with the following settings:
 * - Baud Rate Prescalar
 * - Bit Timing Register 0 (BTR0)
 * - Bit Timing Register 1 (BTR1)
 * 
 * @param	InstancePtr is a pointer to the XCanPs instance.
 * 
 * @return	None.
 * 
 * @note		None.
 */
int Config(XCanPs *InstancePtr);

/**
 * @brief Sends a CAN frame.
 * 
 * @param InstancePtr Pointer to the XCanPs instance.
 */
void SendFrame(XCanPs *InstancePtr);

/**
 * @brief Handler for the CAN send interrupt.
 * 
 * @param CallBackRef Pointer to the callback reference (XCanPs instance).
 */
void SendHandler(void *CallBackRef);

/**
 * @brief Handler for the CAN receive interrupt.
 * 
 * @param CallBackRef Pointer to the callback reference (XCanPs instance).
 */
void RecvHandler(void *CallBackRef);

/**
 * @brief Handler for the CAN error interrupt.
 * 
 * @param CallBackRef Pointer to the callback reference (XCanPs instance).
 * @param ErrorMask The error mask indicating the type of error.
 */
void ErrorHandler(void *CallBackRef, u32 ErrorMask);

/**
 * @brief Handler for the CAN event interrupt.
 * 
 * @param CallBackRef Pointer to the callback reference (XCanPs instance).
 * @param IntrMask The interrupt mask indicating the type of event.
 */
void EventHandler(void *CallBackRef, u32 IntrMask);

/**
 * @brief Transmits motor control commands via CAN.
 * 
 * @param motor1 Current order for motor 1.
 * @param motor2 Current order for motor 2.
 * @param motor3 Current order for motor 3.
 */
void CAN_transmit_motor(int16_t motor1, int16_t motor2, int16_t motor3);

/**
 * @brief Initializes the variables used for CAN motor control.
 */
void Init_CAN_MOTOR_variables(void);

