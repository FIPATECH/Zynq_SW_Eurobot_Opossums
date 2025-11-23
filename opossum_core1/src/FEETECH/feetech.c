#include "../main.h" // keep original defines and FEETECH_* constants

// #define DEBUG

#define COM_FEETECH_IDDLE           0x00
#define COM_FEETECH_SENDING         0x01
#define COM_FEETECH_WAIT_ANSWER     0x02

extern XUartPs Uart1_Instance; // from your uart code (you had this variable)

#define RX_TIMEOUT_MS_DEFAULT 50

uint8_t Com_FEETECH_Status = COM_FEETECH_IDDLE;
uint16_t Time_Of_Last_FEETECH_Received = 0;
uint32_t Com_FEETECH_Maxtime = 5;

uint8_t FEETECH_Loop_State = 0;

uint8_t FEETECH_Transmit_Tab[FEETECH_CMD_BUFF_LENGTH] = {0xFF, 0xFF, 0};
uint8_t FEETECH_Transmit_Goal = 0, FEETECH_Transmit_Ptr = 0;
uint8_t FEETECH_Receive_Tab[FEETECH_CMD_BUFF_LENGTH];
uint8_t FEETECH_Receive_Ptr = 0;

FEETECH_Command Liste_Command_FEETECH[FEETECH_CMD_LIST_SIZE];
uint8_t Command_FEETECH_TODO = 0;
uint8_t Command_FEETECH_DONE = 0;
uint8_t FEETECH_Cmd_Nb_Try = 0;
uint8_t FEETECH_Dumy = 0;

/* keep small local copies like before */
uint8_t feetech_torque_enable;
uint8_t feetech_id ;
uint8_t feetech_goal_position;
uint8_t feetech_moving_speed;
uint8_t feetech_torque_limit;
uint8_t feetech_present_position;
uint8_t feetech_present_speed;
uint32_t BRGVALFEETECH;

volatile uint8_t feetech_ignore_echo = 0;

/* internal flags */
static volatile uint8_t feetech_tx_done = 0; /* set by FEETECH_Uart_EventHandler when SENT event occurs */

/* forward */
uint8_t RegisterLenFEETECH(uint8_t address);

static XGpio GpioFeetechDir;  // instance AXI GPIO

/* Initialize FEETECH port using board-specific macros */
void Init_Com_FEETECH(void){
    /* Put bus in receive mode */
    XGpio_Initialize(&GpioFeetechDir, XPAR_AXI_GPIO_24_DEVICE_ID);
    XGpio_SetDataDirection(&GpioFeetechDir, FEETECH_DIR_CHANNEL, 0x0); // en sortie
    XGpio_DiscreteWrite(&GpioFeetechDir, FEETECH_DIR_CHANNEL, FEETECH_DIR_RX);

    feetech_torque_enable = FEETECH_TORQUE_ENABLE;
    feetech_id = FEETECH_ID;
    feetech_goal_position = FEETECH_GOAL_POSITION_L;
    feetech_moving_speed = FEETECH_GOAL_SPEED_L;
    feetech_torque_limit = FEETECH_TORQUE_LIMIT_L;
    feetech_present_position = FEETECH_PRESENT_POSITION_L;
    feetech_present_speed = FEETECH_PRESENT_SPEED_L;
}

/* This function must be called by your UART1 handler to forward events */
void FEETECH_Uart_EventHandler(unsigned int Event, unsigned int EventData)
{
    if (Event == XUARTPS_EVENT_SENT_DATA) {
        feetech_tx_done = 1;
    } else if (Event == XUARTPS_EVENT_RECV_DATA || Event == XUARTPS_EVENT_RECV_TOUT) {
        /* we don't process bytes here; FEETECH_Loop will fetch bytes via Get_Uart1_Cmd */
        Time_Of_Last_FEETECH_Received = Timer_ms1;
    } else if (Event == XUARTPS_EVENT_RECV_ERROR || Event == XUARTPS_EVENT_PARE_FRAME_BRKE ||
               Event == XUARTPS_EVENT_RECV_ORERR ) {
        /* track time so FEETECH_Loop's timeout logic works */
        Time_Of_Last_FEETECH_Received = Timer_ms1;
    }
}

/* Send a prepared command buffer using your XUartPs wrapper */
void FEETECH_Cmd_Send(FEETECH_Command *Cmd) {
    uint8_t i;
    unsigned int checksum_sum = 0;
    /* build frame (same as original) */
    FEETECH_Transmit_Tab [0] = 0xFF;
    FEETECH_Transmit_Tab [1] = 0xFF;
    FEETECH_Transmit_Tab [2] = Cmd->FEETECH_Addr;
    checksum_sum += Cmd->FEETECH_Addr;
    // printf("Add id %d ", checksum_sum);

    if(Cmd->Command == FEETECH_INST_WRITE_DATA) {
        //instruction 
        FEETECH_Transmit_Tab [4] = Cmd->Command;
        checksum_sum += Cmd->Command;
        // printf("Add cmd %d ", checksum_sum);
        //register address
        FEETECH_Transmit_Tab [5] = Cmd->Reg_Addr;
        checksum_sum += Cmd->Reg_Addr;
        // printf("Add reg %d ", checksum_sum);

        // parameters
        uint32_t Data_To_Send = Cmd->Data_To_Send;
        for (i = 0; i < Cmd->Nb_Data; i++) {
            FEETECH_Transmit_Tab [6 + i] = (uint8_t)(Data_To_Send & 0xFF);
            checksum_sum += (uint8_t)(Data_To_Send & 0xFF);
            // printf("Add data %d ", checksum_sum);
            Data_To_Send = (Data_To_Send >> 8);
        }
        FEETECH_Transmit_Tab [3] = Cmd->Nb_Data + 3; // Len
        checksum_sum += Cmd->Nb_Data + 3;
        // printf("Add len %d ", checksum_sum);

    } else if(Cmd->Command == FEETECH_INST_READ_DATA) {
        FEETECH_Transmit_Tab [4] = FEETECH_INST_READ_DATA;
        checksum_sum += FEETECH_INST_READ_DATA;
        // printf("Add cmd %d ", checksum_sum);
        FEETECH_Transmit_Tab [5] = Cmd->Reg_Addr;
        checksum_sum += Cmd->Reg_Addr;
        // printf("Add reg %d ", checksum_sum);
        FEETECH_Transmit_Tab [6] = Cmd->Nb_Data;
        checksum_sum += Cmd->Nb_Data;
        // printf("Add nb data %d ", checksum_sum);
        FEETECH_Transmit_Tab [3] = 4; // ID + CMD + Addr + Nb
        checksum_sum += 4;
    }

    uint8_t calculate_chk = (uint8_t)(~checksum_sum);
    // printf("Calculate chk %d \n", calculate_chk);

    FEETECH_Transmit_Goal = (uint8_t)(FEETECH_Transmit_Tab [3] + 4); // total length

    FEETECH_Transmit_Tab[FEETECH_Transmit_Goal - 1] = calculate_chk;

    FEETECH_Transmit_Ptr = 0;
    FEETECH_Receive_Ptr = 0;

    // flush cache to ensure data coherency
    Xil_DCacheFlushRange((UINTPTR)FEETECH_Transmit_Tab, FEETECH_Transmit_Goal);
    /* Put bus in TX mode */
    XGpio_DiscreteWrite(&GpioFeetechDir, FEETECH_DIR_CHANNEL, FEETECH_DIR_TX);

    // wait for 20 us settle time - clk is at 667MHz
    volatile uint32_t wait;
    for (wait = 0; wait < 13340; wait++);

    feetech_tx_done = 0;
    feetech_ignore_echo = 1;

    /* send buffer via your wrapper */
    Send_Uart1_Buff_Cmd(FEETECH_Transmit_Tab, FEETECH_Transmit_Goal);

    /* mark state as sending (actual TX completion is detected in FEETECH_Uart_EventHandler) */
    Com_FEETECH_Status = COM_FEETECH_SENDING;
    Time_Of_Last_FEETECH_Received = Timer_ms1;
    Com_FEETECH_Maxtime = RX_TIMEOUT_MS_DEFAULT;
}

/* The original FEETECH loop adapted to pull RX bytes from your RX ring using Get_Uart1_Cmd */
void FEETECH_Loop(void){
    uint8_t val8, i;
    
    uint8_t b;
    while (Get_Uart1_Cmd(&b)) {
        if (feetech_ignore_echo == 0){
            #ifdef DEBUG
                xil_printf("RX: %02X\n", b);
            #endif
            FEETECH_Receive_Tab[FEETECH_Receive_Ptr] = b;
            if(FEETECH_Receive_Ptr < (FEETECH_CMD_BUFF_LENGTH - 1))
                FEETECH_Receive_Ptr++;

            Time_Of_Last_FEETECH_Received = Timer_ms1;
        }
    }

    switch(FEETECH_Loop_State) {
        case 0:
            if (Command_FEETECH_TODO != Command_FEETECH_DONE){
                #ifdef DEBUG
                    printf("New command to process\n");
                #endif
                FEETECH_Loop_State++;
            }
            break;

        case 1:
            FEETECH_Cmd_Nb_Try = 0;
            if (((Liste_Command_FEETECH[Command_FEETECH_DONE].Command == FEETECH_INST_READ_DATA) &&
                    (Liste_Command_FEETECH[Command_FEETECH_DONE].FEETECH_Addr != FEETECH_BROADCAST)) ||
                    (Liste_Command_FEETECH[Command_FEETECH_DONE].Command == FEETECH_INST_WRITE_DATA)){
                FEETECH_Loop_State = 10;
            } else {
                *(Liste_Command_FEETECH[Command_FEETECH_DONE].Status) = FEETECH_STATUS_UNSUPORTED_CMD;
                FEETECH_Loop_State = 100;
            }
            break;

        case 10:
            #ifdef DEBUG
                printf("Sending FEETECH command\n");
            #endif
            FEETECH_Cmd_Send(&Liste_Command_FEETECH[Command_FEETECH_DONE]);
            FEETECH_Loop_State++;
            break;
        case 11:
            /* Wait for TX event flag from ISR */
            if (feetech_tx_done)
            {
                if (XUartPs_IsTransmitEmpty(&Uart1_Instance)) // make sure all data is sent
                {

                    volatile uint32_t wait;
                    for (wait = 0; wait < 1000; wait++);

                    XGpio_DiscreteWrite(&GpioFeetechDir, FEETECH_DIR_CHANNEL, FEETECH_DIR_RX);

                    u8 DummyByte;
                    while(Get_Uart1_Cmd(&DummyByte));

                    #ifdef DEBUG
                        printf("TX Done, Echo Flushed\n");
                    #endif

                    /* Safe to switch bus to RX */
                    

                    /* Clear echo ignore and mark waiting for response */
                    feetech_ignore_echo = 0;

                    /* Reset TX flag */
                    feetech_tx_done = 0;

                    FEETECH_Receive_Ptr = 0;

                    /* small settle time before we actually start waiting for bytes */
                    Time_Of_Last_FEETECH_Received = Timer_ms1;
                    FEETECH_Loop_State = 20; /* next case will check the settle timeout */
                }
            }
            break;

        case 20:
            /* Now officially in 'waiting for answer' */
            Com_FEETECH_Status = COM_FEETECH_WAIT_ANSWER;
            Time_Of_Last_FEETECH_Received = Timer_ms1;
            FEETECH_Loop_State = 21;
            break;
            
        case 21:
            if (Liste_Command_FEETECH[Command_FEETECH_DONE].FEETECH_Addr == FEETECH_BROADCAST) {
                *(Liste_Command_FEETECH[Command_FEETECH_DONE].Status) = FEETECH_STATUS_OK;
                FEETECH_Loop_State = 100;
            } else if ((FEETECH_Receive_Ptr > 3)){
                if((FEETECH_Receive_Tab[3] == (FEETECH_Receive_Ptr - 4)) ) {
                    #ifdef DEBUG
                        printf("FEETECH complete packet received\n");
                    #endif
                    FEETECH_Loop_State = 30;
                }else if((Timer_ms1 - Time_Of_Last_FEETECH_Received) > Com_FEETECH_Maxtime){
                    #ifdef DEBUG
                        printf("FEETECH incomplete packet timeout\n");
                    #endif
                    // incomplete packet and timeout
                    // xil_printf("FEETECH ID %d incomplete packet timeout\n", Liste_Command_FEETECH[Command_FEETECH_DONE].FEETECH_Addr);
                    *(Liste_Command_FEETECH[Command_FEETECH_DONE].Status) = FEETECH_STATUS_TIMEOUT;
                    FEETECH_Loop_State = 90;
                }
            } else if ((Timer_ms1 - Time_Of_Last_FEETECH_Received) > Com_FEETECH_Maxtime) {
                #ifdef DEBUG
                    printf("FEETECH no packet timeout\n");
                #endif
                // xil_printf("FEETECH ID %d timeout\n", Liste_Command_FEETECH[Command_FEETECH_DONE].FEETECH_Addr);
                *(Liste_Command_FEETECH[Command_FEETECH_DONE].Status) = FEETECH_STATUS_TIMEOUT;
                FEETECH_Loop_State = 90;
            }
            break;

        case 30:
            val8 = 0;
            for (i = 2; i <= (FEETECH_Receive_Tab[3] + 2); i++)
                val8 += FEETECH_Receive_Tab[i];
            val8 = ~val8;
            if (val8 == FEETECH_Receive_Tab[FEETECH_Receive_Tab[3] + 3]) {
                *(Liste_Command_FEETECH[Command_FEETECH_DONE].Status) = FEETECH_STATUS_OK;
                FEETECH_Loop_State = 31;
            } else {
                *(Liste_Command_FEETECH[Command_FEETECH_DONE].Status) = FEETECH_STATUS_CHKSUM_ERROR;
                FEETECH_Loop_State = 90;
            }
            break;
        case 31:
            if ((Liste_Command_FEETECH[Command_FEETECH_DONE].Command == FEETECH_INST_READ_DATA) &&
                (Liste_Command_FEETECH[Command_FEETECH_DONE].Data_Answer != NULL) ) {
                uint8_t *ptr_on_u8 = Liste_Command_FEETECH[Command_FEETECH_DONE].Data_Answer;
                for (i = 0; i < Liste_Command_FEETECH[Command_FEETECH_DONE].Nb_Data; i++) {
                    ptr_on_u8[i] = FEETECH_Receive_Tab[i + 5];
                }
            }
            FEETECH_Loop_State = 100;
            break;

        case 90:
            FEETECH_Cmd_Nb_Try ++;
            if (FEETECH_Cmd_Nb_Try < FEETECH_CMD_NB_MAX_TRY_SEND) {
                FEETECH_Loop_State = 10;
                Com_FEETECH_Status = COM_FEETECH_IDDLE;
            } else {
                FEETECH_Loop_State = 100;
            }
            break;

        case 100:
            Com_FEETECH_Status = COM_FEETECH_IDDLE;
            *((uint8_t*) Liste_Command_FEETECH[Command_FEETECH_DONE].Done) = 1;
            Command_FEETECH_DONE++;
            if (Command_FEETECH_DONE == FEETECH_CMD_LIST_SIZE)
                Command_FEETECH_DONE = 0;
            FEETECH_Loop_State = 0;
            break;

        default:
            FEETECH_Loop_State = 0;
            break;
    }
}

/* same helper functions as your dsPIC code (RegisterLen etc.) */
uint8_t RegisterLenFEETECH(uint8_t address) {
    switch (address) {
        case FEETECH_MODEL_L: case FEETECH_MODEL_H: case FEETECH_ID: case FEETECH_BAUD_RATE: case FEETECH_DELAY_TIME_RETURN: case FEETECH_LEVEL_RETURN: case FEETECH_MAX_TEMP_LIMIT: case FEETECH_MAX_INPUT_VOLT:
        case FEETECH_MIN_INPUT_VOLT: case FEETECH_SETTING_BYTE: case FEETECH_PROTECTION_ENABLE: case FEETECH_ALARM_LED: case FEETECH_CW_DEAD: case FEETECH_CCW_DEAD:
        case FEETECH_RESOLUTION: case FEETECH_MODE: case FEETECH_TORQUE_ENABLE: case FEETECH_LOCK: case FEETECH_PRESENT_VOLTAGE:
        case FEETECH_ACC: case FEETECH_PRESENT_TEMPERATURE: case FEETECH_MOVING:
            return 1;
        case FEETECH_MIN_ANGLE_LIMIT_L:  case FEETECH_MAX_ANGLE_LIMIT_L: case FEETECH_MAX_TORQUE_LIMIT_L:
        case FEETECH_OFS_L: case FEETECH_MIN_START_TORQUE: case FEETECH_OVERLOAD_CURRENT_L: case FEETECH_GOAL_POSITION_L: case FEETECH_GOAL_TIME_L: case FEETECH_GOAL_SPEED_L:
        case FEETECH_TORQUE_LIMIT_L: case FEETECH_PRESENT_POSITION_L: case FEETECH_PRESENT_SPEED_L: case FEETECH_PRESENT_LOAD_L: case FEETECH_PRESENT_CURRENT_L:
            return 2;
        default:
            return 0;
    }
}

/* Command queue helpers (same as original) */
void Add_FEETECH_Cmd(uint8_t FEETECH_Addr, uint16_t Uart_Brg, uint8_t Command, uint8_t Reg_Addr, uint32_t Data_To_Send, void *Data_Answer, uint8_t Nb_Data, uint8_t *Status, void *Done) {
    Liste_Command_FEETECH[Command_FEETECH_TODO].Uart_Brg = Uart_Brg;
    Liste_Command_FEETECH[Command_FEETECH_TODO].FEETECH_Addr = FEETECH_Addr;
    Liste_Command_FEETECH[Command_FEETECH_TODO].Command = Command;
    Liste_Command_FEETECH[Command_FEETECH_TODO].Reg_Addr = Reg_Addr;
    Liste_Command_FEETECH[Command_FEETECH_TODO].Data_To_Send = Data_To_Send;
    Liste_Command_FEETECH[Command_FEETECH_TODO].Data_Answer = Data_Answer;
    Liste_Command_FEETECH[Command_FEETECH_TODO].Nb_Data = Nb_Data;
    Liste_Command_FEETECH[Command_FEETECH_TODO].Status = Status;
    Liste_Command_FEETECH[Command_FEETECH_TODO].Done = Done;

    Command_FEETECH_TODO++;
    if (Command_FEETECH_TODO == FEETECH_CMD_LIST_SIZE)
        Command_FEETECH_TODO = 0;
}

void PutFEETECH(uint8_t id, uint8_t Reg, uint32_t Data) {
    Add_FEETECH_Cmd(id, (uint16_t)BRGVALFEETECH, FEETECH_INST_WRITE_DATA, Reg, Data, NULL, RegisterLenFEETECH(Reg), &FEETECH_Dumy, &FEETECH_Dumy);
}

void PutFEETECH_Wait(uint8_t id, uint8_t Reg, uint32_t Data) {
    volatile uint8_t Done = 0;
    Add_FEETECH_Cmd(id, (uint16_t)BRGVALFEETECH, FEETECH_INST_WRITE_DATA, Reg, Data, NULL, RegisterLenFEETECH(Reg), &FEETECH_Dumy, (void*) (&Done));
    while (!Done)
        FEETECH_Loop();
}

void PutFEETECH_Ext_Done(uint8_t id, uint8_t Reg, uint32_t Data, void *Done) {
    Add_FEETECH_Cmd(id, (uint16_t)BRGVALFEETECH, FEETECH_INST_WRITE_DATA, Reg, Data, NULL, RegisterLenFEETECH(Reg), &FEETECH_Dumy, Done);
}

uint32_t GetFEETECH_Wait(uint8_t id, uint8_t Reg) {
    volatile uint8_t Done = 0;
    uint32_t Data_Answer = 0;
    Add_FEETECH_Cmd(id, (uint16_t)BRGVALFEETECH, FEETECH_INST_READ_DATA, Reg, 0, &Data_Answer, RegisterLenFEETECH(Reg), &FEETECH_Dumy, (void*) (&Done));
    while (!Done)
        FEETECH_Loop();
    return Data_Answer;
}

void GetFEETECH(uint8_t id, uint8_t Reg, void *Data_Answer) {
    Add_FEETECH_Cmd(id, (uint16_t)BRGVALFEETECH, FEETECH_INST_READ_DATA, Reg, 0, Data_Answer, RegisterLenFEETECH(Reg), &FEETECH_Dumy, &FEETECH_Dumy);
}

void GetFEETECH_Ext_Done(uint8_t id, uint8_t Reg, void *Data_Answer, void *Done) {
    Add_FEETECH_Cmd(id, (uint16_t)BRGVALFEETECH, FEETECH_INST_READ_DATA, Reg, 0, Data_Answer, RegisterLenFEETECH(Reg), &FEETECH_Dumy, Done);
}

uint8_t FEETECH_All_Cmd_Done(void) {
    return (Command_FEETECH_TODO == Command_FEETECH_DONE);
}

void GetFEETECH_Ext_Done_With_Status(uint8_t id, uint8_t Reg, void *Data_Answer, void *Done, uint8_t *Status) {
    Add_FEETECH_Cmd(id, (uint16_t)BRGVALFEETECH, FEETECH_INST_READ_DATA, Reg, 0, Data_Answer, RegisterLenFEETECH(Reg), Status, Done);
}
