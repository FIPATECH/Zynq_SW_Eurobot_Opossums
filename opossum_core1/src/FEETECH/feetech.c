#include "../main.h" // keep original defines and FEETECH_* constants

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
    uint8_t i, val8;
    /* build frame (same as original) */
    FEETECH_Transmit_Tab [0] = 0xFF;
    FEETECH_Transmit_Tab [1] = 0xFF;
    FEETECH_Transmit_Tab [2] = Cmd->FEETECH_Addr;
    FEETECH_Transmit_Tab [4] = Cmd->Command;
    FEETECH_Transmit_Tab [5] = Cmd->Reg_Addr;

    if (Cmd->Command == FEETECH_INST_WRITE_DATA) {
        uint32_t Data_To_Send = Cmd->Data_To_Send;
        for (i = 0; i < Cmd->Nb_Data; i++) {
            FEETECH_Transmit_Tab [6 + i] = (uint8_t)(Data_To_Send & 0xFF);
            Data_To_Send = (Data_To_Send >> 8);
        }
        FEETECH_Transmit_Tab [3] = Cmd->Nb_Data + 3; // Len
    } else if (Cmd->Command == FEETECH_INST_READ_DATA) {
        FEETECH_Transmit_Tab [6] = Cmd->Nb_Data;
        FEETECH_Transmit_Tab [3] = 4; // ID + CMD + Addr + Nb
    }

    FEETECH_Transmit_Goal = (uint8_t)(FEETECH_Transmit_Tab [3] + 4); // total length
    /* checksum */
    val8 = 0;
    for (i = 2; i <= (FEETECH_Transmit_Tab [3] + 2); i++) {
        val8 += FEETECH_Transmit_Tab[i];
    }
    FEETECH_Transmit_Tab[FEETECH_Transmit_Goal - 1] = (uint8_t)(~val8);

    FEETECH_Transmit_Ptr = 0;
    FEETECH_Receive_Ptr = 0;

    /* === Update UART baudrate live if different === */
    // if (Cmd->Uart_Brg > 0) {
    //     XUartPs_SetBaudRate(&Uart1_Instance, Cmd->Uart_Brg);
    // }

    /* Put bus in TX mode */
    XGpio_DiscreteWrite(&GpioFeetechDir, FEETECH_DIR_CHANNEL, FEETECH_DIR_TX);
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

        // Si on est en phase d’écho, on ignore tout ce qui revient
        if (feetech_ignore_echo == 0){
            FEETECH_Receive_Tab[FEETECH_Receive_Ptr] = b;
            if (FEETECH_Receive_Ptr < (FEETECH_CMD_BUFF_LENGTH - 1))
                FEETECH_Receive_Ptr++;

            Time_Of_Last_FEETECH_Received = Timer_ms1;
        }

    }

    switch(FEETECH_Loop_State) {
        case 0:
            if (Command_FEETECH_TODO != Command_FEETECH_DONE){
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
            FEETECH_Cmd_Send(&Liste_Command_FEETECH[Command_FEETECH_DONE]);
            FEETECH_Loop_State++;
            break;
        case 11:
            /* Wait for TX event flag from ISR */
            if (feetech_tx_done)
            {
                if (XUartPs_IsTransmitEmpty(&Uart1_Instance)) // make sure all data is sent
                {

                    u8 DummyByte;
                    while(XUartPs_IsReceiveData(Uart1_Instance.Config.BaseAddress)) {
                        XUartPs_Recv(&Uart1_Instance, &DummyByte, 1); // flush any received echo bytes
                    }
                    (void)DummyByte; // avoid compiler warning

                    /* Safe to switch bus to RX */
                    XGpio_DiscreteWrite(&GpioFeetechDir, FEETECH_DIR_CHANNEL, FEETECH_DIR_RX);

                    /* Clear echo ignore and mark waiting for response */
                    feetech_ignore_echo = 0;

                    /* Reset TX flag */
                    feetech_tx_done = 0;

                    /* small settle time before we actually start waiting for bytes */
                    Time_Of_Last_FEETECH_Received = Timer_ms1;
                    FEETECH_Loop_State = 20; /* next case will check the settle timeout */
                }
            }
            break;

        case 20:
             /* Wait one millisecond to let the line/PL direction settle, then start answer wait */
            if ((Timer_ms1 - Time_Of_Last_FEETECH_Received) >= 1) {
                /* Now officially in 'waiting for answer' */
                Com_FEETECH_Status = COM_FEETECH_WAIT_ANSWER;
                /* refresh timestamp for RX timeout calculation */
                Time_Of_Last_FEETECH_Received = Timer_ms1;
                FEETECH_Loop_State = 21;
            }
            break;
        case 21:
            if (Liste_Command_FEETECH[Command_FEETECH_DONE].FEETECH_Addr == FEETECH_BROADCAST) {
                *(Liste_Command_FEETECH[Command_FEETECH_DONE].Status) = FEETECH_STATUS_OK;
                FEETECH_Loop_State = 100;
            } else if ((FEETECH_Receive_Ptr > 3) &&
                       (FEETECH_Receive_Tab[3] == (FEETECH_Receive_Ptr - 4)) ) {
                FEETECH_Loop_State = 30;
            } else if ((Timer_ms1 - Time_Of_Last_FEETECH_Received) > Com_FEETECH_Maxtime) {
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


void __attribute__((__interrupt__, no_auto_psv)) _U2TXInterrupt(void) {
#ifdef DEBUG_AXSTS
    printf("interrupt TX \n");
#endif
    // tant qu'il y a des trucs � mettre
    IFS1bits.U2TXIF = 0; // clear TX interrupt flag
    if (AXSTS_Transmit_Goal != AXSTS_Transmit_Ptr) {
        while ((AXSTS_Transmit_Goal != AXSTS_Transmit_Ptr) && (!U2STAbits.UTXBF)) {
            //printf("TX %02X\r\n", AXSTS_Transmit_Tab[AXSTS_Transmit_Ptr]);
            U2TXREG = AXSTS_Transmit_Tab[AXSTS_Transmit_Ptr];
            AXSTS_Transmit_Ptr++;
        }
    } else {
        // desactive l'IT TX, passe en RX, active l'IT RX
        Com_AXSTS_Status = COM_AXSTS_WAIT_ANSWER;
#ifdef DEBUG_AXSTS
        printf("Sw\r\n");
#endif
        _U2RXR = AXSTS_BUS_TX_RX_RP_NB;
        AXSTS_BUS_TX_RX_RP_REG = 0;
        AXSTS_BUS_EN = 0;
        uint8_t poubelle;
        while (U2STAbits.URXDA) {
            poubelle = U2RXREG;
        }
        // change d'IT possible
        IFS1bits.U2RXIF = 0;
        IEC1bits.U2TXIE = 0;
        IEC1bits.U2RXIE = 1;
    }
    Time_Of_Last_AXSTS_Received = Timer_ms1;
}

void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void) {
    uint16_t valrx;
    IFS1bits.U2RXIF = 0; // On baisse le FLAG
    while (U2STAbits.URXDA) { // tant que la FIFO n'est pas vide, on prend
        valrx = U2RXREG;
        AXSTS_Receive_Tab[AXSTS_Receive_Ptr] = valrx;
        //printf("%ld RX %02X\r\n", Timer_ms1, valrx);
        if (AXSTS_Receive_Ptr < (AXSTS_CMD_BUFF_LENGTH - 1))
            AXSTS_Receive_Ptr++;
    }
    Time_Of_Last_AXSTS_Received = Timer_ms1;
}



void AXSTS_Loop(void){
    uint8_t val8, i;
    switch(AXSTS_Loop_State) {
/////////////////////////////////////////////////// repos ///////////////////////////////////////////////       
        case 0: //�tat d'attente d'un envoie � effectuer
            if (Command_AXSTS_TODO != Command_AXSTS_DONE){
#ifdef DEBUG_AXSTS
                printf("go command \n");
#endif
                AXSTS_Loop_State++;      
            }
            break;
        case 1: // check et initialisations
            AXSTS_Cmd_Nb_Try = 0;
            if (((Liste_Command_AXSTS[Command_AXSTS_DONE].Command == AXSTS_INST_READ_DATA) &&
                    (Liste_Command_AXSTS[Command_AXSTS_DONE].AXSTS_Addr != AXSTS_BROADCAST)) || // si ce nouvel ordre est support� (read ou write)
                    (Liste_Command_AXSTS[Command_AXSTS_DONE].Command == AXSTS_INST_WRITE_DATA)){
                AXSTS_Loop_State = 10;
            } else { //si non support� on le mets en status, et on passe � la suite
                *(Liste_Command_AXSTS[Command_AXSTS_DONE].Status) = AXSTS_STATUS_UNSUPORTED_CMD;
                AXSTS_Loop_State = 100;
            }
            break;
            
////////////////////////////////////////////////// en transmission ///////////////////////////////////////////////    
        case 10:    // debut de l'envoi
#ifdef DEBUG_AXSTS
            printf("debut envoi \n");
#endif
            AXSTS_Cmd_Send(&Liste_Command_AXSTS[Command_AXSTS_DONE]);
            AXSTS_Loop_State ++;
            break;
        case 11:
            // declenche l'IT TX :
            IFS1bits.U2TXIF = 1;
            IEC1bits.U2TXIE = 1;
            AXSTS_Loop_State = 20;
            break;
            
////////////////////////////////////////////////// en r�ception ///////////////////////////////////////////////    
        case 20:    // attente de la reception
            
            if (Com_AXSTS_Status == COM_AXSTS_WAIT_ANSWER) {
                AXSTS_Loop_State++;
                Time_Of_Last_AXSTS_Received = Timer_ms1;
            }
            break;
        case 21:
#ifdef DEBUG_AXSTS
            printf("case 21 \n");
#endif
            // si c'est un broadcast, on ne s'attend meme pas a recevoir quoique ce soit
            if (Liste_Command_AXSTS[Command_AXSTS_DONE].AXSTS_Addr == AXSTS_BROADCAST) {
#ifdef DEBUG_AXSTS
                printf("status ok \n");
#endif
                *(Liste_Command_AXSTS[Command_AXSTS_DONE].Status) = AXSTS_STATUS_OK;
                AXSTS_Loop_State = 100;
            // ou alors on a recu une tramme de logueur coh�rente
            } else if ((AXSTS_Receive_Ptr > 3) && (AXSTS_Receive_Tab[3] == (AXSTS_Receive_Ptr - 4)) && U2STAbits.RIDLE) {
                AXSTS_Loop_State = 30;   // va l'analyser
#ifdef DEBUG_AXSTS
                printf("retour ok \n");
#endif
            // ou si on a depasse le maxtime, pb
            } else if ((Timer_ms1 - Time_Of_Last_AXSTS_Received) > Com_AXSTS_Maxtime) {
#ifdef DEBUG_AXSTS
                printf("timeout \n");
#endif
                *(Liste_Command_AXSTS[Command_AXSTS_DONE].Status) = AXSTS_STATUS_TIMEOUT;
                AXSTS_Loop_State = 90;
            }
            break;
        case 30:
#ifdef DEBUG_AXSTS
            printf("case 30\n");
#endif
            // ici la tramme a forcement la bonne longueur
            // verif du checksum
            val8 = 0;
            for (i = 2; i <= (AXSTS_Receive_Tab[3] + 2); i++)
                val8 += AXSTS_Receive_Tab[i];
            // v�rif si checksum est bon
            val8 = ~val8;
            if (val8 == AXSTS_Receive_Tab[AXSTS_Receive_Tab[3] + 3]) { // si le checksum en r�ception est bon
                *(Liste_Command_AXSTS[Command_AXSTS_DONE].Status) = AXSTS_STATUS_OK;
#ifdef DEBUG_AXSTS
                printf("end checksum ok\n");
#endif
                AXSTS_Loop_State = 31;
            } else {
                *(Liste_Command_AXSTS[Command_AXSTS_DONE].Status) = AXSTS_STATUS_CHKSUM_ERROR;
#ifdef DEBUG_AXSTS
                printf("end checksum pas ok\n");
#endif
                AXSTS_Loop_State = 90;
            }
            break;
        case 31:
            // transfert des donn�es recues si c'est une lecture
            if ((Liste_Command_AXSTS[Command_AXSTS_DONE].Command == AXSTS_INST_READ_DATA) &&
                (Liste_Command_AXSTS[Command_AXSTS_DONE].Data_Answer != NULL)          ){ 
                
                uint8_t *ptr_on_u8 = Liste_Command_AXSTS[Command_AXSTS_DONE].Data_Answer;
                for (i = 0; i < Liste_Command_AXSTS[Command_AXSTS_DONE].Nb_Data; i++) {
                    ptr_on_u8[i] = AXSTS_Receive_Tab[i + 5];
                }
            }
            AXSTS_Loop_State = 100;
            break;
            
        case 90:
            // maxtime, ou checksum errror, relance si y a encore des essais a faire
            AXSTS_Cmd_Nb_Try ++;
            if (AXSTS_Cmd_Nb_Try < AXSTS_CMD_NB_MAX_TRY_SEND) {
                AXSTS_Loop_State = 10;   // relance la transmission
                U2MODEbits.UARTEN = 0;  // re eteind le module Uart
                Com_AXSTS_Status = COM_AXSTS_IDDLE;
            } else {
                AXSTS_Loop_State = 100;
            }
            break;
            
        case 100:
            U2MODEbits.UARTEN = 0;  // re eteind le module Uart
            IEC1bits.U2RXIE = 0;
            Com_AXSTS_Status = COM_AXSTS_IDDLE;
            *((uint8_t*) Liste_Command_AXSTS[Command_AXSTS_DONE].Done) = 1;
            Command_AXSTS_DONE++;
            if (Command_AXSTS_DONE == AXSTS_CMD_LIST_SIZE)
                Command_AXSTS_DONE = 0;
#ifdef DEBUG_AXSTS
            printf("fin loop\n");
#endif
            AXSTS_Loop_State = 0;
            break;
            
        default:
#ifdef DEBUG_AXSTS
            printf("AXSTS_Loop Error\r\n");
#endif
            AXSTS_Loop_State = 0;
            break;
    }
}


void AXSTS_Cmd_Send(AXSTS_Command *Cmd) {
    uint8_t i, val8;
    // preparation de la trame
    AXSTS_Transmit_Tab [0] = 0xFF;
    AXSTS_Transmit_Tab [1] = 0xFF;
    AXSTS_Transmit_Tab [2] = Cmd->AXSTS_Addr;
    // la longueur du packet, dans le 3, apr�s :

    AXSTS_Transmit_Tab [4] = Cmd->Command;
    AXSTS_Transmit_Tab [5] = Cmd->Reg_Addr;

    if (Cmd->Command == AXSTS_INST_WRITE_DATA) {
        uint32_t Data_To_Send = Cmd->Data_To_Send;
        for (i = 0; i < Cmd->Nb_Data; i++) {
            AXSTS_Transmit_Tab [6 + i] = Data_To_Send & 0xFF;
            Data_To_Send = Data_To_Send >> 8;
        }
        AXSTS_Transmit_Tab [3] = Cmd->Nb_Data + 3; // ID + CMD + REG + x*datas
    } else if (Cmd->Command == AXSTS_INST_READ_DATA) {
        AXSTS_Transmit_Tab [6] = Cmd->Nb_Data;
        AXSTS_Transmit_Tab [3] = 4; // ID + CMD + Addr_Reg + Nb data to read
    }
    
    AXSTS_Transmit_Goal = AXSTS_Transmit_Tab [3] + 4; //  2*FF + Len + [] + chksum
    
    // calcul du checksum
    val8 = 0;
    for (i = 2; i <= (AXSTS_Transmit_Tab [3] + 2); i++) {
        val8 += AXSTS_Transmit_Tab[i];
    }
    AXSTS_Transmit_Tab[AXSTS_Transmit_Goal - 1] = ~val8;
    
//    printf("Must Send");
//    for (i = 0; i < AXSTS_Transmit_Goal; i++) {
//        printf(" %02X", AXSTS_Transmit_Tab[i]);
//    }
//    printf("\r\n");
    
    AXSTS_Transmit_Ptr = 0;
    AXSTS_Receive_Ptr = 0;
    
    AXSTS_BUS_EN = 1;
    AXSTS_BUS_TX_RX_RP_REG = _RPOUT_U2TX;
    
    _U2RXR = 0x1F;  // desactivtation de la partie RX
    //_U2RXR = AXBUS2_RX_RP_NB;
    
    // parametre et rallume l'uart
    IFS1bits.U2RXIF = 0;
    IFS1bits.U2TXIF = 0;
    IEC1bits.U2RXIE = 0;
    IEC1bits.U2TXIE = 0;
    U2BRG = Cmd->Uart_Brg;
//    printf("BRG : %04X\r\n", U2BRG);
    Com_AXSTS_Maxtime = 20;      // a adapter en focntion de la vitesse de transmission
    U2MODEbits.UARTEN = 1;      // rallume le module uart
    U2STAbits.UTXEN = 1;        // enable TX  (apres UARTEN, sinon marche pas)
        
    // purge buffer r�cepetion
    uint8_t poubelle;
    while (U2STAbits.URXDA) {
        poubelle = U2RXREG;
    }
#ifdef DEBUG_AXSTS
    printf("fin command send \n");
#endif
    Com_AXSTS_Status = COM_AXSTS_SENDING;
    Time_Of_Last_AXSTS_Received = Timer_ms1;
    
}