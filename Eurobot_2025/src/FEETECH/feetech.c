#include "../main.h"

#define COM_FEETECH_IDDLE           0x00
#define COM_FEETECH_SENDING         0x01
#define COM_FEETECH_WAIT_ANSWER     0x02

//uint8_t Com_FEETECH_Selected_Bus = FEETECH_BUS2;

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

uint8_t Choix_AX =0; // 0=feetech LED VERT 1 = AX LED BLEU
uint8_t FEETECH_TORQUE_ENABLE;
uint8_t FEETECH_ID ;
uint8_t FEETECH_GOAL_POSITION; 
uint8_t FEETECH_MOVING_SPEED ;
uint8_t FEETECH_TORQUE_LIMIT ;
uint8_t FEETECH_PRESENT_POSITION ;
uint8_t FEETECH_PRESENT_SPEED ;
uint32_t BRGVALFEETECH ; 


void Init_Com_FEETECH(void){
    // initialise les IT de l'uart,
    
    FEETECH_VOLTAGE_TRIS = 0;
    FEETECH_VOLTAGE  = 1;  // choix entre 5v (feetech)=1 et 12v (dynamixel) =0 
    
    FEETECH_BUS_EN_TRIS     = 0;  // 
    FEETECH_BUS_EN          = 0;  // choix entre lecture et ecriture       
    FEETECH_BUS_TX_RX_TRIS = 1;
    FEETECH_EN_VMOT_TRIS    = 0;
    FEETECH_EN_VMOT         = 1;  //
  
    FEETECH_TORQUE_ENABLE = FEETECH_TORQUE_ENABLE;
    FEETECH_ID = FEETECH_ID;
    FEETECH_GOAL_POSITION = FEETECH_GOAL_POSITION_L;
    FEETECH_MOVING_SPEED = FEETECH_GOAL_SPEED_L;
    FEETECH_TORQUE_LIMIT = FEETECH_TORQUE_LIMIT_L;
    FEETECH_PRESENT_POSITION = FEETECH_PRESENT_POSITION_L;
    FEETECH_PRESENT_SPEED = FEETECH_PRESENT_SPEED_L;
};

/*************************************************
 * TX et RX Interrupt *
 *************************************************/

void __attribute__((__interrupt__, no_auto_psv)) _U2TXInterrupt(void) {
    // tant qu'il y a des trucs a mettre
    IFS1bits.U2TXIF = 0; // clear TX interrupt flag
    if (FEETECH_Transmit_Goal != FEETECH_Transmit_Ptr) {
        while ((FEETECH_Transmit_Goal != FEETECH_Transmit_Ptr) && (!U2STAbits.UTXBF)) {
            U2TXREG = FEETECH_Transmit_Tab[FEETECH_Transmit_Ptr];
            FEETECH_Transmit_Ptr++;
        }
    } else {
        // desactive l'IT TX, passe en RX, active l'IT RX
        Com_FEETECH_Status = COM_FEETECH_WAIT_ANSWER;
        _U2RXR = FEETECH_BUS_TX_RX_RP_NB;
        FEETECH_BUS_TX_RX_RP_REG = 0;
        FEETECH_BUS_EN = 0;
        uint8_t poubelle;
        while (U2STAbits.URXDA) {
            poubelle = U2RXREG;
        }
        // change d'IT possible
        IFS1bits.U2RXIF = 0;
        IEC1bits.U2TXIE = 0;
        IEC1bits.U2RXIE = 1;
    }
    Time_Of_Last_FEETECH_Received = Timer_ms1;
}

void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void) {
    uint16_t valrx;
    IFS1bits.U2RXIF = 0; // On baisse le FLAG
    while (U2STAbits.URXDA) { // tant que la FIFO n'est pas vide, on prend
        valrx = U2RXREG;
        FEETECH_Receive_Tab[FEETECH_Receive_Ptr] = valrx;
        if (FEETECH_Receive_Ptr < (FEETECH_CMD_BUFF_LENGTH - 1))
            FEETECH_Receive_Ptr++;
    }
    Time_Of_Last_FEETECH_Received = Timer_ms1;
}



void FEETECH_Loop(void){
    uint8_t val8, i;
    switch(FEETECH_Loop_State) {
/////////////////////////////////////////////////// repos ///////////////////////////////////////////////       
        case 0: //etat d'attente d'un envoie a effectuer
            if (Command_FEETECH_TODO != Command_FEETECH_DONE){
                FEETECH_Loop_State++;      
            }
            break;
        case 1: // check et initialisations
            FEETECH_Cmd_Nb_Try = 0;
            if (((Liste_Command_FEETECH[Command_FEETECH_DONE].Command == FEETECH_INST_READ_DATA) &&
                    (Liste_Command_FEETECH[Command_FEETECH_DONE].FEETECH_Addr != FEETECH_BROADCAST)) || // si ce nouvel ordre est supporte (read ou write)
                    (Liste_Command_FEETECH[Command_FEETECH_DONE].Command == FEETECH_INST_WRITE_DATA)){
                FEETECH_Loop_State = 10;
            } else { //si non supporte on le mets en status, et on passe a la suite
                *(Liste_Command_FEETECH[Command_FEETECH_DONE].Status) = FEETECH_STATUS_UNSUPORTED_CMD;
                FEETECH_Loop_State = 100;
            }
            break;
            
////////////////////////////////////////////////// en transmission ///////////////////////////////////////////////    
        case 10:    // debut de l'envoi
            FEETECH_Cmd_Send(&Liste_Command_FEETECH[Command_FEETECH_DONE]);
            FEETECH_Loop_State ++;
            break;
        case 11:
            // declenche l'IT TX :
            IFS1bits.U2TXIF = 1;
            IEC1bits.U2TXIE = 1;
            FEETECH_Loop_State = 20;
            break;

////////////////////////////////////////////////// en reception ///////////////////////////////////////////////
        case 20:    // attente de la reception
            if (Com_FEETECH_Status == COM_FEETECH_WAIT_ANSWER) {
                FEETECH_Loop_State++;
                Time_Of_Last_FEETECH_Received = Timer_ms1;
            }
            break;
        case 21:
            // si c'est un broadcast, on ne s'attend meme pas a recevoir quoique ce soit
            if (Liste_Command_FEETECH[Command_FEETECH_DONE].FEETECH_Addr == FEETECH_BROADCAST) {
                *(Liste_Command_FEETECH[Command_FEETECH_DONE].Status) = FEETECH_STATUS_OK;
                FEETECH_Loop_State = 100;
            // ou alors on a recu une trame de longueur coherente
            } else if ((FEETECH_Receive_Ptr > 3) && (FEETECH_Receive_Tab[3] == (FEETECH_Receive_Ptr - 4)) && U2STAbits.RIDLE) {
                FEETECH_Loop_State = 30;   // va l'analyser
            // ou si on a depasse le maxtime, pb
            } else if ((Timer_ms1 - Time_Of_Last_FEETECH_Received) > Com_FEETECH_Maxtime) {
                *(Liste_Command_FEETECH[Command_FEETECH_DONE].Status) = FEETECH_STATUS_TIMEOUT;
                FEETECH_Loop_State = 90;
            }
            break;
        case 30:
            // ici la trame a forcement la bonne longueur
            // verif du checksum
            val8 = 0;
            for (i = 2; i <= (FEETECH_Receive_Tab[3] + 2); i++)
                val8 += FEETECH_Receive_Tab[i];
            // verif si checksum est bon
            val8 = ~val8;
            if (val8 == FEETECH_Receive_Tab[FEETECH_Receive_Tab[3] + 3]) { // si le checksum en reception est bon
                *(Liste_Command_FEETECH[Command_FEETECH_DONE].Status) = FEETECH_STATUS_OK;
                FEETECH_Loop_State = 31;
            } else {
                *(Liste_Command_FEETECH[Command_FEETECH_DONE].Status) = FEETECH_STATUS_CHKSUM_ERROR;
                FEETECH_Loop_State = 90;
            }
            break;
        case 31:
            // transfert des donnees recues si c'est une lecture
            if ((Liste_Command_FEETECH[Command_FEETECH_DONE].Command == FEETECH_INST_READ_DATA) &&
                (Liste_Command_FEETECH[Command_FEETECH_DONE].Data_Answer != NULL)          ){ 
                
                uint8_t *ptr_on_u8 = Liste_Command_FEETECH[Command_FEETECH_DONE].Data_Answer;
                for (i = 0; i < Liste_Command_FEETECH[Command_FEETECH_DONE].Nb_Data; i++) {
                    ptr_on_u8[i] = FEETECH_Receive_Tab[i + 5];
                }
            }
            FEETECH_Loop_State = 100;
            break;
            
        case 90:
            // maxtime, ou checksum errror, relance si y a encore des essais a faire
            FEETECH_Cmd_Nb_Try ++;
            if (FEETECH_Cmd_Nb_Try < FEETECH_CMD_NB_MAX_TRY_SEND) {
                FEETECH_Loop_State = 10;   // relance la transmission
                U2MODEbits.UARTEN = 0;  // re eteind le module Uart
                Com_FEETECH_Status = COM_FEETECH_IDDLE;
            } else {
                FEETECH_Loop_State = 100;
            }
            break;
            
        case 100:
            U2MODEbits.UARTEN = 0;  // re eteind le module Uart
            IEC1bits.U2RXIE = 0;
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


void FEETECH_Cmd_Send(FEETECH_Command *Cmd) {
    uint8_t i, val8;
    // preparation de la trame
    FEETECH_Transmit_Tab [0] = 0xFF;
    FEETECH_Transmit_Tab [1] = 0xFF;
    FEETECH_Transmit_Tab [2] = Cmd->FEETECH_Addr;
    // la longueur du packet, dans le 3, apres :

    FEETECH_Transmit_Tab [4] = Cmd->Command;
    FEETECH_Transmit_Tab [5] = Cmd->Reg_Addr;

    if (Cmd->Command == FEETECH_INST_WRITE_DATA) {
        uint32_t Data_To_Send = Cmd->Data_To_Send;
        for (i = 0; i < Cmd->Nb_Data; i++) {
            FEETECH_Transmit_Tab [6 + i] = Data_To_Send & 0xFF;
            Data_To_Send = Data_To_Send >> 8;
        }
        FEETECH_Transmit_Tab [3] = Cmd->Nb_Data + 3; // ID + CMD + REG + x*datas
    } else if (Cmd->Command == FEETECH_INST_READ_DATA) {
        FEETECH_Transmit_Tab [6] = Cmd->Nb_Data;
        FEETECH_Transmit_Tab [3] = 4; // ID + CMD + Addr_Reg + Nb data to read
    }
    
    FEETECH_Transmit_Goal = FEETECH_Transmit_Tab [3] + 4; //  2*FF + Len + [] + chksum
    
    // calcul du checksum
    val8 = 0;
    for (i = 2; i <= (FEETECH_Transmit_Tab [3] + 2); i++) {
        val8 += FEETECH_Transmit_Tab[i];
    }
    FEETECH_Transmit_Tab[FEETECH_Transmit_Goal - 1] = ~val8;
    
    FEETECH_Transmit_Ptr = 0;
    FEETECH_Receive_Ptr = 0;
    


    
    FEETECH_BUS_EN = 1;
    FEETECH_BUS_TX_RX_RP_REG = _RPOUT_U2TX;
    
    _U2RXR = 0x1F;  // desactivtation de la partie RX
    //_U2RXR = AXBUS2_RX_RP_NB;
    
    // parametre et rallume l'uart
    IFS1bits.U2RXIF = 0;
    IFS1bits.U2TXIF = 0;
    IEC1bits.U2RXIE = 0;
    IEC1bits.U2TXIE = 0;
    U2BRG = Cmd->Uart_Brg;
//    printf("BRG : %04X\r\n", U2BRG);
    Com_FEETECH_Maxtime = 20;      // a adapter en focntion de la vitesse de transmission
    U2MODEbits.UARTEN = 1;      // rallume le module uart
    U2STAbits.UTXEN = 1;        // enable TX  (apres UARTEN, sinon marche pas)
        
    // purge buffer r�cepetion
    uint8_t poubelle;
    while (U2STAbits.URXDA) {
        poubelle = U2RXREG;
    }

    Com_FEETECH_Status = COM_FEETECH_SENDING;
    Time_Of_Last_FEETECH_Received = Timer_ms1;
    
}

uint8_t RegisterLenFEETECH(uint8_t address) {
    switch (address) {
        case FEETECH_MODEL_L: case FEETECH_MODEL_H: case FEETECH_ID: case FEETECH_BAUD_RATE: case FEETECH_DELAY_TIME_RETURN: case FEETECH_LEVEL_RETURN: case FEETECH_MAX_TEMP_LIMIT: case FEETECH_MAX_INPUT_VOLT:
        case FEETECH_MIN_INPUT_VOLT: case FEETECH_SETTING_BYTE: case FEETECH_PROTECTION_ENABLE: case FEETECH_ALARM_LED: case FEETECH_CW_DEAD: case FEETECH_CCW_DEAD:
        case FEETECH_RESOLUTION: case FEETECH_MODE: case FEETECH_TORQUE_ENABLE: case FEETECH_LOCK: case FEETECH_PRESENT_VOLTAGE:
        case FEETECH_ACC: case FEETECH_PRESENT_TEMPERATURE: case FEETECH_MOVING:
            return 1;
            break;
        case FEETECH_MIN_ANGLE_LIMIT_L:  case FEETECH_MAX_ANGLE_LIMIT_L: case FEETECH_MAX_TORQUE_LIMIT_L:  
        case FEETECH_OFS_L: case FEETECH_MIN_START_TORQUE: case FEETECH_OVERLOAD_CURRENT_L: case FEETECH_GOAL_POSITION_L: case FEETECH_GOAL_TIME_L: case FEETECH_GOAL_SPEED_L: 
        case FEETECH_TORQUE_LIMIT_L: case FEETECH_PRESENT_POSITION_L: case FEETECH_PRESENT_SPEED_L: case FEETECH_PRESENT_LOAD_L: case FEETECH_PRESENT_CURRENT_L:
            return 2;
            break;
        default:
            return 0; // Unknown register address
            break;
    }
}


void Add_FEETECH_Cmd(uint8_t FEETECH_Addr, uint16_t Uart_Brg, uint8_t FEETECH_Bus, uint8_t Command, uint8_t Reg_Addr, uint32_t Data_To_Send, void *Data_Answer, uint8_t Nb_Data, uint8_t *Status, void *Done) {
    Liste_Command_FEETECH[Command_FEETECH_TODO].Uart_Brg = Uart_Brg;
    Liste_Command_FEETECH[Command_FEETECH_TODO].FEETECH_Bus = FEETECH_Bus;
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
    Add_FEETECH_Cmd(id, BRGVALFEETECH, FEETECH_BUS2, FEETECH_INST_WRITE_DATA, Reg, Data, NULL, RegisterLenFEETECH(Reg), &FEETECH_Dumy, &FEETECH_Dumy);
}                    

void PutFEETECH_Wait(uint8_t id, uint8_t Reg, uint32_t Data) {
    volatile uint8_t Done = 0;
    Add_FEETECH_Cmd(id, BRGVALFEETECH, FEETECH_BUS2, FEETECH_INST_WRITE_DATA, Reg, Data, NULL, RegisterLenFEETECH(Reg), &FEETECH_Dumy, (void*) (&Done));
    while (!Done)
        FEETECH_Loop();
}

void PutFEETECH_Ext_Done(uint8_t id, uint8_t Reg, uint32_t Data, void *Done) {
    Add_FEETECH_Cmd(id, BRGVALFEETECH, FEETECH_BUS2, FEETECH_INST_WRITE_DATA, Reg, Data, NULL, RegisterLenFEETECH(Reg), &FEETECH_Dumy, Done);
}

uint32_t GetFEETECH_Wait(uint8_t id, uint8_t Reg) {
    volatile uint8_t Done = 0;
    uint32_t Data_Answer = 0;
    Add_FEETECH_Cmd(id, BRGVALFEETECH, FEETECH_BUS2, FEETECH_INST_READ_DATA, Reg, 0, &Data_Answer, RegisterLenFEETECH(Reg), &FEETECH_Dumy, (void*) (&Done));
    while (!Done)
        FEETECH_Loop();
    return Data_Answer;
}

void GetFEETECH(uint8_t id, uint8_t Reg, void *Data_Answer) {
    Add_FEETECH_Cmd(id, BRGVALFEETECH, FEETECH_BUS2, FEETECH_INST_READ_DATA, Reg, 0, Data_Answer, RegisterLenFEETECH(Reg), &FEETECH_Dumy, &FEETECH_Dumy);
}

void GetFEETECH_Ext_Done(uint8_t id, uint8_t Reg, void *Data_Answer, void *Done) {
    Add_FEETECH_Cmd(id, BRGVALFEETECH, FEETECH_BUS2, FEETECH_INST_READ_DATA, Reg, 0, Data_Answer, RegisterLenFEETECH(Reg), &FEETECH_Dumy, Done);
}

uint8_t FEETECH_All_Cmd_Done(void) {
    return (Command_FEETECH_TODO == Command_FEETECH_DONE);
}