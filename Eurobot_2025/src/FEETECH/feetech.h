#ifndef FEETECH_H
#define FEETECH_H


typedef unsigned char byte;
// Broadcast ID
#define FEETECH_H

#define FEETECH_BROADCAST               254

#define FEETECH_BUS1   1
#define FEETECH_BUS2   2

#define FEETECH_CMD_NB_MAX_TRY_SEND 3
#define FEETECH_CMD_LIST_SIZE 20
#define FEETECH_CMD_BUFF_LENGTH 20

#define FEETECH_STATUS_OK              0

#define FEETECH_STATUS_UNSUPORTED_CMD  0x81
#define FEETECH_STATUS_TIMEOUT         0x82
#define FEETECH_STATUS_CHKSUM_ERROR    0x83

//-------EPROM--------
#define  STS_MODEL_L 3
#define  STS_MODEL_H 4

//-------EPROM--------
#define  STS_ID 5
#define  STS_BAUD_RATE 6
#define  STS_DELAY_TIME_RETURN 7
#define  STS_LEVEL_RETURN 8
#define  STS_MIN_ANGLE_LIMIT_L 9
#define  STS_MIN_ANGLE_LIMIT_H 10
#define  STS_MAX_ANGLE_LIMIT_L 11
#define  STS_MAX_ANGLE_LIMIT_H 12
#define  STS_MAX_TEMP_LIMIT 13
#define  STS_MAX_INPUT_VOLT 14
#define  STS_MIN_INPUT_VOLT 15
#define  STS_MAX_TORQUE_LIMIT_L 16
#define  STS_MAX_TORQUE_LIMIT_H 17
#define  STS_SETTING_BYTE 18
#define  STS_PROTECTION_ENABLE 19
#define  STS_ALARM_LED 20
#define  STS_MIN_START_TORQUE 24
#define  STS_CW_DEAD 26
#define  STS_CCW_DEAD 27
#define  STS_OVERLOAD_CURRENT_L 28
#define  STS_OVERLOAD_CURRENT_H 29
#define  STS_RESOLUTION 30
#define  STS_OFS_L 31
#define  STS_OFS_H 32
#define  STS_MODE 33

//-------SRAM--------
#define  STS_TORQUE_ENABLE 40
#define  STS_ACC 41
#define  STS_GOAL_POSITION_L 42
#define  STS_GOAL_POSITION_H 43
#define  STS_GOAL_TIME_L 44
#define  STS_GOAL_TIME_H 45
#define  STS_GOAL_SPEED_L 46
#define  STS_GOAL_SPEED_H 47
#define  STS_TORQUE_LIMIT_L 48
#define  STS_TORQUE_LIMIT_H 49
#define  STS_LOCK 55

//-------SRAM(只读)--------
#define  STS_PRESENT_POSITION_L 56
#define  STS_PRESENT_POSITION_H 57
#define  STS_PRESENT_SPEED_L 58
#define  STS_PRESENT_SPEED_H 59
#define  STS_PRESENT_LOAD_L 60
#define  STS_PRESENT_LOAD_H 61
#define  STS_PRESENT_VOLTAGE 62
#define  STS_PRESENT_TEMPERATURE 63
#define  STS_MOVING 66
#define  STS_PRESENT_CURRENT_L 69
#define  STS_PRESENT_CURRENT_H 70

  
typedef struct {
    uint16_t Uart_Brg;  // vitesse 
    uint8_t FEETECH_Bus;     // 1 ou 2
    uint8_t FEETECH_Addr;
    uint8_t Command;
    uint8_t Reg_Addr;
    uint32_t Data_To_Send;
    void *Data_Answer;
    uint8_t Nb_Data; // Max 4 en send
    uint8_t *Status;
    void *Done;
} FEETECH_Command;


void Init_Com_FEETECH(void);

void FEETECH_Loop(void);

void FEETECH_Cmd_Send(FEETECH_Command *Cmd);

uint8_t RegisterLenFEETECH(uint8_t address);

void Add_FEETECH_Cmd(uint8_t FEETECH_Addr, uint16_t Uart_Brg, uint8_t FEETECH_Bus, uint8_t Command, uint8_t Reg_Addr, uint32_t Data_To_Send, void *Data_Answer, uint8_t Nb_Data, uint8_t *Status, void *Done);

void PutFEETECH(uint8_t id, uint8_t Reg, uint32_t Data);
void PutFEETECH_Wait(uint8_t id, uint8_t Reg, uint32_t Data); // interdit dans le robot, sauf pour d�bug !!!
void PutFEETECH_Ext_Done(uint8_t id, uint8_t Reg, uint32_t Data, void *Done);

void GetFEETECH(uint8_t id, uint8_t Reg, void *Data_Answer);
uint32_t GetFEETECH_Wait(uint8_t id, uint8_t Reg); // interdit dans le robot, sauf pour d�bug !!!
void GetFEETECH_Ext_Done(uint8_t id, uint8_t Reg, void *Data_Answer, void *Done);

uint8_t FEETECH_All_Cmd_Done(void);

#endif // FEETECH_H