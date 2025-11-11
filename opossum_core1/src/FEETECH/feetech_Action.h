#ifndef FEETECH_ACTION_H
#define FEETECH_ACTION_H

uint8_t Send_FEETECH_Cmd(void);
uint8_t Get_FEETECH_Cmd(void);


void FEETECH_Search_ID_Loop(void);
uint8_t Test_ID_FEETECH_Cmd(void);

#endif // FEETECH_ACTION_H