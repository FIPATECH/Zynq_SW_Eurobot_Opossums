#ifndef FEETECH_ACTION_H
#define FEETECH_ACTION_H

uint8_t Send_FEETECH_Cmd(void);
uint8_t Get_FEETECH_Cmd(void);

uint8_t Send_FEETECH_SCS_Cmd(void);
uint8_t Get_FEETECH_SCS_Cmd(void);



void FEETECH_Search_ID_Loop(void);
uint8_t Test_ID_FEETECH_Cmd(void);

void FEETECH_action_loop(void);

void pince_action_loop(void);
uint8_t Monter_pince_cmd(void);
uint8_t Baisser_pince_cmd(void);
uint8_t Allumer_pompes_cmd(void);
uint8_t Eteindre_pompes_cmd(void);
uint8_t Activate_Valves_cmd(void);
uint8_t Ouvrir_clapet_cmd(void);

void Test_pince_action_loop(void);
uint8_t Test_pince_cmd(void);

#endif // FEETECH_ACTION_H