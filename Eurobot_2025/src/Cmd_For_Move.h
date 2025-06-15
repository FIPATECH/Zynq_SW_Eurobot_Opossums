
#ifndef __CMD_FOR_MOVE_H
#define __CMD_FOR_MOVE_H


uint8_t Move_Cmd(void);
uint8_t Speed_Cmd(void);

uint8_t Absolute_Speed_Cmd(void);

uint8_t FREE_Cmd(void);
uint8_t BLOCK_Cmd(void);
uint8_t Asserv_Done_Cmd(void);
uint8_t Get_Pos_Cmd(void);
uint8_t Get_Odo_Cmd(void);
uint8_t Get_Speed_Wheel_Cmd(void);
uint8_t SET_Cmd(void);
uint8_t Set_Lidar_Cmd(void);
uint8_t VMAX_Cmd(void);
uint8_t VTMAX_Cmd(void);
uint8_t AMAX_Cmd(void);
uint8_t PWM_Func(void);

#endif