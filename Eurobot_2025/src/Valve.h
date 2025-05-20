#ifndef __VALVE_H
#define __VALVE_H

extern uint8_t valve_state[4];

void Init_Valve(void);
void Valve_Loop(void);

uint8_t Valve_cmd(void);


#endif