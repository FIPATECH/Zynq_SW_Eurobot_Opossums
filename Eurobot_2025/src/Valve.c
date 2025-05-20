#include "main.h"

XGpio Valve;
uint8_t valve_state[4];
uint8_t previous_valve_state[4];
uint32_t Timer_valve_on[4];

uint8_t valve_cmd = 0;

void Init_Pump(void){
    XGpio_Initialize(&Valve, XPAR_AXI_GPIO_17_DEVICE_ID);
    XGpio_SetDataDirection(&Valve, 1, 0);
    for (int i = 0; i < 4; i++){
        valve_state[i] = 0;
        previous_valve_state[i] = 0;
        Timer_valve_on[i] = 0;
    }
}

uint32_t old_valve_Timer_ms1 = 0;


void Pump_Loop(void){
    if(Timer_ms1 - old_valve_Timer_ms1 > 10){
        old_valve_Timer_ms1 = Timer_ms1;

        for (int i = 0; i < 4; i++){
            // check if the valve state has changed and keep track of the time it was turned on
            if (valve_state[i] != previous_valve_state[i] && valve_state[i] == 1){
                Timer_valve_on[i] = Timer_ms1;
            }

            if(Timer_ms1 - Timer_valve_on[i] > 1000 && valve_state[i] == 1){
                // if the valve has been on for more than 1 second, turn it off
                valve_state[i] = 0;
            }
        }

        valve_cmd = valve_state[0] | (valve_state[1] << 1) | (valve_state[2] << 2) | (valve_state[3] << 3);
        XGpio_DiscreteWrite(&Valve, 1, valve_cmd);
    }
}

uint8_t Valve_cmd(void){
    u32 id;
    if (Get_Param_u32(&id)){
        return PARAM_ERROR_CODE;
    }
	u32 state;
    if (Get_Param_u32(&state)){
        return PARAM_ERROR_CODE;
    }
    xil_printf("Valve number %d, state: %d\n\r", id, state);
    valve_state[id-1] = state;
    return 0;
}