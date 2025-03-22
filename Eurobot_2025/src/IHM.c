#include "main.h"

XGpio YELLOW_SWITCH;
XGpio GREEN_SWITCH;
XGpio BLUE_SWITCH;

int yellow_state;
int green_state; 
int blue_state;

int previous_yellow_state;
int previous_green_state;
int previous_blue_state;

int current_mode = 0;

void init_switch(void){
    XGpio_Initialize(&YELLOW_SWITCH, XPAR_AXI_GPIO_23_DEVICE_ID);
    XGpio_Initialize(&GREEN_SWITCH, XPAR_AXI_GPIO_21_DEVICE_ID);
    XGpio_Initialize(&BLUE_SWITCH, XPAR_AXI_GPIO_22_DEVICE_ID);

    XGpio_SetDataDirection(&YELLOW_SWITCH, 1, 1);
    XGpio_SetDataDirection(&GREEN_SWITCH, 1, 1);
    XGpio_SetDataDirection(&BLUE_SWITCH, 1, 1);
}

int validation_blue = 0;
int validation_yellow = 0;

void IHM_loop(void){
    yellow_state = XGpio_DiscreteRead(&YELLOW_SWITCH, 1);
    green_state = XGpio_DiscreteRead(&GREEN_SWITCH, 1);
    blue_state = XGpio_DiscreteRead(&BLUE_SWITCH, 1);

    if (validation_blue == 1 && validation_state ==1){
        current_mode = 20;
        validation_blue = 0;
        validation_state = 0;
    }

    if (validation_yellow == 1 && validation_state ==1){
        current_mode = 30;
        validation_yellow = 0;
        validation_state = 0;
    }



    if (yellow_state != previous_yellow_state){
        previous_yellow_state = yellow_state;
        if(yellow_state == 0){
            if(current_mode == 1){ // start selection color
                current_mode = 10;
                validation_blue = 1;
            } else if (current_mode == 20){ //abort selection color
                current_mode = 1;
            }
        }
    }
    if (green_state != previous_green_state){
        previous_green_state = green_state;
        if(green_state == 0){
            if(current_mode == 0){ // start selection color
                current_mode = 1;
            } else if (current_mode == 1){ //abort selection color
                current_mode = 0;
            }
        }
    }
    if (blue_state != previous_blue_state){
        previous_blue_state = blue_state;
        if(blue_state == 0){
            if(current_mode == 1){ // start selection color
                current_mode = 10;
                validation_yellow = 1;
            }else if(current_mode == 30){ //abort selection color
                current_mode = 1;
            }
        }
    }
}