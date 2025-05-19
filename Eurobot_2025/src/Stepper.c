#include "main.h"

STEPPER stepper_1;

STEPPER stepper[NBR_STEPPER];

// end stop switches
XGpio high_switch_elevator;
XGpio low_switch_elevator;

int high_switch_elevator_state = 0;
int previous_high_switch_elevator_state = 1;
int low_switch_elevator_state = 0;


void Init_Stepper(void){
    /////////////////////////////////////////////////////////////////////////////
    // init stepper motor
    /////////////////////////////////////////////////////////////////////////////            
    // create a stepper object
    stepper[0] = stepper_1;
    // initialize the stepper object
    XGpio_Initialize(&stepper_1.STEP,   STEP_DEVICE_ID);
    XGpio_Initialize(&stepper_1.SPEED,  SPEED_DEVICE_ID);
    XGpio_Initialize(&stepper_1.MODE,   MODE_DEVICE_ID);
    XGpio_Initialize(&stepper_1.DIR,    DIR_DEVICE_ID);
    XGpio_Initialize(&stepper_1.EN,     EN_DEVICE_ID);
    XGpio_Initialize(&stepper_1.DONE,   DONE_DEVICE_ID);
    XGpio_SetDataDirection(&stepper_1.STEP, 1, 0);
    XGpio_SetDataDirection(&stepper_1.SPEED, 1, 0);
    XGpio_SetDataDirection(&stepper_1.MODE, 1, 0);
    XGpio_SetDataDirection(&stepper_1.DIR, 1, 0);
    XGpio_SetDataDirection(&stepper_1.EN, 1, 0);
    XGpio_SetDataDirection(&stepper_1.DONE, 1, 1);
    stepper_1.step = 0; 
    stepper_1.speed =  1000; // 0 to 65535 in us between each steps
    stepper_1.mode = 0; // 0 to full step, 1 to half step, 2 to quarter step, 3 to eighth step, 4 to sixteenth step, 5 to thirty-second step, 6 to sixty-fourth step, 7 to one hundred twenty-eighth step
    stepper_1.dir = 1; // 0 to go forward, 1 to go backward
    stepper_1.en = 1; // 0 to enable, 1 to disable
    XGpio_DiscreteWrite(&stepper_1.SPEED, 1, stepper_1.speed);
    XGpio_DiscreteWrite(&stepper_1.MODE, 1, stepper_1.mode);
    XGpio_DiscreteWrite(&stepper_1.DIR, 1, stepper_1.dir);
    XGpio_DiscreteWrite(&stepper_1.EN, 1, stepper_1.en);
    XGpio_DiscreteWrite(&stepper_1.STEP, 1, stepper_1.step);
            

    ///////////////////////////////////////////////////////////////////////////////
    // init endstop switches
    ///////////////////////////////////////////////////////////////////////////////
    XGpio_Initialize(&high_switch_elevator, HIGH_SWITCH_ELEVATOR_DEVICE_ID);
    XGpio_SetDataDirection(&high_switch_elevator, 1, 1);
    // XGpio_SetDataDirection(&low_switch_elevator, 1, 1);
    high_switch_elevator_state = XGpio_DiscreteRead(&high_switch_elevator, 1);
    // low_switch_elevator_state = XGpio_DiscreteRead(&low_switch_elevator, 1);

}


int init_stepper_state = 0;
int init_stepper_done = 0;
int old_stepper_init_Timer_ms1 = 0;
int stepper_done = 0;



void Init_Stepper_Loop(void){
    if (init_stepper_done == 0){
        if(Timer_ms1 - old_stepper_init_Timer_ms1 > 10){
            old_stepper_init_Timer_ms1 = Timer_ms1;

            // read the state of the endstop switch
            high_switch_elevator_state = XGpio_DiscreteRead(&high_switch_elevator, 1);
            
            switch(init_stepper_state){
                case 0:
                    ///////////////////////////////////////////////////////////////////////////////
                    // move stepper motor to higher position to press the endstop switch
                    ///////////////////////////////////////////////////////////////////////////////
                    if(high_switch_elevator_state == 0){
                        stepper_1.en = STEPPER_ENABLE;
                        stepper_1.step += 10;
                        stepper_1.dir = STEPPER_DIR_FORWARD;
                        XGpio_DiscreteWrite(&stepper_1.DIR, 1, stepper_1.dir);
                        XGpio_DiscreteWrite(&stepper_1.STEP, 1, stepper_1.step);
                        XGpio_DiscreteWrite(&stepper_1.EN, 1, stepper_1.en);
                    }else{
                        init_stepper_state++;
                    }
                    break;
                case 1:
                    ///////////////////////////////////////////////////////////////////////////////
                    // wait for the endstop switch to be pressed
                    ///////////////////////////////////////////////////////////////////////////////
                    stepper_1.en = STEPPER_DISABLE;
                    stepper_1.step = 0;
                    XGpio_DiscreteWrite(&stepper_1.EN, 1, stepper_1.en);
                    XGpio_DiscreteWrite(&stepper_1.STEP, 1, stepper_1.step);
                    init_stepper_state++;
                    init_stepper_done = 1;
                    printf("stepper init done\n\r");
                    break;
                default:
                    break;
            }
        }
        
    }
}

uint32_t old_stepper_Timer_ms1 = 0;
uint32_t old_stepper_accel_Timer_ms1 = 0;
uint32_t test = 0;

int stepper_loop_state = 0;


void Stepper_Loop(void){
    if (init_stepper_done == 1){
        if(Timer_ms1 - old_stepper_Timer_ms1 > 10){
            old_stepper_Timer_ms1 = Timer_ms1;
            switch(stepper_loop_state){
                case 0:
                    ///////////////////////////////////////////////////////////////////////////////
                    // wait for new command
                    ///////////////////////////////////////////////////////////////////////////////
                    
                    break;
                case 10: // goes to higher position
                    ///////////////////////////////////////////////////////////////////////////////
                    // wait for the endstop switch to be pressed
                    ///////////////////////////////////////////////////////////////////////////////
                    high_switch_elevator_state = XGpio_DiscreteRead(&high_switch_elevator, 1);
                    if (high_switch_elevator_state == 0){
                        stepper_1.en = STEPPER_ENABLE;
                        stepper_1.step += 10;
                        stepper_1.dir = STEPPER_DIR_FORWARD;
                        XGpio_DiscreteWrite(&stepper_1.DIR, 1, stepper_1.dir);
                        XGpio_DiscreteWrite(&stepper_1.STEP, 1, stepper_1.step);
                        XGpio_DiscreteWrite(&stepper_1.EN, 1, stepper_1.en);
                    }else{
                        stepper_1.en = STEPPER_DISABLE;
                        stepper_1.step = 0;
                        XGpio_DiscreteWrite(&stepper_1.EN, 1, stepper_1.en);
                        XGpio_DiscreteWrite(&stepper_1.STEP, 1, stepper_1.step);
                        printf("STEPPER DONE\n\r");
                        stepper_loop_state = 0;
                    }
                    break;
                case 20: // goes to lower position
                    ///////////////////////////////////////////////////////////////////////////////
                    // check the state of the endstop switch
                    ///////////////////////////////////////////////////////////////////////////////
                    high_switch_elevator_state = XGpio_DiscreteRead(&high_switch_elevator, 1);
                    if (high_switch_elevator_state == 0){
                        printf("STEPPERERROR 1 \n\r");
                        stepper_loop_state = 0;
                    } else{
                        stepper_1.en = STEPPER_ENABLE;
                        stepper_1.step = 200;
                        stepper_1.dir = STEPPER_DIR_BACKWARD;
                        XGpio_DiscreteWrite(&stepper_1.DIR, 1, stepper_1.dir);
                        XGpio_DiscreteWrite(&stepper_1.EN, 1, stepper_1.en);
                        XGpio_DiscreteWrite(&stepper_1.STEP, 1, stepper_1.step);
                        stepper_loop_state ++;
                    }
                    break;
                case 11:
                    stepper_done = XGpio_DiscreteRead(&stepper_1.DONE, 1);
                    if(stepper_done == 1){
                        stepper_1.en = STEPPER_DISABLE;
                        XGpio_DiscreteWrite(&stepper_1.EN, 1, stepper_1.en);
                        printf("STEPPER DONE\n\r");
                        stepper_loop_state = 0;
                    }
                    
                default:
                    break;
            }
        }
    }
}

void Set_Stepper_Mode(STEPPER *stepper, uint8_t mode){
    if (mode > 7){
        printf("Mode out of range\n\r");
        return;
    }else{
        stepper->mode = (uint8_t)mode;
        XGpio_DiscreteWrite(&stepper->MODE, 1, stepper->mode);
    }
}

void Set_Stepper_Step(STEPPER *stepper, uint32_t step){
    if (step > 65535){
        printf("Step out of range\n\r");
        return;
    }else{
        stepper->step = (uint16_t)step;
        XGpio_DiscreteWrite(&stepper->STEP, 1, stepper->step);
    }
}

void Set_Stepper_Speed(STEPPER *stepper, uint32_t speed){
    if (speed > 65535){
        printf("Speed out of range\n\r");
        return;
    }else{
        stepper->speed = (uint16_t)speed;
        XGpio_DiscreteWrite(&stepper->SPEED, 1, stepper->speed);
    }
}

void Set_Stepper_Dir(STEPPER *stepper, uint8_t dir){
    if (dir > 1){
        printf("Dir out of range\n\r");
        return;
    }else{
        stepper->dir = (uint8_t)dir;
        XGpio_DiscreteWrite(&stepper->DIR, 1, stepper->dir);
    }
}

void Set_Stepper_En(STEPPER *stepper, uint8_t en){
    if (en > 1){
        printf("En out of range\n\r");
        return;
    }else{
        // stepper->en = (uint8_t)en;
        XGpio_DiscreteWrite(&stepper_1.EN, 1, stepper_1.en);
    }
}

uint8_t Stepper_help_cmd(void){
    printf("This command takes three parameters: \n\r");
    printf("1. Stepper ID between 1 and 2\n\r");
    printf("2. Mode: 1 to set step, 2 to set speed, 3 to set mode, 4 to set direction, 5 to set enable\n\r");
    printf("3. Value to set\n\r");
    return 0;
}


uint8_t Stepper_cmd(void){
    u32 id;
    if (Get_Param_u32(&id)){
        return PARAM_ERROR_CODE;
    }
    u32 mode;
    if (Get_Param_u32(&mode)){
        return PARAM_ERROR_CODE;
    }
    u32 val;
    if (Get_Param_u32(&val)){
        return PARAM_ERROR_CODE;
    }
    if(mode == 1){
        if (id > 2){
            return PARAM_OUT_OF_RANGE_ERROR_CODE;
        }
        Set_Stepper_Step(&stepper[id-1], val);
    } else if(mode == 2){
        if (id > 2){
            return PARAM_OUT_OF_RANGE_ERROR_CODE;
        }
        Set_Stepper_Speed(&stepper[id-1], val);
    } else if(mode == 3){
        if (id > 2){
            return PARAM_OUT_OF_RANGE_ERROR_CODE;   
        }
        Set_Stepper_Mode(&stepper[id-1], val);
    } else if(mode == 4){
        if (id > 2){    
            return PARAM_OUT_OF_RANGE_ERROR_CODE;
        }
        Set_Stepper_Dir(&stepper[id-1], val);
    } else if(mode == 5){
        if (id > 2){
            return PARAM_OUT_OF_RANGE_ERROR_CODE;
        }
        Set_Stepper_En(&stepper[id-1], val);
    }
    return 0;
}

uint8_t Stepper_1_cmd(void){
    u32 mode;
    if (Get_Param_u32(&mode)){
        return PARAM_ERROR_CODE;
    }
    if (mode == 1){
        stepper_loop_state = 10;
    } else if (mode == 2){
        stepper_loop_state = 20;
    } else {
        return PARAM_OUT_OF_RANGE_ERROR_CODE;
    }
    return 0;
}