#include "main.h"

Servo servo[NBR_SERVO];

int old_Timer_ms1 = 0;

void Init_Servo(Servo *servo, int axi_addr, int default_pos, int min_pos, int max_pos, int to_do){
    servo->axi_addr     = axi_addr;
    servo->default_pos  = default_pos;
    servo->min_pos      = min_pos;
    servo->max_pos      = max_pos;
    servo->pos          = default_pos;
    servo->current_pos  = default_pos;
    servo->to_do        = to_do;
}

void PWM_Init(void)
{
    int gpio_id = XPAR_AXI_GPIO_2_DEVICE_ID; //change XPAR_AXI_GPIO_2_DEVICE_ID with the correct ID when all pwm will be created
    for (int i = 0; i < NBR_SERVO; i++){
        gpio_id = XPAR_AXI_GPIO_2_DEVICE_ID + i;
        Init_Servo(&servo[i], gpio_id, DEFAULT_ANGLE, DEFAULT_ANGLE_MIN, DEFAULT_ANGLE_MAX, 0); 
        XGpio_Initialize(&servo[i].gpio, servo[i].axi_addr);
	    XGpio_SetDataDirection(&servo[i].gpio, 1, 0);
    }
}


int pwm_loop_state = 0;

void PWM_Loop(void){
    switch (pwm_loop_state){
        case 0:
            if (Timer_ms1 - old_Timer_ms1 > 10){
                pwm_loop_state = 1;
            }
            break;
        case 1:
            for (int i = 0; i < NBR_SERVO; i++){
                if(servo[i].to_do == 1){
                    XGpio_DiscreteWrite(&servo[i].gpio, 1, servo[i].pos);
                    servo[i].to_do = 0;
                }
            }
            pwm_loop_state = 0;
            break;
    }
}
