#ifndef __STEPPER_H
#define __STEPPER_H

#define STEP_DEVICE_ID  XPAR_AXI_GPIO_11_DEVICE_ID
#define SPEED_DEVICE_ID XPAR_AXI_GPIO_12_DEVICE_ID
#define MODE_DEVICE_ID  XPAR_AXI_GPIO_13_DEVICE_ID
#define DIR_DEVICE_ID   XPAR_AXI_GPIO_14_DEVICE_ID
#define EN_DEVICE_ID    XPAR_AXI_GPIO_15_DEVICE_ID
#define DONE_DEVICE_ID  XPAR_AXI_GPIO_16_DEVICE_ID

#define HIGH_SWITCH_ELEVATOR_DEVICE_ID XPAR_AXI_GPIO_24_DEVICE_ID
#define LOW_SWITCH_ELEVATOR_DEVICE_ID  XPAR_AXI_GPIO_27_DEVICE_ID

#define NBR_STEPPER 1

#define STEPPER_ENABLE 0
#define STEPPER_DISABLE 1

#define STEPPER_MODE_FULL_STEP 0
#define STEPPER_MODE_HALF_STEP 1
#define STEPPER_MODE_QUARTER_STEP 2
#define STEPPER_MODE_EIGHTH_STEP 3
#define STEPPER_MODE_SIXTEENTH_STEP 4
#define STEPPER_MODE_THIRTY_SECOND_STEP 5
#define STEPPER_MODE_SIXTY_FOURTH_STEP 6
#define STEPPER_MODE_ONE_HUNDRED_TWENTY_EIGHTH_STEP 7

#define STEPPER_DIR_FORWARD 0
#define STEPPER_DIR_BACKWARD 1


typedef struct {
    XGpio STEP;
    XGpio SPEED;
    XGpio MODE;
    XGpio DIR;
    XGpio EN;
    XGpio DONE;
    uint16_t step;
    uint16_t speed;
    uint8_t mode;
    uint8_t dir;
    uint8_t en;
} STEPPER;



void Init_Stepper(void);
void Stepper_Loop(void);

void Set_Stepper_Mode(STEPPER *stepper, uint8_t mode);
void Set_Stepper_Step(STEPPER *stepper, uint32_t step);
void Set_Stepper_Speed(STEPPER *stepper, uint32_t speed);
void Set_Stepper_Dir(STEPPER *stepper, uint8_t dir);
void Set_Stepper_En(STEPPER *stepper, uint8_t en);

uint8_t Stepper_cmd(void);
uint8_t Stepper_help_cmd(void);

#endif