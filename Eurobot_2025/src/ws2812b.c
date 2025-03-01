#include "main.h"

LED led[NBR_LED];
LED color;

// XGpio LED_ADDR;
XGpio LED_DATA;

int LED_old_timer_ms1 = 0;

void ws2812b_init(){
    // XGpio_Initialize(&LED_ADDR, XPAR_AXI_GPIO_24_DEVICE_ID);
    XGpio_Initialize(&LED_DATA, XPAR_AXI_GPIO_25_DEVICE_ID);

    // XGpio_SetDataDirection(&LED_ADDR, 1, 0);
    XGpio_SetDataDirection(&LED_DATA, 1, 0);

    for (int i = 0; i < NBR_LED; i++){
        led[i].red = 0;
        led[i].green = 0;
        led[i].blue = 0;
    }
}

void ws2812b_set_color(int led_id, LED color){

    if (led_id < 0 || led_id > NBR_LED){
        xil_printf("LED id out of range\n\r");
    }else{
        led[led_id] = color;   
    }
}


uint32_t led_id = 0;
uint32_t led_color = 0;

void LED_loop(){
    if (Timer_ms1 - LED_old_timer_ms1 > 1){
        LED_old_timer_ms1 = Timer_ms1;

        // ws2812b_set_color(led_id, led[led_id]);
        // XGpio_DiscreteWrite(&LED_ADDR, 1, led_id);
        XGpio_DiscreteWrite(&LED_DATA, 1, led_color);
            
        
    }
}

void LED_AU(void){
    led_color = DEFAULT_RED;
}

void LED_GREEN(void){
    led_color = DEFAULT_GREEN;
}

uint8_t LED_cmd(void) {
    u32 id;
    if (Get_Param_u32(&id)){
        return PARAM_ERROR_CODE;
    }
	u32 red;
    if (Get_Param_u32(&red)){
        return PARAM_ERROR_CODE;
    }
    u32 green;
    if (Get_Param_u32(&green)){
        return PARAM_ERROR_CODE;
    }
    u32 blue;
    if (Get_Param_u32(&blue)){
        return PARAM_ERROR_CODE;
    }
    if (red < 0 || red > 255 || green < 0 || green > 255 || blue < 0 || blue > 255){
        xil_printf("Color out of range\n\r");
    }else {
        xil_printf("LED number %d, color: %d, %d, %d\n\r", id, red, green, blue);
        led_color = (green << 16) | (red << 8) | blue;
    }
    return 0;
}

