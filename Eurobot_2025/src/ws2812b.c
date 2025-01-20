#include "main.h"

LED led[NBR_LED];
LED color;

int LED_old_timer_ms1 = 0;

void ws2812b_init(){
    for (int i = 0; i < NBR_LED; i++){
        if (i % 2 == 0){
            led[i].red = 255;
            led[i].green = 0;
            led[i].blue = 0;
        }else{
            led[i].red = 0;
            led[i].green = 255;
            led[i].blue = 0;
        }
    }
}

void ws2812b_set_color(int led_id, LED color){
    volatile uint32_t* ws2812 = (uint32_t*)WS2812_BASEADDR;
    if (led_id < 0 || led_id > NBR_LED){
        xil_printf("LED id out of range\n\r");
    }else{
        uint32_t led_color = (color.red << 16) | (color.green << 8) | color.blue;
        ws2812[led_id] = led_color;
    }
}
void start_transfer(){
    volatile uint32_t* ws2812 = (uint32_t*)WS2812_BASEADDR;
    ws2812[NBR_LED] = 1;
}



void LED_loop(){
    if (Timer_ms1 - LED_old_timer_ms1 > 10){
        LED_old_timer_ms1 = Timer_ms1;
        for (int i = 0; i < NBR_LED; i++){
            ws2812b_set_color(i, led[i]);
        }
        start_transfer();
    }
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
        led[id].red = red;
        led[id].green = green;
        led[id].blue = blue;
    }
    return 0;
}


int AU_led_counter = 0;
int AU_led_old_timer_ms1 = 0;
int on_off_loop_status = 0;
// chenillard led rouge 
void AU_led_loop(){
    if (Timer_ms1 - AU_led_old_timer_ms1 >= 100) {
        xil_printf("AU_led_loop\n\r");
        AU_led_old_timer_ms1 = Timer_ms1;
        if (on_off_loop_status == 0){
            led[AU_led_counter].red = 0;
            led[AU_led_counter].green = 0;
            led[AU_led_counter].blue = 0;
            ws2812b_set_color(AU_led_counter, led[AU_led_counter]);
            AU_led_counter++;
            if (AU_led_counter == NBR_LED - 1){
                on_off_loop_status = 1;
                AU_led_counter = 0;
            }
        }else{
            led[AU_led_counter].red = 255;
            led[AU_led_counter].green = 0;
            led[AU_led_counter].blue = 0;
            ws2812b_set_color(AU_led_counter, led[AU_led_counter]);
            AU_led_counter++;
            if (AU_led_counter == NBR_LED - 1){
                on_off_loop_status = 0;
                AU_led_counter = 0;
            }
        }
    }
}