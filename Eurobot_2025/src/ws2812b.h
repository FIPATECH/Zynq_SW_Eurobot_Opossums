#define WS2812_BASEADDR XPAR_WS2812B_AXI_CONTROLL_0_BASEADDR

#define NBR_LED 44

#define DEFAULT_RED 0xFF0000
#define DEFAULT_GREEN 0x00FF00
#define DEFAULT_BLUE 0x0000FF

typedef struct{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
}LED;

void ws2812b_init();

void ws2812b_set_color(int led_id, LED color);
void start_transfer();

void LED_loop();

uint8_t LED_cmd(void);

void AU_led_loop();