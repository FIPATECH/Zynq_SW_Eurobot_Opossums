
#define AXI_PWM_BASEADDR 0x41220000

#define NBR_SERVO 1

#define DEFAULT_ANGLE_MIN 0
#define DEFAULT_ANGLE_MAX 180
#define DEFAULT_ANGLE 90

typedef struct{
    int axi_addr;
    int default_pos;
    int current_pos;
    int min_pos;
    int max_pos;
    int pos;
    XGpio gpio;
    int to_do;
} Servo;

void Init_Servo(Servo *servo, int axi_addr, int default_pos, int min_pos, int max_pos, int to_do);
void PWM_Init(void);
void PWM_Loop(void);