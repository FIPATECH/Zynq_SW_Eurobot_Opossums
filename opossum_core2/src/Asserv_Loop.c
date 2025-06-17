#include "main.h"
#include "lib_asserv/Lib_Asserv.h"

u32 Last_Timer_print_pos = 0;

uint8_t auto_printpos_en = 1;
uint16_t auto_printpos_delay = 100;

uint8_t Debug_Timing = 0;

uint16_t Asserv_Full_Count = 0;

CAN_Message CAN_Motor1;
CAN_Message CAN_Motor2;
CAN_Message CAN_Motor3;

uint8_t Channel_Motor1 = 0;
uint8_t Channel_Motor2 = 1;
uint8_t Channel_Motor3 = 2;

int16_t Rotor_RPM1 = 0;
int16_t Rotor_RPM2 = 0;
int16_t Rotor_RPM3 = 0;

float wheel_speed1 = 0;
float wheel_speed2 = 0;
float wheel_speed3 = 0;

uint8_t stop = 0;

Position position_lidar;

int Last_Timer_Asserv = 0;
int Asserv_State = 0;
int Asserv_Odo_Count = 0;

ESC_Command Consigne;
ESC_Command Wanted_Forced_Consigne;
ESC_Command old_Consigne;

int Lidar_inconsistency_count = 0;

int enable_kalman = 1;
int kalman_initialized = 0;

float dx, dy, dt = 0;

int lidar_delay = 0; // délai de la dernière mesure lidar

int tampon;

void Init_Asserv(void) {
    Consigne.command1 = 0;
    Consigne.command2 = 0;
    Consigne.command3 = 0;

    Wanted_Forced_Consigne.command1 = 0;
    Wanted_Forced_Consigne.command2 = 0;
    Wanted_Forced_Consigne.command3 = 0;

    old_Consigne.command1 = 0;
    old_Consigne.command2 = 0;
    old_Consigne.command3 = 0;

    enable_kalman = 1;

    asserv_init();
}

void Asserv_Loop(void)
{
	if (Asserv_State == 0) {
        //-----------------------------------
        // ODO step 1:
        // - calcul de la vitesse du robot 
        //-----------------------------------
        if ((Timer_ms1 - Last_Timer_Asserv) > ODO_EVERY_MS) {
                        #ifdef DEBUG_TIMING
                        tampon = Timer_us1;
                        #endif
            Last_Timer_Asserv += ODO_EVERY_MS;
            odo_speed_step(speed_motor_1, speed_motor_2, speed_motor_3);       
            Asserv_State = 2;
                        #ifdef DEBUG_TIMING
                        local_data.asserv_step_timing.odo_step_1 = Timer_us1 - tampon;
                        #endif
        }

    } else if (Asserv_State == 2) {
        // -----------------------------------
        // ODO step 2:
        // - calcul du kalman + history
        // -----------------------------------
                        #ifdef DEBUG_TIMING
                        tampon = Timer_us1;
                        #endif
        kalman_predict(&kalman_current_state, &speed_robot_odom, ODO_EVERY_MS*0.001f);
        kalman_fifo_push(&kalman_fifo, &kalman_current_state, &speed_robot_odom);
        Asserv_Odo_Count ++;
                        #ifdef DEBUG_TIMING
                        local_data.asserv_step_timing.odo_step_2 = Timer_us1 - tampon;
                        // printf("Timer_us1: %d, %d\n\r", tampon, Timer_us1);
                        #endif

        if (Asserv_Odo_Count >= ASSERV_EVERY){
            Asserv_Odo_Count = 0;
            Asserv_State ++;
        } else {
            Asserv_State = 0;
        }

    } else if (Asserv_State == 3) {
        // -----------------------------------
        // ODO step 3:
        // - calcul de la vitesse du robot
        // -----------------------------------
                        #ifdef DEBUG_TIMING
                        tampon = Timer_us1;
                        #endif
        odo_speed_cumulate_step(ASSERV_EVERY);
                        #ifdef DEBUG_TIMING
                        local_data.asserv_step_timing.odo_step_3 = Timer_us1 - tampon;
                        #endif
        Asserv_State = 10;
    
    } else if (Asserv_State == 10) {
        //-----------------------------------
        // motion step
        //-----------------------------------
                        #ifdef DEBUG_TIMING
                        tampon = Timer_us1;
                        #endif
        motion_step();
                        #ifdef DEBUG_TIMING
                        local_data.asserv_step_timing.motion_step = Timer_us1 - tampon;
                        #endif
        Asserv_State = 20;

    } else if (Asserv_State == 20) {
        //-----------------------------------
        // spped constrain
        //-----------------------------------
                        #ifdef DEBUG_TIMING
                        tampon = Timer_us1;
                        #endif
        constrain_speed_order();
                        #ifdef DEBUG_TIMING
                        local_data.asserv_step_timing.speed_constrain_step = Timer_us1 - tampon;
                        #endif
        Asserv_State = 21;

    } else if (Asserv_State == 21) {
        //-----------------------------------
        // acceleration constrain
        //-----------------------------------
                        #ifdef DEBUG_TIMING
                        tampon = Timer_us1;
                        #endif
        constrain_acceleration_order(ASSERV_EVERY*ODO_EVERY_MS*0.001f);
                        #ifdef DEBUG_TIMING
                        local_data.asserv_step_timing.acceleration_constrain_step = Timer_us1 - tampon;
                        #endif
        Asserv_State = 30;

    } else if (Asserv_State == 30) {
        //-----------------------------------
        // consigne
        //-----------------------------------
                        #ifdef DEBUG_TIMING
                        tampon = Timer_us1;
                        #endif
        Asserv_PWM_calculator(&Consigne);
                        #ifdef DEBUG_TIMING
                        local_data.asserv_step_timing.consigne_step = Timer_us1 - tampon;
                        #endif
        Asserv_State = 40;


    } else if (Asserv_State == 40) {
        if ((Wanted_Forced_Consigne.command1 != 0) || 
            (Wanted_Forced_Consigne.command2 != 0) || 
            (Wanted_Forced_Consigne.command3 != 0)) {

            Consigne = Wanted_Forced_Consigne;
        }
        
        float Abs_Consigne1 = Abs_Ternaire(Consigne.command1);
        float Abs_Consigne2 = Abs_Ternaire(Consigne.command2);
        float Abs_Consigne3 = Abs_Ternaire(Consigne.command3);
        
        float Consigne_Max = Max_Trois(Abs_Consigne1, Abs_Consigne2, Abs_Consigne3);

        if (Consigne_Max > 10000) {
            float Consigne_rapport = 10000.0/Consigne_Max;
            Consigne.command1 *= Consigne_rapport;
            Consigne.command2 *= Consigne_rapport;
            Consigne.command3 *= Consigne_rapport;
        }

        old_Consigne = Consigne;
       
        Asserv_State = 50;

    } else if (Asserv_State == 50) {
        if(AU_state){
            asserv_off_step();
        }else{
            motor1_current_order = Consigne.command1;
            motor2_current_order = Consigne.command2;
            motor3_current_order = Consigne.command3;
        }
        CAN_transmit_motor(motor1_current_order, motor2_current_order, motor3_current_order);
        Asserv_State = 60;

    } else if (Asserv_State == 60) {

        local_data.kalman_out.x = kalman_current_state.x[0];
        local_data.kalman_out.y = kalman_current_state.x[1];
        local_data.kalman_out.t = kalman_current_state.x[2];
        SEND_FIELD(&local_data, kalman_out);
        local_data.speed_robot = speed_robot;
        SEND_FIELD(&local_data, speed_robot);

        SEND_FIELD(&local_data, asserv_step_timing);
        Asserv_State = 0;
        
        #ifdef DEBUG_TIMING
        SEND_FIELD(shared_mem, asserv_step_timing);
        #endif

    } else {
        Asserv_State = 0;
    }
}

void Set_Lidar_Cmd(Set_lidar set_lidar) {
    tampon = Timer_us1;
    // Vérification de la cohérence des données LIDAR
    if(set_lidar.delay < 0 || set_lidar.delay > 200) {
        printf("ERROR: Lidar delay out of range\n");
        return; // erreur
    }

    Position position_lidar;
    position_lidar.x = set_lidar.lidar_position_x;
    position_lidar.y = set_lidar.lidar_position_y;
    position_lidar.t = set_lidar.lidar_position_t;

    if(enable_kalman) {
        if(!kalman_initialized){
            // Initialisation du filtre de Kalman avec les données LIDAR
            kalman_init_with_lidar(&kalman_fifo, &position_lidar);
            kalman_initialized = 1;
        }else{
            // Récupération de l'index dans la FIFO correspondant au délai LiDAR
            int delay_index = kalman_fifo_get_delay(&kalman_fifo, set_lidar.delay, 1);
            if (delay_index < 0) {
                return; // erreur
            }
            
            // Correction de l’état dans la FIFO
            float z[3] = {position_lidar.x, position_lidar.y, position_lidar.t};
            kalman_update(&kalman_fifo.buffer[delay_index], z);

            // Repropagation depuis l’état corrigé
            kalman_fifo_repropagate(&kalman_fifo, delay_index, 0.001f);

            // Mise à jour de l’état courant
            kalman_current_state = kalman_fifo.buffer[(kalman_fifo.head - 1 + KALMAN_FIFO_LEN) % KALMAN_FIFO_LEN];
        }
    }
    int tampon2 = Timer_us1 - tampon;
    printf("timer : %d\n", tampon2);
}

