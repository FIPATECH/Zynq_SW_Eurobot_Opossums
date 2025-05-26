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

    // xil_printf("Asserv Init done\n");
}

void Asserv_Loop(void)
{
	if (Asserv_State == 0) {
        //-----------------------------------
        // ODO step 1:
        // - calcul de la vitesse du robot 
        //-----------------------------------
        if ((Timer_ms1 - Last_Timer_Asserv) > ODO_EVERY_MS) {
            // if((Timer_ms1 - Last_Timer_Asserv) > 2*ODO_EVERY_MS){
            //     printf("ERROR ODO TIMEOUT");
            // }
            Last_Timer_Asserv += ODO_EVERY_MS;
            odo_speed_step(speed_motor_1, speed_motor_2, speed_motor_3);       
            Asserv_State = 2;
        }

    } else if (Asserv_State == 2) {
        // -----------------------------------
        // ODO step 2:
        // - calcul du kalman + history
        // -----------------------------------
        kalman_predict(&kalman_current_state, &speed_robot_odom, ODO_EVERY_MS*0.001f);
        kalman_fifo_push(&kalman_fifo, &kalman_current_state, &speed_robot_odom);
        Asserv_Odo_Count ++;

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
        odo_speed_cumulate_step(ASSERV_EVERY);
        Asserv_State = 10;
    
    } else if (Asserv_State == 10) {
        //-----------------------------------
        // motion step
        //-----------------------------------
        motion_step();
        Asserv_State = 20;

    } else if (Asserv_State == 20) {
        //-----------------------------------
        // spped constrain
        //-----------------------------------
        constrain_speed_order();
        Asserv_State = 21;

    } else if (Asserv_State == 21) {
        //-----------------------------------
        // acceleration constrain
        //-----------------------------------
        constrain_acceleration_order(ASSERV_EVERY*ODO_EVERY_MS*0.001f);
        Asserv_State = 30;

    } else if (Asserv_State == 30) {
        //-----------------------------------
        // consigne
        //-----------------------------------
        Asserv_PWM_calculator(&Consigne);
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
        if (auto_printpos_en && ((Timer_ms1 - Last_Timer_print_pos) > auto_printpos_delay)) {
            float speed_linear = sqrtf(speed_robot.vx*speed_robot.vx + speed_robot.vy*speed_robot.vy);
            float speed_direction = atan2f(speed_robot.vy, speed_robot.vx);
            printf("ROBOTDATA %0.4f %0.4f %0.4f %0.2f %0.2f %0.2f\n", kalman_current_state.x[0], kalman_current_state.x[1], kalman_current_state.x[2], speed_linear, speed_direction, speed_robot.vt);          
            Last_Timer_print_pos += auto_printpos_delay;

            if(Lidar_inconsistency_count > 10){
                printf("Lidar inconsistency count: %d\n", Lidar_inconsistency_count);
                Lidar_inconsistency_count = 0;
            }
        }
        Asserv_State = 0;
        
    } else {
        Asserv_State = 0;
    }
}

uint8_t Activate_Position_Sending_Func (void) {
    uint32_t state;
    if (Get_Param_u32(&state)) return PARAM_ERROR_CODE;
    auto_printpos_en = (state != 0);
    Last_Timer_print_pos = Timer_ms1;
    
    uint32_t Delay;
    if (!Get_Param_u32(&Delay)) {   // s'il y a un 2eme param, on s'en sert comme delai entre les prints (en ms, of course)
        auto_printpos_delay = Delay;
    }
    return 0;
}


uint8_t Set_Odo_Spacing_Cmd(void){
    float valf;
    if (Get_Param_Float(&valf))
        return 1;
    odo_set_spacing(valf);
    return 0;
}

uint8_t Set_Lidar_Cmd(void) {
    float z_x, z_y, z_theta;

    // Récupération des mesures LIDAR
    if (Get_Param_Float(&z_x)) return 1;
    if (Get_Param_Float(&z_y)) return 1;
    if (Get_Param_Float(&z_theta)) return 1;

    if(!enable_kalman) {
        position_lidar.x = z_x;
        position_lidar.y = z_y;
        position_lidar.t = principal_angle(z_theta);
    }else{
        // Mise à jour des données LIDAR (avec angle normalisé)
        position_lidar.x = z_x;
        position_lidar.y = z_y;
        position_lidar.t = principal_angle(z_theta);

        if(!kalman_initialized){
            // Initialisation du filtre de Kalman avec les données LIDAR
            kalman_init_with_lidar(&kalman_fifo, &position_lidar);
            kalman_initialized = 1;
        }else{
            // Récupération de l'index dans la FIFO correspondant au délai LiDAR
            int delay_index = kalman_fifo_get_delay(&kalman_fifo, LIDAR_DELAY, 1);
            if (delay_index < 0) {
                return 0; // erreur
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
    return 0;
}


uint8_t Synchro_Lidar_Cmd(void){
    float z_x, z_y, z_theta;
    if (Get_Param_Float(&z_x)) return 1;
    if (Get_Param_Float(&z_y)) return 1;
    if (Get_Param_Float(&z_theta)) return 1;
    
    position_lidar.x = z_x;
    position_lidar.y = z_y;
    position_lidar.t = principal_angle(z_theta);

    kalman_init_with_lidar(&kalman_fifo, &position_lidar);
    return 0;
}

uint8_t Enable_Kalman_Cmd(void){
    uint32_t state;
    if (Get_Param_u32(&state)) return PARAM_ERROR_CODE;
    enable_kalman = state;
    return 0;
}