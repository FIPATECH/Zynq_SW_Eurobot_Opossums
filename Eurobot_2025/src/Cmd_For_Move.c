#include "main.h"


//MOVE
uint8_t Move_Cmd(void) {
    if (AU_state) {
        xil_printf("INVALID COMMAND : AU\n");
        return 0;
    }else{
        Position cmd_position;
        if (Get_Param_Float(&cmd_position.x))     return PARAM_ERROR_CODE;
        if (Get_Param_Float(&cmd_position.y))     return PARAM_ERROR_CODE;
        if (Get_Param_Float(&cmd_position.t))     return PARAM_ERROR_CODE;

        // ecriture dans la mémoire partagée
        SEND_FIELD(shared_mem, cmd_position);
        return 0;
    }
}

// SPEED
uint8_t Speed_Cmd(void) {
    if (AU_state) {
        printf("INVALID COMMAND : AU\n");
        return 0;
    }else{
        Speed cmd_speed;
        if (Get_Param_Float(&cmd_speed.vx)) return PARAM_ERROR_CODE;
        if (Get_Param_Float(&cmd_speed.vy)) return PARAM_ERROR_CODE;
        if (Get_Param_Float(&cmd_speed.vt)) return PARAM_ERROR_CODE;

        // ecriture dans la mémoire partagée
        SEND_FIELD(shared_mem, cmd_speed);
        return 0;
    }
}

// ASPEED
uint8_t Absolute_Speed_Cmd(void) {
    if (AU_state) {
        printf("INVALID COMMAND : AU\n");
        return 0;
    }else{
        Speed cmd_abs_speed;
        if (Get_Param_Float(&cmd_abs_speed.vx)) return PARAM_ERROR_CODE;
        if (Get_Param_Float(&cmd_abs_speed.vy)) return PARAM_ERROR_CODE;
        if (Get_Param_Float(&cmd_abs_speed.vt)) return PARAM_ERROR_CODE;
        
        // ecriture dans la mémoire partagée
        SEND_FIELD(shared_mem, cmd_abs_speed);
        return 0;
    }
}

// FREE
uint8_t FREE_Cmd(void) {
    //ecriture dans la mémoire partagée
    shared_mem->asserv_mode = 0; // Mode libre
    __asm__ volatile("dsb sy");
    shared_mem->flag_assser_mode_valid = 1; // Indique que le mode asservissement est valide
    __asm__ volatile("dsb sy");
    return 0;
}

// block
uint8_t BLOCK_Cmd(void) {
    if (AU_state) {
        printf("INVALID COMMAND : AU\n");
        return 0;
    }else{
        // ecriture dans la mémoire partagée
        shared_mem->asserv_mode = 4; // Mode blocage
        __asm__ volatile("dsb sy");
        shared_mem->flag_assser_mode_valid = 1; // Indique que le mode asservissement est valide
        __asm__ volatile("dsb sy");
        return 0;
    }
}

uint8_t Asserv_Done_Cmd(void) {
    // lecture de la mémoire partagée
    __asm__ volatile("dsb sy");
    int asserv_done = shared_mem->asserv_done;
    __asm__ volatile("dsb sy");
    printf("%d\n", asserv_done);
    return 0;
}

uint8_t Get_Pos_Cmd(void) {
    Position pos;
    // lecture de la mémoire partagée
    if (shared_mem->flag_kalman_out_valid) {
        __asm__ volatile("dsb sy");
        pos.x = shared_mem->kalman_out.x;
        pos.y = shared_mem->kalman_out.y;
        pos.t = shared_mem->kalman_out.t;
        __asm__ volatile("dsb sy");

        shared_mem->flag_kalman_out_ack = 1; // Indique que la position a été lue
        shared_mem->flag_kalman_out_valid = 0; // Réinitialise le flag

        printf("GETPOS ");
        printf("%.4f ", (double) (pos.x));
        printf("%.4f ", (double) (pos.y));
        printf("%.4f\n", (double) (pos.t));
    } else {
        printf("GETPOS ERRROR: Position not valid\n");
    }
    return 0;
}

uint8_t Get_Odo_Cmd(void) {
    Position kalman_out;
    Speed speed_robot;
    // lecture de la mémoire partagée
    int status1;
    int status2;
    status1 = CHECK_FIELD(shared_mem, kalman_out);
    status2 = CHECK_FIELD(shared_mem, speed_robot);

    if(!status1 || !status2) {
        printf("GETODO ERROR: Position or speed not valid\n");
        return 0;
    }else{
        printf("ODO ");
        printf("%.4f ", (double)(kalman_out.x));
        printf("%.4f ", (double)(kalman_out.y));
        printf("%.4f ", (double)(kalman_out.t));
        printf("%.4f ", (double)(speed_robot.vx));
        printf("%.4f ", (double)(speed_robot.vy));
        printf("%.4f\n", (double)(speed_robot.vt));
        return 0;
    }
}

uint8_t SET_Cmd(void) {
    Position set_pos;
    // Récupération de la position à définir
    if (Get_Param_Float(&set_pos.x)) return 1;
    if (Get_Param_Float(&set_pos.y)) return 1;
    if (Get_Param_Float(&set_pos.t)) return 1;
    // ecriture de la mémoire partagée
    SEND_FIELD(shared_mem, set_pos);
    return 0;
}

uint8_t Set_Lidar_Cmd(void) {
    Position lidar_position;
    uint32_t time;

    // Récupération des mesures LIDAR
    if (Get_Param_Float(&lidar_position.x))     return PARAM_ERROR_CODE;
    if (Get_Param_Float(&lidar_position.y))     return PARAM_ERROR_CODE;
    if (Get_Param_Float(&lidar_position.t))     return PARAM_ERROR_CODE; 
    if (Get_Param_u32(&time))                   return PARAM_ERROR_CODE;

    // ecriture dans la mémoire partagée
    shared_mem->lidar_position.lidar_position = lidar_position;
    shared_mem->lidar_position.delay = (int)time;

    __asm__ volatile("dsb sy");
    shared_mem->flag_lidar_data_valid = 1; // Indique que les données LIDAR sont valides
    __asm__ volatile("dsb sy");
    return 0;
}

// VMAX
uint8_t VMAX_Cmd(void) {
    float vmax;
    if (Get_Param_Float(&vmax))     return PARAM_ERROR_CODE;
    // ecriture dans la mémoire partagée
    SEND_FIELD(shared_mem, vmax);
    return 0;
}

// VTMAX
uint8_t VTMAX_Cmd(void) {
    float vtmax;
    if (Get_Param_Float(&vtmax))    return PARAM_ERROR_CODE;
    // ecriture dans la mémoire partagée
    SEND_FIELD(shared_mem, vtmax);
    return 0;
}

// AMAX
uint8_t AMAX_Cmd(void) {
    float amax;
    if (Get_Param_Float(&amax))    return PARAM_ERROR_CODE;  //almax
    // ecriture dans la mémoire partagée
    SEND_FIELD(shared_mem, amax);
    return 0;
}

uint8_t PWM_Func(void)
{
    ESC_Command cmd_esc;
    if (Get_Param_Float(&cmd_esc.command1))    return PARAM_ERROR_CODE;
    if (Get_Param_Float(&cmd_esc.command2))    return PARAM_ERROR_CODE;
    if (Get_Param_Float(&cmd_esc.command3))    return PARAM_ERROR_CODE;
    
    // ecriture dans la mémoire partagée
    SEND_FIELD(shared_mem, cmd_esc);
    return 0;
}

uint8_t Enable_Kalman_Cmd(void) {
    uint32_t enable_kalman;
    if (Get_Param_u32(&enable_kalman)) return PARAM_ERROR_CODE;
    
    // ecriture dans la mémoire partagée
    SEND_FIELD(shared_mem, enable_kalman);
    return 0;
}

uint8_t Set_Odo_Spacing_Cmd(void) {
    float odo_spacing;
    if (Get_Param_Float(&odo_spacing)) return PARAM_ERROR_CODE;

    // ecriture dans la mémoire partagée
    SEND_FIELD(shared_mem, odo_spacing);
    return 0;
}


int auto_printpos_en = 0; // si on active l'envoi de la position
uint32_t auto_printpos_delay = 100; // en ms
uint32_t Last_Timer_print_pos = 0; // dernier envoi de la position

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
