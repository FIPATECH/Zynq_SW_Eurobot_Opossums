#include "main.h"


//MOVE
uint8_t Move_Cmd(void) {
    if (AU_state) {
        xil_printf("INVALID COMMAND : AU\n");
        return 0;
    }else{
        if (Get_Param_Float(&local_data.cmd_position.x))     return PARAM_ERROR_CODE;
        if (Get_Param_Float(&local_data.cmd_position.y))     return PARAM_ERROR_CODE;
        if (Get_Param_Float(&local_data.cmd_position.t))     return PARAM_ERROR_CODE;
        // ecriture dans la mémoire partagée
        SEND_FIELD(&local_data, cmd_position);
        return 0;
    }
}

// SPEED
uint8_t Speed_Cmd(void) {
    if (AU_state) {
        printf("INVALID COMMAND : AU\n");
        return 0;
    }else{
        if (Get_Param_Float(&local_data.cmd_speed.vx)) return PARAM_ERROR_CODE;
        if (Get_Param_Float(&local_data.cmd_speed.vy)) return PARAM_ERROR_CODE;
        if (Get_Param_Float(&local_data.cmd_speed.vt)) return PARAM_ERROR_CODE;

        // ecriture dans la mémoire partagée
        SEND_FIELD(&local_data, cmd_speed);
        return 0;
    }
}

// ASPEED
uint8_t Absolute_Speed_Cmd(void) {
    if (AU_state) {
        printf("INVALID COMMAND : AU\n");
        return 0;
    }else{
        if (Get_Param_Float(&local_data.cmd_abs_speed.vx)) return PARAM_ERROR_CODE;
        if (Get_Param_Float(&local_data.cmd_abs_speed.vy)) return PARAM_ERROR_CODE;
        if (Get_Param_Float(&local_data.cmd_abs_speed.vt)) return PARAM_ERROR_CODE;
        
        // ecriture dans la mémoire partagée
        SEND_FIELD(&local_data, cmd_abs_speed);
        return 0;
    }
}

// FREE
uint8_t FREE_Cmd(void) {
    //ecriture dans la mémoire partagée
    local_data.asserv_mode = 0; // Mode libre
    SEND_FIELD(&local_data, asserv_mode);
    return 0;
}

// block
uint8_t BLOCK_Cmd(void) {
    if (AU_state) {
        printf("INVALID COMMAND : AU\n");
        return 0;
    }else{
        local_data.asserv_mode = 4; // Mode blocage
        SEND_FIELD(&local_data, asserv_mode);
        return 0;
    }
}

uint8_t Asserv_Done_Cmd(void) {
    // lecture de la mémoire partagée
    CHECK_FIELD(&local_data, asserv_done);
    printf("%d\n", local_data.asserv_done);
    return 0;
}

uint8_t Get_Pos_Cmd(void) {
    // lecture de la mémoire partagée
    if (CHECK_FIELD(&local_data, kalman_out)) {
        printf("GETPOS ");
        printf("%.4f ", (double)  (local_data.kalman_out.x));
        printf("%.4f ", (double)  (local_data.kalman_out.y));
        printf("%.4f\n", (double) (local_data.kalman_out.t));
    } else {
        printf("GETPOS ERRROR: Position not valid\n");
    }
    return 0;
}

uint8_t Get_Odo_Cmd(void) {
    // lecture de la mémoire partagée
    int status1;
    int status2;
    status1 = CHECK_FIELD(&local_data, kalman_out);
    status2 = CHECK_FIELD(&local_data, speed_robot);

    if(!status1 || !status2) {
        printf("GETODO ERROR: Position or speed not valid\n");
        return 0;
    }else{
        printf("ODO ");
        printf("%.4f ", (double)(local_data.kalman_out.x));
        printf("%.4f ", (double)(local_data.kalman_out.y));
        printf("%.4f ", (double)(local_data.kalman_out.t));
        printf("%.4f ", (double)(local_data.speed_robot.vx));
        printf("%.4f ", (double)(local_data.speed_robot.vy));
        printf("%.4f\n", (double)(local_data.speed_robot.vt));
        return 0;
    }
}

uint8_t SET_Cmd(void) {
    // Récupération de la position à définir
    if (Get_Param_Float(&local_data.set_pos.x)) return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_pos.y)) return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_pos.t)) return PARAM_ERROR_CODE;
    // ecriture de la mémoire partagée
    SEND_FIELD(&local_data, set_pos);
    return 0;
}

uint8_t SET0_Cmd(void) {
    local_data.set_pos.x = 0.0f; // position x
    local_data.set_pos.y = 0.0f; // position y
    local_data.set_pos.t = 0.0f; // position t
    // ecriture de la mémoire partagée
    SEND_FIELD(&local_data, set_pos);
    return 0;
}

uint8_t Set_Lidar_Cmd(void) {
    // Récupération des mesures LIDAR
    if (Get_Param_Float(&local_data.set_lidar.lidar_position_x))     return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_lidar.lidar_position_y))     return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_lidar.lidar_position_t))     return PARAM_ERROR_CODE; 
    if (Get_Param_u32(&local_data.set_lidar.delay))                  return PARAM_ERROR_CODE;
    // printf("lidar pos: %f %f %f\n", 
    //         (double)(local_data.set_lidar.lidar_position_x), 
    //         (double)(local_data.set_lidar.lidar_position_y), 
    //         (double)(local_data.set_lidar.lidar_position_t));
    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, set_lidar);
    return 0;
}

uint8_t Set_Lidar_Noise_Cmd(void) {
    if(Get_Param_Float(&local_data.kalman_noise_lidar.process_noise_lidar_x)) return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.kalman_noise_lidar.process_noise_lidar_y)) return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.kalman_noise_lidar.process_noise_lidar_t)) return PARAM_ERROR_CODE;

    SEND_FIELD(&local_data, kalman_noise_lidar);
    return 0;
}

uint8_t Set_Camera_1_Cmd(void) {
    // Récupération des mesures caméra
    if (Get_Param_Float(&local_data.set_camera_1.camera_position_x))     return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_camera_1.camera_position_y))     return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_camera_1.camera_position_t))     return PARAM_ERROR_CODE; 
    if (Get_Param_u32(&local_data.set_camera_1.delay))                  return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.set_camera_1.noise_x))                 return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.set_camera_1.noise_y))                 return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.set_camera_1.noise_t))                 return PARAM_ERROR_CODE;
    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, set_camera_1);
    return 0;
}

uint8_t Set_Camera_2_Cmd(void) {
    // Récupération des mesures caméra
    if (Get_Param_Float(&local_data.set_camera_2.camera_position_x))     return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_camera_2.camera_position_y))     return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_camera_2.camera_position_t))     return PARAM_ERROR_CODE; 
    if (Get_Param_u32(&local_data.set_camera_2.delay))                  return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.set_camera_2.noise_x))                 return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.set_camera_2.noise_y))                 return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.set_camera_2.noise_t))                 return PARAM_ERROR_CODE;

    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, set_camera_2);
    return 0;
}

uint8_t Set_Camera_3_Cmd(void) {
    // Récupération des mesures caméra
    if (Get_Param_Float(&local_data.set_camera_3.camera_position_x))     return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_camera_3.camera_position_y))     return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.set_camera_3.camera_position_t))     return PARAM_ERROR_CODE; 
    if (Get_Param_u32(&local_data.set_camera_3.delay))                  return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.set_camera_3.noise_x))                 return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.set_camera_3.noise_y))                 return PARAM_ERROR_CODE;
    if(Get_Param_Float(&local_data.set_camera_3.noise_t))                 return PARAM_ERROR_CODE;
    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, set_camera_3);
    return 0;
}

// VMAX
uint8_t VMAX_Cmd(void) {
    if (Get_Param_Float(&local_data.vmax))     return PARAM_ERROR_CODE;
    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, vmax);
    return 0;
}

// VTMAX
uint8_t VTMAX_Cmd(void) {
    if (Get_Param_Float(&local_data.vtmax))    return PARAM_ERROR_CODE;
    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, vtmax);
    return 0;
}

// AMAX
uint8_t AMAX_Cmd(void) {
    if (Get_Param_Float(&local_data.amax))    return PARAM_ERROR_CODE;  //almax
    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, amax);
    return 0;
}

uint8_t PWM_Func(void)
{
    if (Get_Param_Float(&local_data.cmd_esc.command1))    return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.cmd_esc.command2))    return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.cmd_esc.command3))    return PARAM_ERROR_CODE;
    if (Get_Param_Float(&local_data.cmd_esc.command4))    return PARAM_ERROR_CODE;
    
    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, cmd_esc);
    return 0;
}

uint8_t Enable_Kalman_Cmd(void) {
    if (Get_Param_u32(&local_data.enable_kalman.enable_lidar_kalman))         return PARAM_ERROR_CODE;
    if (Get_Param_u32(&local_data.enable_kalman.enable_camera_kalman))         return PARAM_ERROR_CODE;
    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, enable_kalman);
    return 0;
}

uint8_t Set_Odo_Spacing_Cmd(void) {
    if (Get_Param_Float(&local_data.odo_spacing)) return PARAM_ERROR_CODE;

    // ecriture dans la mémoire partagée
    SEND_FIELD(&local_data, odo_spacing);
    return 0;
}


int auto_printpos_en = 1; // si on active l'envoi de la position
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

void Print_Position_loop(void) {
    if (auto_printpos_en) {
        if ((Timer_ms1 - Last_Timer_print_pos) >= auto_printpos_delay) {
            Last_Timer_print_pos = Timer_ms1;
            if (CHECK_FIELD(&local_data, kalman_out) && CHECK_FIELD(&local_data, speed_robot)) {
                float speed_linear = sqrtf(local_data.speed_robot.vx * local_data.speed_robot.vx + local_data.speed_robot.vy * local_data.speed_robot.vy);
                float speed_direction = atan2f(local_data.speed_robot.vy, local_data.speed_robot.vx);
                printf("ROBOTDATA %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f\n", local_data.kalman_out.x, local_data.kalman_out.y, local_data.kalman_out.t, speed_linear, speed_direction, local_data.speed_robot.vt);          
            } else {
                printf("POS ERROR: Position or speed not valid\n");
            }
        }
    }
}