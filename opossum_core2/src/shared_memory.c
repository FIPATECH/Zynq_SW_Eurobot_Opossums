#include "main.h"
#include "lib_asserv/Lib_Asserv.h"

volatile sharedCommand *shared_mem = (volatile sharedCommand *)SHARED_MEMORY_BASEADDR;
sharedCommand local_data;

void init_shared_memory() {
    //Disable cache on OCM
    // S=b1 TEX=b100 AP=b11, Domain=b1111, C=b0, B=b0
    Xil_SetTlbAttributes(SHARED_MEMORY_BASEADDR,0x14de2);

    // Ensure memory ordering before writes
    __asm__ volatile("dsb sy");

    // Initialize flags to 0
    shared_mem->flag_cmd_position_valid = 0;
    shared_mem->flag_cmd_position_ack = 0;

    shared_mem->flag_cmd_speed_valid = 0;
    shared_mem->flag_cmd_speed_ack = 0;

    shared_mem->flag_cmd_abs_speed_valid = 0;
    shared_mem->flag_cmd_abs_speed_ack = 0;

    shared_mem->flag_set_lidar_valid = 0;
    shared_mem->flag_set_lidar_ack = 0;

    shared_mem->flag_set_camera_1_valid = 0;
    shared_mem->flag_set_camera_1_ack = 0;

    shared_mem->flag_set_camera_2_valid = 0;
    shared_mem->flag_set_camera_2_ack = 0;

    shared_mem->flag_set_camera_3_valid = 0;
    shared_mem->flag_set_camera_3_ack = 0;

    shared_mem->flag_asserv_mode_valid = 0;
    shared_mem->flag_asserv_mode_ack = 0;

    shared_mem->flag_asserv_done_valid = 0;
    shared_mem->flag_asserv_done_ack = 0;

    shared_mem->flag_set_pos_valid = 0;
    shared_mem->flag_set_pos_ack = 0;

    shared_mem->flag_vmax_valid = 0;
    shared_mem->flag_vmax_ack = 0;

    shared_mem->flag_vtmax_valid = 0;
    shared_mem->flag_vtmax_ack = 0;

    shared_mem->flag_amax_valid = 0;
    shared_mem->flag_amax_ack = 0;

    shared_mem->flag_cmd_esc_valid = 0;
    shared_mem->flag_cmd_esc_ack = 0;

    shared_mem->flag_kalman_out_valid = 0;
    shared_mem->flag_kalman_out_ack = 0;

    shared_mem->flag_enable_kalman_valid = 0;
    shared_mem->flag_enable_kalman_ack = 0;

    shared_mem->flag_odo_spacing_valid = 0;
    shared_mem->flag_odo_spacing_ack = 0;

    shared_mem->flag_kalman_out_valid = 0;
    shared_mem->flag_kalman_out_ack = 0;

    shared_mem->flag_speed_robot_valid = 0;
    shared_mem->flag_speed_robot_ack = 0;

    // Ensure writes are complete
    __asm__ volatile("dsb sy");
}


int send_to_other_core(const void *data, size_t size,
                        volatile void *dest,
                        volatile uint32_t *flag_valid,
                        volatile uint32_t *flag_ack) {
    // Si la donnée précédente n'a pas encore été lue, on refuse d'envoyer
    // if (*flag_valid && !(*flag_ack)) {
    //     return 0; // Pas encore consommé
    // }

    // Copie mémoire
    const uint8_t *src_bytes = (const uint8_t *)data;
    volatile uint8_t *dst_bytes = (volatile uint8_t *)dest;
    for (size_t i = 0; i < size; ++i) {
        dst_bytes[i] = src_bytes[i];
    }

    __asm__ volatile("dsb sy" ::: "memory");

    *flag_valid = 1;
    *flag_ack = 0;

    return 1; // Succès
}


void send_to_other_core_blocking(const void *data, size_t size,
                                 volatile void *dest,
                                 volatile uint32_t *flag_valid,
                                 volatile uint32_t *flag_ack) {
    // Attendre que la donnée ait été consommée
    while (*flag_valid && !(*flag_ack)) {
        // Attente active : peut être optimisée par __WFE() ou sleep
    }

    // Copie mémoire champ par champ (byte-wise)
    const uint8_t *src_bytes = (const uint8_t *)data;
    volatile uint8_t *dst_bytes = (volatile uint8_t *)dest;
    for (size_t i = 0; i < size; ++i) {
        dst_bytes[i] = src_bytes[i];
    }

    // Barrière de synchronisation
    __asm__ volatile("dsb sy" ::: "memory");

    // Signale que la donnée est prête
    *flag_valid = 1;
    *flag_ack = 0;
}

// Fonction générique de réception
int check_from_other_core(void *data_out, size_t size,
                          const volatile void *src,
                          volatile uint32_t *flag_valid,
                          volatile uint32_t *flag_ack) {
    if (*flag_valid && !(*flag_ack)) {
        const volatile uint8_t *src_bytes = (const volatile uint8_t *)src;
        uint8_t *dst_bytes = (uint8_t *)data_out;

        for (size_t i = 0; i < size; ++i) {
            dst_bytes[i] = src_bytes[i];
        }

        __asm__ volatile("dmb sy" ::: "memory");

        *flag_ack = 1;
        *flag_valid = 0;

        return 1; // Nouvelle donnée reçue
    }
    return 0; // Rien à lire
}

int check_for_cmd_state = 0;
int old_check_timer_ms1 = 0;

int previous_timer = 0;

void check_for_cmd_loop(void){
    if(Timer_ms1 - old_check_timer_ms1 > CHECK_FOR_NEW_COMMANDS_EVERY){
        old_check_timer_ms1 = Timer_ms1;
        switch(check_for_cmd_state){
            case 0: 
                if(CHECK_FIELD(&local_data, cmd_position)){
                    motion_pos(local_data.cmd_position); 
                }
                check_for_cmd_state++;
                break;
            case 1: 
                if(CHECK_FIELD(&local_data, cmd_speed)){
                    motion_speed(local_data.cmd_speed);
                }
                check_for_cmd_state++;
                break;
            case 2: 
                if(CHECK_FIELD(&local_data, cmd_abs_speed)){
                    motion_absolute_speed(local_data.cmd_abs_speed);
                }
                check_for_cmd_state++;
                break;
            case 3: 
                if(CHECK_FIELD(&local_data, asserv_mode)){
                    if(local_data.asserv_mode == 0){
                        motion_free();
                    } else if(local_data.asserv_mode == 4){
                        motion_block();
                    }
                }
                check_for_cmd_state++;
                break;
            case 4: 
                if(CHECK_FIELD(&local_data, set_lidar)){
                    Set_Lidar_Cmd(local_data.set_lidar);                        
                }
                // printf("lidar pos: %f %f %f\n", 
                //     (double)(local_data.set_lidar.lidar_position_x), 
                //     (double)(local_data.set_lidar.lidar_position_y), 
                //     (double)(local_data.set_lidar.lidar_position_t));
                check_for_cmd_state++;
                break;
            case 5: 
                if(CHECK_FIELD(&local_data, set_pos)){
                    set_position(local_data.set_pos);
                }
                check_for_cmd_state++;
                break;
            case 6: 
                if(CHECK_FIELD(&local_data, vmax)){
                    set_Constraint_vitesse_xy_max(local_data.vmax);
                }
                check_for_cmd_state++;
                break;
            case 7: 
                if(CHECK_FIELD(&local_data, vtmax)){
                    set_Constraint_vt_max(local_data.vtmax);
                }
                check_for_cmd_state++;
                break;
            case 8: 
                if(CHECK_FIELD(&local_data, amax)){
                    set_Constraint_a_xy_max(local_data.amax);
                }
                check_for_cmd_state++;
                break;
            case 9: 
                if(CHECK_FIELD(&local_data, cmd_esc)){
                    Asserv_PWM_calculator(&local_data.cmd_esc);
                }
                check_for_cmd_state++;
                break;
            case 10: 
                if(CHECK_FIELD(&local_data, enable_kalman)){
                    en_kalman = local_data.enable_kalman;
                }
                check_for_cmd_state++;
                break;
            case 11: 
                if(CHECK_FIELD(&local_data, odo_spacing)){
                    odo_set_spacing(local_data.odo_spacing);
                }
                check_for_cmd_state++; 
                break;
            case 12: 
                if(CHECK_FIELD(&local_data, set_camera_1)){
                    Set_Camera_Cmd(local_data.set_camera_1);                        
                }
                check_for_cmd_state++;
                break;
            case 13: 
                if(CHECK_FIELD(&local_data, set_camera_2)){
                    Set_Camera_Cmd(local_data.set_camera_2);                        
                }
                check_for_cmd_state++;
                break;
            case 14: 
                if(CHECK_FIELD(&local_data, set_camera_3)){
                    Set_Camera_Cmd(local_data.set_camera_3);                        
                }
                check_for_cmd_state = 0; //return to the first command for the next loop
                break;
        }
    }
}
