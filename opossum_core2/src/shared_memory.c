#include "main.h"
#include "lib_asserv/Lib_Asserv.h"

volatile sharedCommand *shared_mem = (volatile sharedCommand *)SHARED_MEMORY_BASEADDR;

void init_shared_memory() {
    //Disable cache on OCM
    // S=b1 TEX=b100 AP=b11, Domain=b1111, C=b0, B=b0
    // Xil_SetTlbAttributes(SHARED_MEMORY_BASEADDR,0x14de2);
}


void send_to_other_core_blocking(const void *data, size_t size,
                        volatile void *dest,
                        volatile uint32_t *flag_valid,
                        volatile uint32_t *flag_ack) {
    // Ne pas écraser si pas encore lu
    if (*flag_valid && !(*flag_ack)) return;

    // Copier les données dans la mémoire partagée
    memcpy((void *)dest, data, size);

    __asm__ volatile("dsb sy");

    *flag_valid = 1;
    *flag_ack = 0;
}

void send_to_other_core(const void *data, size_t size,
                        volatile void *dest,
                        volatile uint32_t *flag_valid,
                        volatile uint32_t *flag_ack) {
    // Copier les données dans la mémoire partagée
    memcpy((void *)dest, data, size);

    // Barrière mémoire : assure que le memcpy est terminé avant de lever le flag
    __asm__ volatile("dsb sy");

    // Signaler que la donnée est disponible
    *flag_valid = 1;
    *flag_ack = 0;
}

int check_from_other_core(volatile void *data_out, size_t size,
                          volatile void *src,
                          volatile uint32_t *flag_valid,
                          volatile uint32_t *flag_ack) {
    if (*flag_valid) {
        memcpy(data_out, (const void *)src, size);

        __asm__ volatile("dsb sy");

        *flag_ack = 1;
        *flag_valid = 0;
        return 1; // Data received
    }
    return 0; // Nothing to read
}



int check_for_cmd_state = 0;
int old_check_timer_ms1 = 0;

void check_for_cmd_loop(void){
    if(Timer_ms1 - old_check_timer_ms1 > CHECK_FOR_NEW_COMMANDS_EVERY){
        switch(check_for_cmd_state){
            case 0: {
                Position cmd_position;
                if(CHECK_FIELD(shared_mem, cmd_position)){
                    motion_pos(cmd_position);                    
                }
                check_for_cmd_state++;
                break;
            }
            case 1: {
                Speed cmd_speed;
                if(CHECK_FIELD(shared_mem, cmd_speed)){
                    motion_speed(cmd_speed);
                }
                check_for_cmd_state++;
                break;
            }
            case 2: {
                Speed cmd_abs_speed;
                if(CHECK_FIELD(shared_mem, cmd_abs_speed)){
                    motion_absolute_speed(cmd_abs_speed);
                }
                check_for_cmd_state++;
                break;
            }
            case 3: {
                int asserv_mode;
                if(CHECK_FIELD(shared_mem, asserv_mode)){
                    if(asserv_mode == 0){
                        motion_free();
                    } else if(asserv_mode == 4){
                        motion_block();
                    }
                }
                check_for_cmd_state++;
                break;
            }
            case 4: {
                // Position lidar_position;
                // int lidar_delay;
                // if(CHECK_FIELD(&lidar_position, lidar_position) && 
                //    CHECK_FIELD(&lidar_delay, lidar_delay)){
                //     Set_Lidar_Cmd(lidar_position, lidar_delay);
                // }
                check_for_cmd_state++;
                break;
            }
            case 5: {
                Position set_pos;
                if(CHECK_FIELD(shared_mem, set_pos)){
                    set_position(set_pos);
                }
                check_for_cmd_state++;
                break;
            }
            case 6: {
                float vmax;
                if(CHECK_FIELD(shared_mem, vmax)){
                    set_Constraint_vitesse_xy_max(vmax);
                }
                check_for_cmd_state++;
                break;
            }
            case 7: {
                float vtmax;
                if(CHECK_FIELD(shared_mem, vtmax)){
                    set_Constraint_vt_max(vtmax);
                }
                check_for_cmd_state++;
                break;
            }
            case 8: {
                float amax;
                if(CHECK_FIELD(shared_mem, amax)){
                    set_Constraint_a_xy_max(amax);
                }
                check_for_cmd_state++;
                break;
            }
            case 9: {
                ESC_Command cmd_esc;
                if(CHECK_FIELD(shared_mem, cmd_esc)){
                    Asserv_PWM_calculator(&cmd_esc);
                }
                check_for_cmd_state++;
                break;
            }
            case 10: {
                CHECK_FIELD(shared_mem, enable_kalman);
                check_for_cmd_state++;
                break;
            }
            case 11: {
                int odo_spacing;
                if(CHECK_FIELD(shared_mem, odo_spacing)){
                    odo_set_spacing(odo_spacing);
                }
                check_for_cmd_state = 0; // Reset to start over
                break;
            }
        }
    }
}