#include "main.h"

// Initialize the shared memory area
volatile sharedCommand *shared_mem = (volatile sharedCommand *)SHARED_MEMORY_BASEADDR;

void init_shared_memory() {
    //Disable cache on OCM
    // S=b1 TEX=b100 AP=b11, Domain=b1111, C=b0, B=b0
    Xil_SetTlbAttributes(SHARED_MEMORY_BASEADDR,0x14de2);
    
    // Ensure memory ordering before writes
    __asm__ volatile("dsb sy");

    // Clear the flags and data structures
    shared_mem->flag_position_valid = 0;
    shared_mem->flag_position_ack = 0;
    shared_mem->flag_speed_valid = 0;
    shared_mem->flag_speed_ack = 0;
    shared_mem->lidar_data_valid = 0;
    shared_mem->lidar_data_ack = 0;

    
    shared_mem->flag_kalman_out_valid = 0;
    shared_mem->flag_kalman_out_ack = 0;
    shared_mem->flag_pos_done_valid = 0;
    shared_mem->flag_pos_done_ack = 0;

    shared_mem->flag_timer_valid = 0;
    shared_mem->flag_timer_ack = 0;
    
    // Ensure writes are complete
    __asm__ volatile("dsb sy");
}


void send_timer_to_core0(int timer_ms) {
    // Attendre que l'ancien timer ait été pris en compte
    if (shared_mem->flag_timer_valid && !shared_mem->flag_timer_ack) {
        return; // CORE0 n'a pas encore lu l'ancien timer
    }

    shared_mem->Timer = timer_ms;

    __asm__ volatile("dsb sy"); // S'assurer que Timer est bien écrit avant le flag

    shared_mem->flag_timer_valid = 1;
    shared_mem->flag_timer_ack = 0;
}