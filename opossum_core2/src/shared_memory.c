#include "main.h"

void init_shared_memory() {
    //Disable cache on OCM
    // S=b1 TEX=b100 AP=b11, Domain=b1111, C=b0, B=b0
    Xil_SetTlbAttributes(0xFFFF0000,0x14de2);

    // Initialize the shared memory area
    volatile sharedCommand *shared_mem = (volatile sharedCommand *)SHARED_MEMORY_BASEADDR;
    
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

    
    // Ensure writes are complete
    __asm__ volatile("dsb sy");
}