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

    shared_mem->flag_Timer_ms1_valid = 0;
    shared_mem->flag_Timer_ms1_ack = 0;
    
    // Ensure writes are complete
    __asm__ volatile("dsb sy");
}

void send_to_other_core(const void *data, size_t size,
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

int check_from_other_core(void *data_out, size_t size,
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