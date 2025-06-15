#include "main.h"

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

    __asm__ volatile("dsb sy" ::: "memory");

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
    __asm__ volatile("dsb sy" ::: "memory");

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

        __asm__ volatile("dsb sy" ::: "memory");

        *flag_ack = 1;
        *flag_valid = 0;
        return 1; // Data received
    }
    return 0; // Nothing to read
}