#include "main.h"

volatile sharedCommand *shared_mem = (volatile sharedCommand *)SHARED_MEMORY_BASEADDR;

void init_shared_memory() {
    //Disable cache on OCM
    // S=b1 TEX=b100 AP=b11, Domain=b1111, C=b0, B=b0
    // Xil_SetTlbAttributes(SHARED_MEMORY_BASEADDR,0x14de2);
}


void send_to_other_core(const volatile void *data, size_t size,
                        volatile void *dest,
                        volatile uint32_t *flag_valid,
                        volatile uint32_t *flag_ack) {
    const volatile uint8_t *src_bytes = (const volatile uint8_t *)data;
    volatile uint8_t *dst_bytes = (volatile uint8_t *)dest;

    for (size_t i = 0; i < size; i++) {
        dst_bytes[i] = src_bytes[i];
    }

    __asm__ volatile("dsb sy" ::: "memory");

    *flag_valid = 1;
    *flag_ack = 0;
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