#include "main.h"

volatile sharedCommand *shared_mem = (volatile sharedCommand *)SHARED_MEMORY_BASEADDR;
sharedCommand local_data;

void init_shared_memory() {
    //Disable cache on OCM
    // S=b1 TEX=b100 AP=b11, Domain=b1111, C=b0, B=b0
    // Xil_SetTlbAttributes(SHARED_MEMORY_BASEADDR,0x14de2);
}


int send_to_other_core(const void *data, size_t size,
                        volatile void *dest,
                        volatile uint32_t *flag_valid,
                        volatile uint32_t *flag_ack) {
    // Si la donnée précédente n'a pas encore été lue, on refuse d'envoyer
    if (*flag_valid && !(*flag_ack)) {
        return 0; // Pas encore consommé
    }

    // Copie mémoire
    void *non_volatile_dst = (void *)(uintptr_t)dest;
    memcpy(non_volatile_dst, data, size);

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

    void *non_volatile_dst = (void *)(uintptr_t)dest;
    memcpy(non_volatile_dst, data, size);

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

        // Copie mémoire
        memcpy(dst_bytes, src_bytes, size);

        __asm__ volatile("dmb sy" ::: "memory");

        *flag_ack = 1;
        *flag_valid = 0;

        return 1; // Nouvelle donnée reçue
    }
    return 0; // Rien à lire
}