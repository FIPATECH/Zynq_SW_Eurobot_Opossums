#include "main.h"

volatile sharedCommand *shared_mem = (volatile sharedCommand *)SHARED_MEMORY_BASEADDR;

void init_shared_memory() {
    //Disable cache on OCM
    // S=b1 TEX=b100 AP=b11, Domain=b1111, C=b0, B=b0
    // Xil_SetTlbAttributes(SHARED_MEMORY_BASEADDR,0x14de2);
}

void check_timer_from_core1(){
    if (shared_mem->flag_timer_valid) {
        int received_timer = shared_mem->Timer;

        // Utiliser la valeur reçue
        printf("Reçu Timer = %d ms\n", received_timer);

        __asm__ volatile("dsb sy"); // S'assurer que la lecture est bien finie avant ack

        // Envoyer l'ack
        shared_mem->flag_timer_ack = 1;
        shared_mem->flag_timer_valid = 0;
    }

}