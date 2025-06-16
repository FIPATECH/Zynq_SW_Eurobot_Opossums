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

void send_to_other_core_blocking(const volatile void *data, size_t size,
                                 volatile void *dest,
                                 volatile uint32_t *flag_valid,
                                 volatile uint32_t *flag_ack) {
    if (*flag_valid && !(*flag_ack)) return;

    const volatile uint8_t *src_bytes = (const volatile uint8_t *)data;
    volatile uint8_t *dst_bytes = (volatile uint8_t *)dest;

    for (size_t i = 0; i < size; i++) {
        dst_bytes[i] = src_bytes[i];
    }

    __asm__ volatile("dsb sy" ::: "memory");

    *flag_valid = 1;
    *flag_ack = 0;
}

int check_from_other_core(volatile void *data_out, size_t size,
                          const volatile void *src,
                          volatile uint32_t *flag_valid,
                          volatile uint32_t *flag_ack) {
    if (*flag_valid) {
        uint8_t *dst_bytes = (uint8_t *)data_out;
        const volatile uint8_t *src_bytes = (const volatile uint8_t *)src;
        for (size_t i = 0; i < size; i++) {
            dst_bytes[i] = src_bytes[i];
        }

        __asm__ volatile("dsb sy" ::: "memory");

        *flag_ack = 1;
        *flag_valid = 0;
        return 1;
    }
    return 0;
}