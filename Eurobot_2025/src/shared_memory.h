#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include "shared_memory_structure.h"

#define SHARED_MEMORY_BASEADDR 0xFFFF0000 // Base address for shared memory

extern volatile sharedCommand *shared_mem;
/**
 * @brief Initialize the shared memory area
 * 
 */
void init_shared_memory(void); 

void check_timer_from_core1(void);

#endif // SHARED_MEMORY_H
