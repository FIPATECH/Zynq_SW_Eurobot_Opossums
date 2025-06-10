#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#define SHARED_MEMORY_BASEADDR 0xFFFF0000 // Base address for shared memory

/**
 * @brief Initialize the shared memory area
 * 
 */
void init_shared_memory(void); 

#endif // SHARED_MEMORY_H