#ifndef SHARED_MEMORY_STRUCTURE_H
#define SHARED_MEMORY_STRUCTURE_H

#include <stdint.h>

/**
 * @brief This strcucture describes the organization of the shared memory
 * 
 * @note 
 * Here is the list of variables that are shared between the two cores:
 * 
 * - `cmd_position`: contains the goal position in the world frame
 * 
 * - `cmd_speed`: contains the goal speed in the robot frame
 */
typedef struct {
    // ******************************* CORE0 -> CORE1 *******************************
    volatile uint32_t flag_position_valid; // CORE0 -> CORE1: 1 if position is valid, 0 otherwise
    volatile uint32_t flag_position_ack;   // CORE1 -> CORE0: 1 new position taken into account, 0 otherwise
    struct{
        float x; // x position in m
        float y; // y position in m
        float theta; // orientation in rad
    } cmd_position;

    volatile uint32_t flag_speed_valid; // CORE0 -> CORE1: 1 if speed is valid, 0 otherwise
    volatile uint32_t flag_speed_ack;   // CORE1 -> CORE0: 1 new speed taken into account, 0 otherwise
    struct {
        float vx; // linear speed in m/s
        float vy; // linear speed in m/s
        float omega; // angular speed in rad/s
    } cmd_speed;

    volatile uint32_t lidar_data_valid; // CORE0 -> CORE1: 1 if lidar data is valid, 0 otherwise
    volatile uint32_t lidar_data_ack;   // CORE1 -> CORE0: 1 new lidar data taken into account, 0 otherwise
    struct {
        float x; // x position in m
        float y; // y position in m
        float theta; // orientation in rad
    } lidar_position;


    // ******************************* CORE1 -> CORE0 *******************************
    volatile uint32_t flag_kalman_out_valid; // CORE1 -> CORE0: 1 if kalman output is valid, 0 otherwise
    volatile uint32_t flag_kalman_out_ack;   // CORE0 -> CORE1: 1 new kalman output taken into account, 0 otherwise
    struct {
        float x; // x position in m
        float y; // y position in m
        float theta; // orientation in rad
    } kalman_out;

    volatile uint32_t flag_pos_done_valid; // CORE1 -> CORE0: 1 if position done is valid, 0 otherwise
    volatile uint32_t flag_pos_done_ack;   // CORE0 -> CORE1: 1 new position done taken into account, 0 otherwise
    struct {
        int pos_done; // 1 if position is done, 0 otherwise
    } pos_done;

} sharedCommand;

#endif // SHARED_MEMORY_STRUCTURE_H