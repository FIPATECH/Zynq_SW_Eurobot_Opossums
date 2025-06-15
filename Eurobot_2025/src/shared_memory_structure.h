#ifndef SHARED_MEMORY_STRUCTURE_H
#define SHARED_MEMORY_STRUCTURE_H

#include <stdint.h>
#include "main.h"
#include "Asserv_type.h"

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
    //
    // ******************************************************************************
    volatile uint32_t flag_cmd_position_valid; // CORE0 -> CORE1: 1 if position is valid, 0 otherwise
    volatile uint32_t flag_cmd_position_ack;   // CORE1 -> CORE0: 1 new position taken into account, 0 otherwise
    Position cmd_position;

    volatile uint32_t flag_cmd_speed_valid; // CORE0 -> CORE1: 1 if speed is valid, 0 otherwise
    volatile uint32_t flag_cmd_speed_ack;   // CORE1 -> CORE0: 1 new speed taken into account, 0 otherwise
    Speed cmd_speed;

    volatile uint32_t flag_cmd_abs_speed_valid; // CORE0 -> CORE1: 1 if absolute speed is valid, 0 otherwise
    volatile uint32_t flag_cmd_abs_speed_ack;   // CORE1 -> CORE0: 1 new absolute speed taken into account, 0 otherwise
    Speed cmd_abs_speed;

    volatile uint32_t flag_lidar_data_valid; // CORE0 -> CORE1: 1 if lidar data is valid, 0 otherwise
    volatile uint32_t flag_lidar_data_ack;   // CORE1 -> CORE0: 1 new lidar data taken into account, 0 otherwise
    struct {
        Position lidar_position; // position of the robot according to the lidar
        int delay; // calculation delay in ms 
    } lidar_position;

    volatile uint32_t flag_assser_mode_valid; // CORE0 -> CORE1: 1 if asserv mode is valid, 0 otherwise
    volatile uint32_t flag_assser_mode_ack;   // CORE1 -> CORE0: 1 new asserv mode taken into account, 0 otherwise
    int asserv_mode; // asserv mode (0: free, 1: position, 2: speed, 3: absolute speed, 4: break)

    volatile uint32_t flag_asserv_done_valid; // CORE0 -> CORE1: 1 if asserv done is valid, 0 otherwise
    volatile uint32_t flag_asserv_done_ack;   // CORE1 -> CORE0: 1 new asserv done taken into account, 0 otherwise
    int asserv_done; // 1 if asserv is done, 0 otherwise

    volatile uint32_t flag_set_pos_valid; // CORE0 -> CORE1: 1 if asserv done is valid, 0 otherwise
    volatile uint32_t flag_set_pos_ack;   // CORE1 -> CORE0: 1 new asserv done taken into account, 0 otherwise
    Position set_pos; // position to set in the world frame

    volatile uint32_t flag_vmax_valid; // CORE0 -> CORE1: 1 if speed to set is valid, 0 otherwise
    volatile uint32_t flag_vmax_ack;   // CORE1 -> CORE0: 1 new speed to set taken into account, 0 otherwise
    float vmax; // maximum speed in the world frame

    volatile uint32_t flag_vtmax_valid; // CORE0 -> CORE1: 1 if angular speed to set is valid, 0 otherwise
    volatile uint32_t flag_vtmax_ack;   // CORE1 -> CORE0: 1 new angular speed to set taken into account, 0 otherwise
    float vtmax; // maximum angular speed in the world frame

    volatile uint32_t flag_amax_valid; // CORE0 -> CORE1: 1 if acceleration to set is valid, 0 otherwise
    volatile uint32_t flag_amax_ack;   // CORE1 -> CORE0: 1 new acceleration to set taken into account, 0 otherwise
    float amax; // maximum acceleration in the world frame

    volatile uint32_t flag_cmd_esc_valid; 
    volatile uint32_t flag_cmd_esc_ack; 
    ESC_Command cmd_esc; // command to send to the ESCs  

    volatile uint32_t flag_enable_kalman_valid; // CORE0 -> CORE1: 1 if kalman is enabled, 0 otherwise
    volatile uint32_t flag_enable_kalman_ack;   // CORE1 -> CORE0: 1 new kalman enable taken into account, 0 otherwise
    int enable_kalman; // 1 if kalman is enabled, 0 otherwise

    volatile uint32_t flag_odo_spacing_valid; // CORE0 -> CORE1: 1 if odo spacing is valid, 0 otherwise
    volatile uint32_t flag_odo_spacing_ack;   // CORE1 -> CORE0: 1 new odo spacing taken into account, 0 otherwise
    float odo_spacing; // spacing between the wheels in meters
    
    // ******************************* CORE1 -> CORE0 *******************************
    //
    // ******************************************************************************
    volatile uint32_t flag_kalman_out_valid; // CORE1 -> CORE0: 1 if kalman output is valid, 0 otherwise
    volatile uint32_t flag_kalman_out_ack;   // CORE0 -> CORE1: 1 new kalman output taken into account, 0 otherwise
    Position kalman_out;

    volatile uint32_t flag_speed_robot_valid; // CORE1 -> CORE0: 1 if speed robot is valid, 0 otherwise
    volatile uint32_t flag_speed_robot_ack;   // CORE0 -> CORE1: 1 new speed robot taken into account, 0 otherwise  
    Speed speed_robot; // speed of the robot in the world frame

    // ********************************* Timer variables *********************************
    volatile uint32_t flag_Timer_ms1_valid; // CORE1 -> CORE0: 1 if timer is valid, 0 otherwise
    volatile uint32_t flag_Timer_ms1_ack;   // CORE0 -> CORE1: 1 new timer taken into account, 0 otherwise
    int Timer_ms1; // Timer value in ms
} sharedCommand;

#endif // SHARED_MEMORY_STRUCTURE_H