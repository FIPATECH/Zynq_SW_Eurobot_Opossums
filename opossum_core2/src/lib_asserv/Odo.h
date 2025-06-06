#ifndef __ODO_H_
#define __ODO_H_


extern float Speed_1, Speed_2, Speed_3, Speed_4;

extern float robot_wheel_distance;

/******************************    Fonctions    *******************************/

/**
 * @brief Initialise odometry.
 * 
 */
void odo_init(void);

/**
 * @brief Set the spacing between the wheels of the robot and the center of the robot.
 * 
 * @param param_spacing Spacing between the wheels in meters. 
 */
void odo_set_spacing(float param_spacing);

/**
 * @brief Calculate the speed of the robot based on the RPM of the rotors in the robot base
 * 
 * @param Rotor_RPM1 Speed of the first rotor in RPM
 * @param Rotor_RPM2 Speed of the second rotor in RPM
 * @param Rotor_RPM3 Speed of the third rotor in RPM
 * @param Rotor_RPM4 Speed of the fourth rotor in RPM
 */
void odo_speed_step(int16_t Rotor_RPM1, int16_t Rotor_RPM2, int16_t Rotor_RPM3, int16_t Rotor_RPM4);

/**
 * @brief Calculate the position of the robot based on the speed and the period.
 * 
 * @param period Time period in seconds for the position update.
 */
void odo_position_step(float period);

/**
 * @brief Cumulate the mean speed of the robot based on the number of steps.
 * 
 * @param nbr_step Number of steps to average the speed over. 
 */
void odo_speed_cumulate_step(float nbr_step);

/**
 * @brief Get the position of the robot.
 * 
 * @return Position structure containing the robot's position (x, y, theta).
 */
Position get_position(void);

/**
 * @brief Get the speed of the robot.
 * 
 * @return Speed structure containing the robot's speed (vx, vy, vt).
 */
Speed get_speed(void);

/**
 * @brief Get the acceleration of the robot.
 * 
 * @return Acceleration structure containing the robot's acceleration (ax, ay, at).
 */
Acceleration get_acceleration(void);

/**
 * @brief Set the position of the robot.
 * 
 * @param pos Position structure containing the new position (x, y, theta).
 */
void set_position(Position pos);

/**
 * @brief Set the x-coordinate of the robot's position.
 * 
 * @param x New x-coordinate in meters.
 */
void set_position_x(float x);

/**
 * @brief Set the y-coordinate of the robot's position.
 * 
 * @param y New y-coordinate in meters.
 */
void set_position_y(float y);

/**
 * @brief Set the orientation (theta) of the robot's position.
 * 
 * @param t New orientation in radians.
 */
void set_position_t(float t);

#endif // _ODO_H_
