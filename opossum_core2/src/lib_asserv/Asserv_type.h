#ifndef ASSERV_TYPE_H
#define ASSERV_TYPE_H

/*****************************    Odometrie    *******************************/

// Position absolue du robot (x, y, et theta)
typedef struct {
    float x; // en metre
    float y; // en metre
    float t; // en radian
} Position;

extern Position position_robot_odom;
extern Position position_robot;


// Vitesse et vitesse angulaire du robot
typedef struct {
    float vx; // en m/s
    float vy; // en m/s
    float vt; // en rad/s
} Speed;

extern Position Wanted_Pos;
extern Speed Wanted_Speed;
extern Speed speed_robot;
extern Speed speed_robot_odom;



// acceleration du robot (dv/dt,  d2theta/dt2   et   v*(dtheta/dt))
typedef struct {
    float ax; // en m/s2
    float ay; // en m/s2
    float at; // en rad/s2
} Acceleration;

extern Acceleration acceleration_robot;


/**************************** PID  *****************************/
typedef struct {
    float kp;
    float ki;
    float kd;
} PID_coef;

typedef struct {
    float err;
    float err_int;
    float err_der;
} PID_err;

typedef struct {
    PID_coef coef;
    PID_err err1;
    PID_err err2;
    PID_err err3;
} PID_speed;


typedef struct{
    float command1;
    float command2;
    float command3;
} ESC_Command;

extern ESC_Command Consigne;
extern ESC_Command Wanted_Forced_Consigne;
extern ESC_Command old_Consigne;



typedef struct {
    int odo_step_1; // step 1: calcul de la vitesse du robot
    int odo_step_2; // step 2: calcul du kalman + history
    int odo_step_3; // step 3: calcul de la vitesse du robot
    int motion_step; // step 4: calcul de la position du robot
    int speed_constrain_step; // step 5: contrainte de vitesse
    int acceleration_constrain_step; // step 6: contrainte d'acceleration
    int consigne_step; // step 7: calcul de la consigne
    int pwm_step; // step 8: calcul du PWM
    int transmit_step; // step 9: transmission des ordres aux moteurs
} Asserv_Step_Timing;

Asserv_Step_Timing asserv_step_timing;

#endif