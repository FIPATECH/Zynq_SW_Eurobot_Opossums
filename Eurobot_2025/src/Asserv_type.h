#ifndef ASSERV_TYPE_H
#define ASSERV_TYPE_H

/*****************************    Odometrie    *******************************/

// Position absolue du robot (x, y, et theta)
typedef struct {
    float x; // en metre
    float y; // en metre
    float t; // en radian
} Position;

// Vitesse et vitesse angulaire du robot
typedef struct {
    float vx; // en m/s
    float vy; // en m/s
    float vt; // en rad/s
} Speed;

// acceleration du robot (dv/dt,  d2theta/dt2   et   v*(dtheta/dt))
typedef struct {
    float ax; // en m/s2
    float ay; // en m/s2
    float at; // en rad/s2
} Acceleration;


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

#endif