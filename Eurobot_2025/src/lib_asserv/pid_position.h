#ifndef _PID_POSITION_H_
#define _PID_POSITION_H_


// coefficient du PID (proportionnel, integrale, derivee) et leurs coefs de moyennage pour l'asserv en position (dist)
extern float distx_kp;
extern float distx_ki;
extern float distx_kd;
extern float disty_kp;
extern float disty_ki;
extern float disty_kd;

//coefficient du PID (proportionnel, integrale, derivee) et leurs coefs de moyennage pour l'asserv en position (angle)
extern float angle_kp;
extern float angle_ki;
extern float angle_kd;


/******************************    Fonctions    *******************************/
void pid_position_init(void);

void pid_position_reset(void); //reset des erreurs integrales des pid

float pid_distx (float err);
float pid_disty (float err);
float pid_angle (float err);



#endif

