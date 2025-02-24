#ifndef _PID_POSITION_H_
#define _PID_POSITION_H_

extern PID pid_dist;
extern PID pid_angle;

/******************************    Fonctions    *******************************/
void pid_position_init(void);

void pid_position_reset(void); //reset des erreurs integrales des pid

float pid_position_processing(PID *pid, float err);

#endif

