#ifndef __PID_SPEED_H_
#define __PID_SPEED_H_

// coefficient du PID (proportionnel, integrale, derivee) et leurs coefs de moyennage pour l'asserv en vitesse lineaire
extern float v_kp;
extern float v_ki;
extern float v_kd;

extern float err_v1;
extern float err_int_v1;
extern float err_der_v1;

extern float err_v2;
extern float err_int_v2;
extern float err_der_v2;

extern float err_v3;
extern float err_int_v3;
extern float err_der_v3;

extern uint8_t Pid_Speed_En;

/******************************    Fonctions    *******************************/
void pid_vitesse_init(void);

void pid_vitesse_reset(void); //reset des erreurs integrales des pid

/******************************    Fonctions Utilitaires   *******************************/
float pid_v1(float err);
float pid_v2(float err);
float pid_v3(float err);

#endif
