#include "lib_asserv.h"

/******************************    Variables    *******************************/
// coefficient du PID (proportionnel, integrale, derivee) et leurs coefs de moyennage pour l'asserv en vitesse lineaire
float v_kp;
float v_ki;
float v_kd;

////////////////////////// Vitesse lineaire ////////////////////////////
// Etat des valeurs du PID (erreur, integrale de l'erreur, derivee de l'erreur)
float err_v1;
float err_int_v1;
float err_der_v1;

float err_v2;
float err_int_v2;
float err_der_v2;

float err_v3;
float err_int_v3;
float err_der_v3;


uint8_t Pid_Speed_En = 0;

/******************************    Fonctions    *******************************/
void pid_vitesse_init (void) {
    // PID pour l'asserv en vitesse
    v_kp = DEFAULT_PID_V_LIN_KP;
    v_ki = DEFAULT_PID_V_LIN_KI;
    v_kd = DEFAULT_PID_V_LIN_KD;
}

void pid_vitesse_reset (void) {
    err_v1 = 0;
    err_int_v1 = 0;
	err_der_v1 = 0;
    err_v2 = 0;
    err_int_v2 = 0;
    err_der_v2 = 0;
    err_v3 = 0;
    err_int_v3 = 0;
    err_der_v3 = 0;
}

/******************************    Fonctions Utilitaires   *******************************/
float pid_v1 (float err) {
    if (Pid_Speed_En) {
        //maj de la derivee de l'erreur du PID
        err_der_v1 = err - err_v1;

        //maj de l'erreur du PID
        err_v1 = err;
        // maj de l'integrale de l'erreur du PID
        // avec limite pour que KP+KI soit toujours entre 100 et - 100 (% de PWM)
        // ne gere que si KI != 0
        if (v_ki != 0) {
            err_int_v1 += err;
            float max_int = (10000 - (err_v1 * v_kp)) / v_ki;
            float min_int = (-10000 - (err_v1 * v_kp)) / v_ki;
            err_int_v1 = limit_float(err_int_v1, min_int, max_int);
        }
        return (err_v1 * v_kp) + (err_int_v1 * v_ki) + (err_der_v1 * v_kd);
    } else {
        err_v1 = 0;
        err_der_v1 = 0;
        err_int_v1 = 0;
        return 0;
    }
}

float pid_v2 (float err) {
    if (Pid_Speed_En) {
        //maj de la derivee de l'erreur du PID
        err_der_v2 = err - err_v2;

        //maj de l'erreur du PID
        err_v2 = err;
        // maj de l'integrale de l'erreur du PID
        // avec limite pour que KP+KI soit toujours entre 100 et - 100 (% de PWM)
        // ne gere que si KI != 0
        if (v_ki != 0) {
            err_int_v2 += err;
            float max_int = (10000 - (err_v2 * v_kp)) / v_ki;
            float min_int = (-10000 - (err_v2 * v_kp)) / v_ki;
            err_int_v2 = limit_float(err_int_v2, min_int, max_int);
        }
        return (err_v2 * v_kp) + (err_int_v2 * v_ki) + (err_der_v2 * v_kd);
    } else {
        err_v2 = 0;
        err_der_v2 = 0;
        err_int_v2 = 0;
        return 0;
    }
}

float pid_v3 (float err) {
    if (Pid_Speed_En) {
        //maj de la derivee de l'erreur du PID
        err_der_v3 = err - err_v3;

        //maj de l'erreur du PID
        err_v3 = err;

        // maj de l'integrale de l'erreur du PID
        // avec limite pour que KP+KI soit toujours entre 100 et - 100 (% de PWM)
        // ne gere que si KI != 0
        if (v_ki != 0) {
            err_int_v3 += err;
            float max_int = (10000 - (err_v3 * v_kp)) / v_ki;
            float min_int = (-10000 - (err_v3 * v_kp)) / v_ki;
            err_int_v3 = limit_float(err_int_v3, min_int, max_int);
        }
        return (err_v3 * v_kp) + (err_int_v3 * v_ki) + (err_der_v3 * v_kd);
    } else {
        err_v3 = 0;
        err_der_v3 = 0;
        err_int_v3 = 0;
        return 0;
    }
}

