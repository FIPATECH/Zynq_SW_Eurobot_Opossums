
#include "lib_asserv.h"

/******************************    Variables    *******************************/
// coefficient du PID (proportionnel, integrale, derivee) et leurs coefs de moyennage pour l'asserv en position (dist)
float distx_kp;
float distx_ki;
float distx_kd;
float disty_kp;
float disty_ki;
float disty_kd;

//coefficient du PID (proportionnel, integrale, derivee) et leurs coefs de moyennage pour l'asserv en position (angle)
float angle_kp;
float angle_ki;
float angle_kd;

///////////////////////// POS_DIST /////////////////////////
// Etat des valeurs du PID (erreur, integrale de l'erreur, derivee de l'erreur) et borne de l'integrale
float pid_distx_err;
float pid_distx_err_int;
float pid_distx_err_der;
float pid_disty_err;
float pid_disty_err_int;
float pid_disty_err_der;

////////////////////////// POS_ANGLE ////////////////////////
// Etat des valeurs du PID (erreur, integrale de l'erreur, derivee de l'erreur) et borne de l'integrale
float pid_angle_err;
float pid_angle_err_int;
float pid_angle_err_der;

/******************************    Fonctions    *******************************/
void pid_position_init(void) {
    // PID pour l'asserv en position (dist)
    distx_kp = DEFAULT_PID_DIST_KP;
    distx_ki = DEFAULT_PID_DIST_KI;
    distx_kd = DEFAULT_PID_DIST_KD;
    disty_kp = DEFAULT_PID_DIST_KP;
    disty_ki = DEFAULT_PID_DIST_KI;
    disty_kd = DEFAULT_PID_DIST_KD;

    // PID pour l'asserv en position angulaire (angle)
    angle_kp = DEFAULT_PID_ANGLE_KP;
    angle_ki = DEFAULT_PID_ANGLE_KI;
    angle_kd = DEFAULT_PID_ANGLE_KD;

    pid_position_reset();
}

void pid_position_reset(void) {
    // PID pour l'asserv en position (dist)
    pid_distx_err = 0;
    pid_distx_err_int = 0;
    pid_distx_err_der = 0;
    pid_disty_err = 0;
    pid_disty_err_int = 0;
    pid_disty_err_der = 0;
    // PID pour l'asserv en position angulaire (angle)
    pid_angle_err = 0;
    pid_angle_err_int = 0;
    pid_angle_err_der = 0;
}

/******************************    Fonctions Utilitaires   *******************************/
float pid_distx (float err) {
	// maj de la derivee de l'erreur du PID
    float pid_distx_derr = err - pid_distx_err;

    // maj de l'erreur du PID
    pid_distx_err = err;

	// maj de l'integrale
	if (distx_ki != 0) {
		pid_distx_err_int += err;
	} else {
		pid_distx_err_int = 0;
	}

	return (pid_distx_err * distx_kp) + (pid_distx_err_int * distx_ki) + (pid_distx_derr * distx_kd);
}

float pid_disty (float err) {
	// maj de la derivee de l'erreur du PID
    float pid_disty_derr = err - pid_disty_err;

    // maj de l'erreur du PID
    pid_disty_err = err;

	// maj de l'integrale
	if (disty_ki != 0) {
		pid_disty_err_int += err;
	} else {
		pid_disty_err_int = 0;
	}

	return (pid_disty_err * disty_kp) + (pid_disty_err_int * disty_ki) + (pid_disty_derr * disty_kd);
}

float pid_angle (float err) {
	// maj de la derivee de l'erreur du PID
    float pid_angle_derr = err - pid_angle_err;

    // maj de l'erreur du PID
    pid_angle_err = err;

	// maj de l'integrale
	if (angle_ki != 0) {
		pid_angle_err_int += err;
	} else {
		pid_angle_err_int = 0;
	}

	return (pid_angle_err * angle_kp) + (pid_angle_err_int * angle_ki) + (pid_angle_derr * angle_kd);
}


