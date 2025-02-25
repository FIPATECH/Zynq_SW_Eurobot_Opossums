#include "lib_asserv.h"


void Asserv_PWM_calculator(float *commande_1, float *commande_2, float *commande_3) {
    // maj des consignes des PID
	float err1 = Speed_Order_1 - Speed_1;
	float err2 = Speed_Order_2 - Speed_2;
	float err3 = Speed_Order_3 - Speed_3;

	// calcul des commandes
	*commande_1, *commande_2, *commande_3 = pid_speed_processing(&pid_speed, err1, err2, err3);
}
