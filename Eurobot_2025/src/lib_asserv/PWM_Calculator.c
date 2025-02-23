#include "lib_asserv.h"


void Asserv_PWM_calculator(float *commande_1, float *commande_2, float *commande_3) {
    // maj des consignes des PID
	*commande_1 = pid_v1(Speed_Order_1 - Speed_1);
	*commande_2 = pid_v2(Speed_Order_2 - Speed_2);
	*commande_3 = pid_v3(Speed_Order_3 - Speed_3);
}
