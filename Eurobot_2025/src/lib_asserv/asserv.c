#include "../main.h"
#include "lib_asserv.h"


// ******************************    Variables    *******************************
int asserv_mode;
int motion_done;

float blocked_time;

float current_stop_distance;
float default_stop_distance;

Position Wanted_Pos;
Speed Wanted_Speed;

float speed_order_d;

float v_constrained = 0;

float v_max = DEFAULT_CONSTRAINT_V_MAX;
float a_max = DEFAULT_CONSTRAINT_A_MAX;

/******************************    Fonctions    *******************************/

// init de tout l'asservissement
void asserv_init(void) {
	// init des autres trucs de la lib
	odo_init();
	pid_position_init();
	speed_constrainer_init();
	pid_vitesse_init();


	// init des consignes / modes de ce fichier :
    asserv_mode = ASSERV_MODE_OFF;
    motion_done = 0;
	blocked_time = 0;

    Wanted_Pos.x = 0;
    Wanted_Pos.y = 0;
    Wanted_Pos.t = 0;
	Wanted_Speed.vx = 0;
	Wanted_Speed.vy = 0;
	Wanted_Speed.vt = 0;


    current_stop_distance = DEFAULT_STOP_DISTANCE;
    default_stop_distance = DEFAULT_STOP_DISTANCE;

}


// consignes de deplacements du robot
void motion_block(void) {
    asserv_mode = ASSERV_MODE_OFF;
}

void motion_free(void) {
    asserv_mode = ASSERV_MODE_FREE;
}

void motion_pos(Position pos) {
    current_stop_distance = default_stop_distance;
    Wanted_Pos = pos;
    asserv_mode = ASSERV_MODE_POS;
}

void motion_speed(Speed speed) {
    Wanted_Speed = speed;
    asserv_mode = ASSERV_MODE_SPEED;
}

void motion_absolute_speed(Speed speed) {
    Wanted_Speed = speed;
    asserv_mode = ASSERV_MODE_ABSOLUTE_SPEED;
}

void set_Constraint_vitesse_max(float v_max_in) {
    if (v_max_in != 0) {
        if (v_max_in <= DEFAULT_AUTHORIZED_V_MAX) {
            v_max = v_max_in;
        } else {
            v_max = DEFAULT_CONSTRAINT_V_MAX;
        }
    } else {
        v_max = DEFAULT_CONSTRAINT_V_MAX;
    }
}

void set_Constraint_acceleration_max(float a_max_in) {
    if (a_max_in != 0) {
        a_max = a_max_in;
    } else {
        a_max = DEFAULT_CONSTRAINT_A_MAX;
    }
}

// effectue un pas d'asservissement
void motion_step(void) {
    // choix en fonction du mode d'asservissement (off, position ou vitesse)
    switch (asserv_mode) {
            // si on est en roue libre
        case ASSERV_MODE_OFF:
            asserv_off_step();
            motion_done = 1;
            break;
        case ASSERV_MODE_FREE:
            asserv_free_step();
            motion_done = 1;
            break;
            // si on est en asservissement en position
        case ASSERV_MODE_POS:
            pos_asserv_step();
            motion_done = 0;
            break;
        // si on est en asservissement en vitesse
        case ASSERV_MODE_SPEED:
            speed_asserv_step();
            motion_done = 0;
            break;
        case ASSERV_MODE_ABSOLUTE_SPEED:
            absolute_speed_asserv_step();
            motion_done = 0;
            break;
		default:
			asserv_off_step();
			break;
    }
}

void asserv_off_step(void) {
	speed_order.vx = 0;
	speed_order.vy = 0;
	speed_order.vt = 0;
    Pid_Speed_En = 0;
}

void asserv_free_step(void)
{
	speed_order.vx = 0;
	speed_order.vy = 0;
	speed_order.vt = 0;
    Pid_Speed_En = 1;

	if ((fabs(speed_robot.vx) < 0.05*Speed_Max.vx) && (fabs(speed_robot.vy) < 0.05*Speed_Max.vy) && (fabs(speed_robot.vt) < 0.05*Speed_Max.vt)) {
        motion_block();
	}
}

float old_angle = 0;


void pos_asserv_step(void) {
    // --- Consignes
    float x_o = Wanted_Pos.x;
    float y_o = Wanted_Pos.y;
    float t_o = Wanted_Pos.t;

    // --- État actuel
    float x = position_robot.x;
    float y = position_robot.y;
    float t = position_robot.t;

    // --- Erreurs
    float rdx = x_o - x;
    float rdy = y_o - y;
    float d = sqrtf(rdx*rdx + rdy*rdy);                 // Erreur positionnelle
    float dt = principal_angle(t_o - t);            // Erreur angulaire

    // --- Flags de cas particuliers
    int pos_reached = d < DIST_TOL;
    int angle_reached = fabsf(dt) < ANGLE_TOL;

    static float angle_filtered = 0.0f;
    if (d > 1e-3) {
        angle_filtered = atan2f(rdy, rdx);  // Mémorise la dernière direction utile
    }

    float cos_t = cosf(t);
    float sin_t = sinf(t);

    // --- Par défaut : pas de mouvement
    speed_order.vx = 0.0f;
    speed_order.vy = 0.0f;
    speed_order.vt = 0.0f;

    // --- Cas 1 : Déplacement complet
    if (!pos_reached) {
        float angle = principal_angle(angle_filtered);

        float speed_order_d = pid_position_processing(&pid_dist, d);

        // Limitation (profil d'accélération et saturation)
        // speed_order_d = limit_float(speed_order_d, -v_max, v_max);
        // speed_order_d = limit_float(speed_order_d, v_constrained - a_max, v_constrained + a_max);
        // v_constrained = speed_order_d;

        // Décomposition en X/Y monde
        float vx_world = speed_order_d * cosf(angle);
        float vy_world = speed_order_d * sinf(angle);

        // Transformation vers repère robot
        speed_order.vx = vx_world * cos_t - vy_world * sin_t;
        speed_order.vy = vx_world * sin_t + vy_world * cos_t;
    }

    // --- Cas 2 ou 3 : commande en rotation
    if (!angle_reached) {
        speed_order.vt = pid_position_processing(&pid_angle, dt);
    }

    // --- Activation de l’asservissement vitesse
    Pid_Speed_En = 1;

    // --- Stop condition globale (position + angle atteints)
    if (pos_reached && angle_reached) {
        set_Constraint_vitesse_max(DEFAULT_CONSTRAINT_V_MAX);
        set_Constraint_vt_max(DEFAULT_CONSTRAINT_VT_MAX);
        motion_free();
        printf("Pos,done\n");
    }
}



void speed_asserv_step(void) {

	speed_order.vx = Wanted_Speed.vx;
	speed_order.vy = Wanted_Speed.vy;
	speed_order.vt = Wanted_Speed.vt;
    Pid_Speed_En = 1;
}

void absolute_speed_asserv_step(void) {
    float cos_t = cos(position_robot.t);
    float sin_t = sin(position_robot.t);
	speed_order.vx =  Wanted_Speed.vx*cos_t + Wanted_Speed.vy*sin_t;
	speed_order.vy = -Wanted_Speed.vx*sin_t + Wanted_Speed.vy*cos_t;
	speed_order.vt = Wanted_Speed.vt;
    Pid_Speed_En = 1;
}



// indique si l'asservissement en cours a termine
int Get_asserv_done(void) {
    if (asserv_mode == ASSERV_MODE_OFF) {
         return 1;
    } else {
        return 0;
    }
}

int Get_Sens_Deplacement(void) {
//    float valf = speed_order.v;
//    if (valf > 0.01)
//        return 1;
//    else if (valf < -0.01)
//        return -1;
//    else
//        return 0;
//    
    return 0;
}

// verifier qu'on est pas bloque par un obstacle
// si bloque, annule la consigne de vitesse
void asserv_check_blocked(float period) {
    if (   (fabs(speed_robot.vx - speed_order_constrained.vx) > 0.1) || 
           (fabs(speed_robot.vy - speed_order_constrained.vy) > 0.1) || 
           (fabs(speed_robot.vt - speed_order_constrained.vt) > 0.4)    ) {
        if (blocked_time >= ASSERV_BLOCK_TIME_LIMIT) {
			printf("asserv,blocked\n");
            motion_free();
            blocked_time = 0;
        }
        blocked_time += period;
    } else {
        blocked_time = 0;
    }
}

