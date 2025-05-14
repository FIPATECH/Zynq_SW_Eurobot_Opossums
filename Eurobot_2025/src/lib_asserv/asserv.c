#include "../main.h"
#include "lib_asserv.h"


// ******************************    Variables    *******************************
int asserv_mode; // asservissement off par defaut (refer to asserv_init function)
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

    // init acccel and speed constrainers
	speed_constrainer_init();
    acceleration_constrainer_init();

    // init PID
	pid_vitesse_init();

    // init kalman
    kalman_init(&position_robot);

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
    asserv_mode = ASSERV_MODE_BREAK;
}

void motion_off(void) {
    asserv_mode = ASSERV_MODE_OFF;
}

void motion_free(void) {
    asserv_mode = ASSERV_MODE_FREE;
}

void motion_pos(Position pos) {
    current_stop_distance = default_stop_distance;
    Wanted_Pos = pos;
    robot_a_max = DEFAULT_CONSTRAINT_A_ROUE;
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


// effectue un pas d'asservissement
void motion_step(void) {
    // choix en fonction du mode d'asservissement (off, position ou vitesse)
    switch (asserv_mode) {
        // si on est en roue libre
        case ASSERV_MODE_OFF:
            asserv_off_step();
            motion_done = 1;
            break;
        // si on s'arrête mais qu'on ne doit pas freiner (pas de roue libre ni de blocage roues)
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
        // si on doit freiner en urgence
        case ASSERV_MODE_BREAK:
            speed_asserv_break_step();
            motion_done = 0;
            break;
        // si on est en asservissement en vitesse absolue
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
    acceleration_constrainer_init();
	speed_order.vx = 0;
	speed_order.vy = 0;
	speed_order.vt = 0;
    Pid_Speed_En = 0;
}

void asserv_free_step(void)
{   
    acceleration_constrainer_init();
	speed_order.vx = 0;
	speed_order.vy = 0;
	speed_order.vt = 0;
    Pid_Speed_En = 1;

	if ((fabs(speed_robot.vx) < 0.05*robot_v_max) && (fabs(speed_robot.vy) < 0.05*robot_v_max) && (fabs(speed_robot.vt) < 0.05*robot_vt_max)) {
        motion_off();
	}
}

void speed_asserv_break_step(void) {
    // break only if the robot is moving 
    if (fabs(speed_robot.vx) > 0.1 || fabs(speed_robot.vy) > 0.1 || fabs(speed_robot.vt) > 0.1){
        robot_a_max = 10 * DEFAULT_CONSTRAINT_A_MAX;
    }
    speed_order.vx = 0;
    speed_order.vy = 0;
    speed_order.vt = 0;
    Pid_Speed_En = 1;

    // if the robot is almost not moving anymore, free the motion
    if (fabs(speed_robot.vx) < 0.05 && fabs(speed_robot.vy) < 0.05 && fabs(speed_robot.vt) < 0.05) {
        robot_a_max = DEFAULT_CONSTRAINT_A_MAX;
        motion_free();
        printf("Break,done\n");
        motion_done = 1;
    }
}

float old_angle = 0;

void pos_asserv_step(void) {
    acceleration_constrainer_init();
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
    float d = sqrtf(rdx*rdx + rdy*rdy);             // Erreur positionnelle
    float dt = principal_angle(t_o - t);            // Erreur angulaire

    float cos_t = cosf(t);
    float sin_t = sinf(t);

    // --- Par défaut : pas de mouvement
    speed_order.vx = 0.0f;
    speed_order.vy = 0.0f;
    speed_order.vt = 0.0f;

    float angle = atan2f(rdy, rdx);

    // --- Calcul de la vitesse radiale
    float speed_order_d = radial_speed_calculation(d); // vitesse de consigne radiale
    speed_order_d = limit_float(speed_order_d, -DEFAULT_CONSTRAINT_V_MAX, DEFAULT_CONSTRAINT_V_MAX);
    
    // Décomposition en X/Y monde
    float vx_world = speed_order_d * cosf(angle);
    float vy_world = speed_order_d * sinf(angle);

    // Transformation vers repère robot
    speed_order.vx = vx_world * cos_t + vy_world * sin_t;
    speed_order.vy = - vx_world * sin_t + vy_world * cos_t;

    // --- Calcul de la vitesse angulaire
    float speed_order_vt =  angular_speed_calculation(dt);
    speed_order.vt = limit_float(speed_order_vt, -DEFAULT_CONSTRAINT_VT_MAX, DEFAULT_CONSTRAINT_VT_MAX) * exp(-d);

    // --- Activation de l’asservissement vitesse
    Pid_Speed_En = 1;

    // --- Stop condition globale (position + angle atteints)
    if ((d < current_stop_distance) && (fabs(dt) < DEFAULT_STOP_ANGLE)) {
        speed_constrainer_init();
        acceleration_constrainer_init();
        motion_free();
        printf("Pos,done\n");
    }
}

float radial_speed_calculation(float distance) {
    return sqrtf(2.0f * DEFAULT_CONSTRAINT_A_MAX * distance * 0.7f);
}

float angular_speed_calculation(float angle) {
    float fabs_angle = fabsf(angle);
    int sign = 1;
    if (angle < 0) {
        sign = -1;
    }
    return sign * sqrtf(2.0f * DEFAULT_CONSTRAINT_AT_MAX * fabs_angle * 0.7f);
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

