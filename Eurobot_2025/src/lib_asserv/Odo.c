#include "lib_asserv.h"


/******************************    Variables    *******************************/
float robot_wheel_distance;

Speed speed_robot;                  // en repere relatif
Speed speed_robot_odom;
Speed cumulated_speed;    
Position position_robot; // Position odometrique
Acceleration acceleration_robot;    // en repere relatif

float Speed_1, Speed_2, Speed_3;
float Cumulated_Speed_1, Cumulated_Speed_2, Cumulated_Speed_3;


/******************************    Fonctions    *******************************/

// initialiser l'odometrie
void odo_init(void) {
    odo_set_spacing(DEFAULT_ODO_SPACING);

    position_robot_predict.x = 0;
    position_robot_predict.y = 0;
    position_robot_predict.t = 0;

    position_robot.x = 0;
    position_robot.y = 0;
    position_robot.t = 0;

    speed_robot.vx = 0;
    speed_robot.vy = 0;
    speed_robot.vt = 0;

    acceleration_robot.ax = 0;
    acceleration_robot.ay = 0;
    acceleration_robot.at = 0;

    xil_printf("Odo init done\n");
}

// assigner une valeur e l'ecart entre les roues d'odometrie
void odo_set_spacing(float param_spacing) {
    robot_wheel_distance = param_spacing;
}

void odo_speed_step(int16_t Rotor_RPM1, int16_t Rotor_RPM2, int16_t Rotor_RPM3) {
    // vitesse roues en m/s
    float Odo_Speed_1 = (float)((Rotor_RPM1*PI*DEFAULT_SIZE_WHEEL)/(36.0*60.0)); //36 reducteur du moteur
    float Odo_Speed_2 = (float)((Rotor_RPM2*PI*DEFAULT_SIZE_WHEEL)/(36.0*60.0));
    float Odo_Speed_3 = (float)((Rotor_RPM3*PI*DEFAULT_SIZE_WHEEL)/(36.0*60.0));

    // vitesse cumulee roues
    Cumulated_Speed_1 += Odo_Speed_1;
    Cumulated_Speed_2 += Odo_Speed_2;
    Cumulated_Speed_3 += Odo_Speed_3;
    
    // maj des vitesses odometrique pur pour le calcul de la position
    speed_robot_odom.vx = (2.0f/3.0f) * (Odo_Speed_1) - (1.0f/3.0f) * (Odo_Speed_2 + Odo_Speed_3);
    speed_robot_odom.vy = (sqrtf(3.0f) / 3.0f) * (Odo_Speed_2 - Odo_Speed_3); // translation avant (X robot)
    speed_robot_odom.vt = -(Odo_Speed_1 + Odo_Speed_2 + Odo_Speed_3) / (3.0f * robot_wheel_distance);

    // maj des vitesses odometrique cumulé pour le calcul de la vitesse
    cumulated_speed.vx += speed_robot_odom.vx;
    cumulated_speed.vy += speed_robot_odom.vy;
    cumulated_speed.vt += speed_robot_odom.vt;
}

void odo_position_step(float period) {   
    float dx_local = (speed_robot_odom.vx * period);
    float dy_local = (speed_robot_odom.vy * period);
    float dt = (speed_robot_odom.vt * period);
    
    float cos_t = cosf(position_robot_predict.t);
    float sin_t = sinf(position_robot_predict.t);

    float dx_global = dx_local * cos_t - dy_local * sin_t;
    float dy_global = dx_local * sin_t + dy_local * cos_t;

    position_robot_predict.x += dx_global;
    position_robot_predict.y += dy_global;
    position_robot_predict.t = principal_angle(position_robot_predict.t + dt);
}

void odo_speed_cumulate_step(float nbr_step) {
    Speed_1 = (Cumulated_Speed_1 / nbr_step);
    Speed_2 = (Cumulated_Speed_2 / nbr_step);
    Speed_3 = (Cumulated_Speed_3 / nbr_step);

    Cumulated_Speed_1 = 0;
    Cumulated_Speed_2 = 0;
    Cumulated_Speed_3 = 0;
    
    speed_robot.vx = (cumulated_speed.vx / nbr_step);
    speed_robot.vy = (cumulated_speed.vy / nbr_step);
    speed_robot.vt = (cumulated_speed.vt / nbr_step);

    cumulated_speed.vx = 0;
    cumulated_speed.vy = 0;
    cumulated_speed.vt = 0;
}


void set_position(Position pos) {
    position_robot_predict = pos;
    position_robot = pos;
}

void set_position_x(float x) {
    position_robot_predict.x = x;
    position_robot.x = x;
}

void set_position_y(float y) {
    position_robot_predict.y = y;
    position_robot.y = y;
}

void set_position_t(float t) {
    position_robot_predict.t = t;
    position_robot.t = t;
}


Position get_position(void) {
    return position_robot;
}

Speed get_speed(void) {
    return speed_robot;
}

Acceleration get_acceleration(void) {
    return acceleration_robot;
}
