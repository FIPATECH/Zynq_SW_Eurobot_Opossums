#include "Lib_Asserv.h"

/******************************    Variables    *******************************/

float meter_by_tic_droit;
float meter_by_tic_gauche;
float spacing;

int tics_g;
int tics_d;
float Odo_Cumul_d = 0;
float Odo_Cumul_dt = 0;


Acceleration acceleration_robot;
Speed speed_robot;
Position position_robot;

/******************************    Fonctions    *******************************/

// initialiser l'odometrie
void odo_init(void) {
    odo_set_tic_by_meter(DEFAULT_ODO_TIC_BY_M_D, DEFAULT_ODO_TIC_BY_M_G);
    odo_set_spacing(DEFAULT_ODO_SPACING);
    tics_d = 0;
    tics_g = 0;
}
// assigner des valeurs aux coefs (relations tic/metre et entraxe)
void odo_set_tic_by_meter(double param_tic_by_meter_droit, double param_tic_by_meter_gauche) {
    meter_by_tic_droit = 1 / (param_tic_by_meter_droit);
    meter_by_tic_gauche = 1 / (param_tic_by_meter_gauche);
}

// assigner une valeur e l'ecart entre les roues d'odometrie
void odo_set_spacing(float param_spacing) {
    spacing = param_spacing;
}

void odo_position_step(int qei_g, int qei_d) {
    // calculs intermediaires des deplacements gauches et droites
    float dg = (float) (((int)(qei_g - tics_g)) * meter_by_tic_gauche);
    float dd = (float) (((int)(qei_d - tics_d)) * meter_by_tic_droit);
    // distance et angle de deplacement depuis le dernier step
    float d = (dd + dg) / 2;
    float dt = atan2f((dd - dg), spacing);

    Odo_Cumul_d += d;
    Odo_Cumul_dt += dt;

    // maj des tics
    tics_g = qei_g;
    tics_d = qei_d;

    // maj des positions
    position_robot.x += d * cos(position_robot.t);
    position_robot.y += d * sin(position_robot.t);
    position_robot.t = principal_angle(position_robot.t + dt);
}

void odo_speed_step(float period)
{
    // sauvegarde des anciennes vitesses
    float v = speed_robot.v;
    float vt = speed_robot.vt;

    // maj des vitesses
    speed_robot.v = Odo_Cumul_d / period;
    speed_robot.vt = Odo_Cumul_dt / period;


    Odo_Cumul_d = 0;
    Odo_Cumul_dt = 0;

    // maj des accelerations
    acceleration_robot.a = speed_robot.v - v;
    acceleration_robot.at = speed_robot.vt - vt;
    acceleration_robot.v_vt = (speed_robot.v)*(speed_robot.vt);
}


void set_position(Position pos) {
    position_robot = pos;
}

void set_position_x(float x) {
    position_robot.x = x;
}

void set_position_y(float y) {
    position_robot.y = y;
}

void set_position_t(float t) {
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
