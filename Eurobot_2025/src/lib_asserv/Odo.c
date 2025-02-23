#include "lib_asserv.h"


/******************************    Variables    *******************************/

float meter_by_tic;
float robot_wheel_distance;

int16_t old_qeif, old_qeibl, old_qeibr;

float Odo_Cumul_dx = 0;
float Odo_Cumul_dy = 0;
float Odo_Cumul_dt = 0;

float Odo_Cumul_df  = 0;
float Odo_Cumul_dbl = 0;
float Odo_Cumul_dbr = 0;


Position position_robot;    // en repere absolu
Speed speed_robot;                  // en repere relatif
Acceleration acceleration_robot;    // en repere relatif

float Speed_1, Speed_2, Speed_3;

/******************************    Fonctions    *******************************/

// initialiser l'odometrie
void odo_init(void) {
    odo_set_tic_by_meter(DEFAULT_ODO_TIC_BY_M);
    odo_set_spacing(DEFAULT_ODO_SPACING);
    old_qeif  = 0;
    old_qeibl = 0;
    old_qeibr = 0;
}
// assigner des valeurs aux coefs (relations tic/metre et entraxe)
void odo_set_tic_by_meter(double param_tic_by_meter) {
    meter_by_tic      = 1 / (param_tic_by_meter);
}

// assigner une valeur e l'ecart entre les roues d'odometrie
void odo_set_spacing(float param_spacing) {
    robot_wheel_distance = param_spacing;
}

void odo_position_step(float period) {   
    float dx = (speed_robot.vx * period);
    float dy = (speed_robot.vy * period);
    float dt = (speed_robot.vt * period);
    
    float rdx = dx*cos(position_robot.t) - dy*sin(position_robot.t);
    float rdy = dx*sin(position_robot.t) + dy*cos(position_robot.t);
    // maj des positions
    position_robot.x += rdx;
    position_robot.y += rdy;
    position_robot.t = principal_angle(position_robot.t + dt);
}

void odo_speed_step(int16_t Rotor_RPM1, int16_t Rotor_RPM2, int16_t Rotor_RPM3) {
    // vitesse roues en m/s
    Speed_1 = (float)((Rotor_RPM1*PI*DEFAULT_SIZE_WHEEL)/(36.0*60.0));
    Speed_2 = (float)((Rotor_RPM2*PI*DEFAULT_SIZE_WHEEL)/(36.0*60.0));
    Speed_3 = (float)((Rotor_RPM3*PI*DEFAULT_SIZE_WHEEL)/(36.0*60.0));

    // sauvegarde des anciennes vitesses
    float vx = speed_robot.vx;
    float vy = speed_robot.vy;
    float vt = speed_robot.vt;

    // maj des vitesses
    speed_robot.vx = -0.5*(Speed_2 - Speed_3)/sin(PI/3);
    speed_robot.vy = 0.5*(Speed_1 -(Speed_2 + Speed_3));
    speed_robot.vt = -(Speed_1 + Speed_2 + Speed_3) / (3*robot_wheel_distance);

    // maj des accelerations
    acceleration_robot.ax = speed_robot.vx - vx;
    acceleration_robot.ay = speed_robot.vy - vy;
    acceleration_robot.at = speed_robot.vt - vt;
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
