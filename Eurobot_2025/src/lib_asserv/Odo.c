#include "lib_asserv.h"


/******************************    Variables    *******************************/
float robot_wheel_distance;

Position position_robot;    // en repere absolu
Speed speed_robot;                  // en repere relatif
Acceleration acceleration_robot;    // en repere relatif

float Speed_1, Speed_2, Speed_3;

/******************************    Fonctions    *******************************/

// initialiser l'odometrie
void odo_init(void) {
    odo_set_spacing(DEFAULT_ODO_SPACING);

    position_robot.x = 0;
    position_robot.y = 0;
    position_robot.t = 0;

    position_robot_odom.x = 0;
    position_robot_odom.y = 0;
    position_robot_odom.t = 0;

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

void odo_position_step(float* dx, float* dy, float* delta_t) {   
    float dx_local = (speed_robot.vx * 0.001f);
    float dy_local = (speed_robot.vy * 0.001f);
    float dt = (speed_robot.vt * 0.001f);
    
    float cos_t = cosf(position_robot.t);
    float sin_t = sinf(position_robot.t);

    float dx_global = dx_local * cos_t - dy_local * sin_t;
    float dy_global = dx_local * sin_t + dy_local * cos_t;

    *dx = dx_global;
    *dy = dy_global;
    *delta_t = dt;

    position_robot.x += dx_global;
    position_robot.y += dy_global;
    position_robot.t = principal_angle(position_robot.t + dt);
}


float odo_dist_roue(int16_t delta_angle_motor) {
    static const float facteur_conversion = TWO_PI / (MOTOR_ANGLE_CODEUR_MAX * 36.0f); // Conversion en radian

    // Calcul de la variation d'angle normalisée
    if (delta_angle_motor > (MOTOR_ANGLE_CODEUR_MAX / 2)) {
        delta_angle_motor -= MOTOR_ANGLE_CODEUR_MAX;
    } else if (delta_angle_motor < -(MOTOR_ANGLE_CODEUR_MAX / 2)) {
        delta_angle_motor += MOTOR_ANGLE_CODEUR_MAX;
    }

    float deltaAngleRad = delta_angle_motor * facteur_conversion; // Conversion en radian
    float distance = DEFAULT_WHEEL_RADIUS * deltaAngleRad; // Calcul de la distance parcourue
    return distance; // Retourne la distance parcourue
}

void odo_speed_step(int16_t Rotor_RPM1, int16_t Rotor_RPM2, int16_t Rotor_RPM3) {
    // vitesse roues en m/s
    Speed_1 = (float)((Rotor_RPM1*PI*DEFAULT_SIZE_WHEEL)/(36.0*60.0)); //36 reducteur du moteur
    Speed_2 = (float)((Rotor_RPM2*PI*DEFAULT_SIZE_WHEEL)/(36.0*60.0));
    Speed_3 = (float)((Rotor_RPM3*PI*DEFAULT_SIZE_WHEEL)/(36.0*60.0));

    // maj des vitesses
    speed_robot.vx = (2.0f/3.0f) * (Speed_1) - (1.0f/3.0f) * (Speed_2 + Speed_3);
    speed_robot.vy = (sqrtf(3.0f) / 3.0f) * (Speed_2 - Speed_3); // translation avant (X robot)
    speed_robot.vt = -(Speed_1 + Speed_2 + Speed_3) / (3.0f * robot_wheel_distance);
}


void set_position(Position pos) {
    position_robot = pos;
    position_robot_odom = pos;
}

void set_position_x(float x) {
    position_robot.x = x;
    position_robot_odom.x = x;
}

void set_position_y(float y) {
    position_robot.y = y;
    position_robot_odom.y = y;
}

void set_position_t(float t) {
    position_robot.t = t;
    position_robot_odom.t = t;
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
