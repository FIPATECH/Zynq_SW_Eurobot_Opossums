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

void odo_position_step(int16_t delta_angle_motor_1, int16_t delta_angle_motor_2, int16_t delta_angle_motor_3) {   
    float dist_motor_1 = odo_dist_roue(delta_angle_motor_1);
    float dist_motor_2 = odo_dist_roue(delta_angle_motor_2);
    float dist_motor_3 = odo_dist_roue(delta_angle_motor_3);

    // Nouvelle formule correcte pour odométrie holonome 3 roues
    float dx = (2.0f / 3.0f) * (dist_motor_1 - 0.5f * dist_motor_2 - 0.5f * dist_motor_3);
    float dy = (2.0f / 3.0f) * ((sqrtf(3.0f) / 2.0f) * (dist_motor_2 - dist_motor_3));
    float dt = -(dist_motor_1 + dist_motor_2 + dist_motor_3) / (3.0f * robot_wheel_distance);

    float cos_t = cosf(position_robot.t);
    float sin_t = sinf(position_robot.t);

    float rdx = dx * cos_t - dy * sin_t;
    float rdy = dx * sin_t + dy * cos_t;

    position_robot.x += rdx;
    position_robot.y += rdy;
    position_robot.t = principal_angle(position_robot.t + dt);
}

float odo_dist_roue(int16_t delta_angle_motor) {
    static const float facteur_conversion = TWO_PI / (MOTOR_ANGLE_CODEUR_MAX * 36); // Conversion en radian

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

void odo_speed_step(int16_t RPM1, int16_t RPM2, int16_t RPM3) {
    float v1 = (RPM1 * PI * DEFAULT_SIZE_WHEEL) / (36.0f * 60.0f);
    float v2 = (RPM2 * PI * DEFAULT_SIZE_WHEEL) / (36.0f * 60.0f);
    float v3 = (RPM3 * PI * DEFAULT_SIZE_WHEEL) / (36.0f * 60.0f);

    float vx_r = (2.0f / 3.0f) * (v1 - 0.5f * v2 - 0.5f * v3);
    float vy_r = (2.0f / 3.0f) * (cosf(60) * (v2 - v3));
    float vt = -(v1 + v2 + v3) / (3.0f * robot_wheel_distance);

    static float last_vx = 0.0f, last_vy = 0.0f, last_vt = 0.0f;
    acceleration_robot.ax = vx_r - last_vx;
    acceleration_robot.ay = vy_r - last_vy;
    acceleration_robot.at = vt - last_vt;

    last_vx = vx_r;
    last_vy = vy_r;
    last_vt = vt;

    speed_robot.vx = vx_r;
    speed_robot.vy = vy_r;
    speed_robot.vt = vt;
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
