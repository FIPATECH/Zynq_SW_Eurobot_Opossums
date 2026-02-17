
#include "lib_asserv.h"


Acceleration Accel_Max;

Speed speed_order;
Speed speed_order_constrained;
Speed speed_order_constrained_1;

float Speed_Order_1, Speed_Order_2, Speed_Order_3, Speed_Order_4;

float robot_v_max;
float robot_vt_max;

float robot_a_max;
float robot_at_max;

void speed_constrainer_init(void)
{
    robot_v_max   = DEFAULT_CONSTRAINT_V_MAX;
    robot_vt_max  = DEFAULT_CONSTRAINT_VT_MAX;
}

void acceleration_constrainer_init(void)
{
    robot_a_max = DEFAULT_CONSTRAINT_A_MAX;
    robot_at_max = DEFAULT_CONSTRAINT_AT_MAX;
}

void constrain_speed_order(void) {
    speed_order_constrained_1.vx = speed_order.vx;
    speed_order_constrained_1.vy = speed_order.vy;
    speed_order_constrained_1.vt = speed_order.vt;

    float v_linear = sqrtf(speed_order_constrained_1.vx * speed_order_constrained_1.vx + speed_order_constrained_1.vy * speed_order_constrained_1.vy);

    if(v_linear > robot_v_max) {
        float scale = robot_v_max / v_linear;
        speed_order_constrained_1.vx *= scale;
        speed_order_constrained_1.vy *= scale;
    }
    speed_order_constrained_1.vt = limit_float(speed_order_constrained_1.vt, -robot_vt_max, robot_vt_max);
}

void constrain_acceleration_order(float period) {
    float delta_v_max   = robot_a_max * period;
    float delta_vt_max  = robot_at_max * period;

    // process old speed constrained (be aware of the rotation)
    float delta_angle = speed_order_constrained.vt * period;
    float cos_angle = cosf(delta_angle);
    float sin_angle = sinf(delta_angle);
    float previous_vx_order = speed_order_constrained.vx * cos_angle + speed_order_constrained.vy * sin_angle;
    float previous_vy_order = speed_order_constrained.vy * cos_angle - speed_order_constrained.vx * sin_angle;

    // process acceleration steps
    float delta_vx = speed_order_constrained_1.vx - previous_vx_order;
    float delta_vy = speed_order_constrained_1.vy - previous_vy_order;

    float dv_linear = sqrtf(delta_vx * delta_vx + delta_vy * delta_vy);
    if (emergency_break_requested){
        // in case the BREAK command is requested from the user
        speed_order_constrained.vx = 0;
        speed_order_constrained.vy = 0;
        speed_order_constrained.vt = 0;
    } else {
        if (dv_linear > delta_v_max) {
            float scale = delta_v_max / dv_linear;
            speed_order_constrained.vx = previous_vx_order + delta_vx * scale;
            speed_order_constrained.vy = previous_vy_order + delta_vy * scale;
        } else {
            speed_order_constrained.vx = speed_order_constrained_1.vx;
            speed_order_constrained.vy = speed_order_constrained_1.vy;
        }

        speed_order_constrained.vt = limit_float(
            speed_order_constrained_1.vt, 
            speed_order_constrained.vt - delta_vt_max, 
            speed_order_constrained.vt + delta_vt_max
        );
    }

    float inv_sqrt2 = 0.70710678f;  // 1 / sqrt(2)

    float raw_s1 = inv_sqrt2 * (-speed_order_constrained.vx + speed_order_constrained.vy) - robot_wheel_distance * speed_order_constrained.vt;  // avant-droite
    float raw_s2 = inv_sqrt2 * ( speed_order_constrained.vx + speed_order_constrained.vy) - robot_wheel_distance * speed_order_constrained.vt;  // arrière-droite
    float raw_s3 = inv_sqrt2 * ( speed_order_constrained.vx - speed_order_constrained.vy) - robot_wheel_distance * speed_order_constrained.vt;  // arrière-gauche
    float raw_s4 = inv_sqrt2 * (-speed_order_constrained.vx - speed_order_constrained.vy) - robot_wheel_distance * speed_order_constrained.vt;  // avant-gauche 

    // --- Désaturation Proportionnelle Holonome ---
    float abs_s1 = fabsf(raw_s1);
    float abs_s2 = fabsf(raw_s2);
    float abs_s3 = fabsf(raw_s3);
    float abs_s4 = fabsf(raw_s4);

    float max_wheel_speed = Max_Quatre(abs_s1, abs_s2, abs_s3, abs_s4);

    if (max_wheel_speed > MAX_PHYSICAL_WHEEL_SPEED) {
        float scale_desat = MAX_PHYSICAL_WHEEL_SPEED / max_wheel_speed;

        // 1. On applique le ratio aux roues
        Speed_Order_1 = raw_s1 * scale_desat;
        Speed_Order_2 = raw_s2 * scale_desat;
        Speed_Order_3 = raw_s3 * scale_desat;
        Speed_Order_4 = raw_s4 * scale_desat;

        // 2. On rétrograde AUSSI l'état du robot global pour la cohérence de l'accélération au cycle suivant
        speed_order_constrained.vx *= scale_desat;
        speed_order_constrained.vy *= scale_desat;
        speed_order_constrained.vt *= scale_desat;
    } else {
        Speed_Order_1 = raw_s1;
        Speed_Order_2 = raw_s2;
        Speed_Order_3 = raw_s3;
        Speed_Order_4 = raw_s4;
    }
}


void set_Constraint_vitesse_xy_max(float v_max) {
    if (v_max != 0) {
        if (v_max <= DEFAULT_CONSTRAINT_V_MAX) {
            robot_v_max = v_max;
        } else {
            robot_v_max = DEFAULT_CONSTRAINT_V_MAX;
        }
    } else {
        robot_v_max = DEFAULT_CONSTRAINT_V_MAX;
    }
}

void set_Constraint_vt_max(float vt_max) {
    if (vt_max != 0) {
        if (vt_max <= DEFAULT_CONSTRAINT_VT_MAX) {
            robot_vt_max = vt_max;
        } else {
            robot_vt_max = DEFAULT_CONSTRAINT_VT_MAX;
        }         
    } else {
        robot_vt_max = DEFAULT_CONSTRAINT_VT_MAX;
    }
}

void set_Constraint_a_xy_max(float a_max) {
    if (a_max != 0) {
        if (a_max <= DEFAULT_CONSTRAINT_A_MAX) {
            robot_a_max = a_max;
        } else {
            robot_a_max = DEFAULT_CONSTRAINT_A_MAX;
        }
    } else {
        robot_a_max = DEFAULT_CONSTRAINT_A_MAX;
    }
}

void set_Constraint_at_max(float at_max) {
    if (at_max != 0) {
        if (at_max <= DEFAULT_CONSTRAINT_AT_MAX) {
            robot_at_max = at_max;
        } else {
            robot_at_max = DEFAULT_CONSTRAINT_AT_MAX;
        }
    } else {
        robot_at_max = DEFAULT_CONSTRAINT_AT_MAX;
    }
}

