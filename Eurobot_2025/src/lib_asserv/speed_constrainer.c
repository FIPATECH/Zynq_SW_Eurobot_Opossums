
#include "lib_asserv.h"


Acceleration Accel_Max;
Speed Speed_Max;

Speed speed_order;
Speed speed_order_constrained;
Speed speed_order_constrained_1;

float Accel_Max_Roue;
float Speed_Max_Roue;

float Speed_Order_1, Speed_Order_2, Speed_Order_3;

float vx_o, vy_o, vt_o = 0;

float robot_v_max;
float robot_vt_max;

float robot_a_max;
float robot_at_max;

void speed_constrainer_init(void)
{
	Speed_Max.vx   = DEFAULT_CONSTRAINT_V_MAX;
	Speed_Max.vy   = DEFAULT_CONSTRAINT_V_MAX;
	Speed_Max.vt   = DEFAULT_CONSTRAINT_VT_MAX;
    Speed_Max_Roue = DEFAULT_CONSTRAINT_V_ROUE_MAX;

    robot_v_max   = DEFAULT_CONSTRAINT_V_MAX;
    robot_vt_max  = DEFAULT_CONSTRAINT_VT_MAX;
}

void acceleration_constrainer_init(void)
{
    Accel_Max.ax    = DEFAULT_CONSTRAINT_A_MAX;
    Accel_Max.ay    = DEFAULT_CONSTRAINT_A_MAX;
    Accel_Max.at    = DEFAULT_CONSTRAINT_AT_MAX;
    Accel_Max_Roue  = DEFAULT_CONSTRAINT_A_ROUE;

    robot_a_max = DEFAULT_CONSTRAINT_A_MAX;
    robot_at_max = DEFAULT_CONSTRAINT_AT_MAX;
}

void constrain_speed_order(void) {
    speed_order_constrained_1.vx = speed_order.vx;
    speed_order_constrained_1.vy = speed_order.vy;
    speed_order_constrained_1.vt = speed_order.vt;

    float v_linear = sqrtf(speed_order_constrained_1.vx * speed_order_constrained_1.vx + speed_order_constrained_1.vy * speed_order_constrained_1.vy);

    if(v_linear > Speed_Max.vx) {
        float scale = Speed_Max.vx / v_linear;
        speed_order_constrained_1.vx *= scale;
        speed_order_constrained_1.vy *= scale;
    }
    speed_order_constrained_1.vt = limit_float(speed_order_constrained_1.vt, -Speed_Max.vt, Speed_Max.vt);
}

void constrain_acceleration_order(float period) {
    float delta_v_max   = robot_a_max * period;
    float delta_vt_max  = robot_at_max * period;


    // process old speed constrained (be aware of the rotation)
    float delta_angle = speed_order_constrained.vt * period;
    float cos_angle = cosf(delta_angle);
    float sin_angle = sinf(delta_angle);
    float previous_vx_order = speed_order_constrained.vx * cos_angle + speed_order_constrained.vy * sin_angle;
    float previous_vy_order = speed_order_constrained.vx * sin_angle - speed_order_constrained.vy * cos_angle;

    // process acceleration steps
    float delta_vx = speed_order_constrained_1.vx - previous_vx_order;
    float delta_vy = speed_order_constrained_1.vy - previous_vy_order;

    float dv_linear = sqrtf(delta_vx * delta_vx + delta_vy * delta_vy);
    if (dv_linear > delta_v_max) {
        float scale = delta_v_max / dv_linear;
        speed_order_constrained.vx = previous_vx_order + delta_vx * scale;
        speed_order_constrained.vy = previous_vy_order + delta_vy * scale;
    } else {
        speed_order_constrained.vx = speed_order_constrained_1.vx;
        speed_order_constrained.vy = speed_order_constrained_1.vy;
    }
    speed_order_constrained.vt = limit_float(speed_order_constrained_1.vt, speed_order_constrained.vt - delta_vt_max, speed_order_constrained.vt + delta_vt_max);

    // process wheel speed
    float vt_component = -(vt_o * robot_wheel_distance);
    Speed_Order_1 = vt_component + vx_o;
    Speed_Order_2 = vt_component - (vx_o * 0.5f) + (vy_o * (sqrtf(3.0f) / 2.0f)); 
    Speed_Order_3 = vt_component - (vx_o * 0.5f) - (vy_o * (sqrtf(3.0f) / 2.0f)); 
}


void set_Constraint_vitesse_xy_max(float v_max) {
    if (v_max != 0) {
        if (v_max <= DEFAULT_CONSTRAINT_V_MAX) {
            Speed_Max.vx = v_max;
            Speed_Max.vy = v_max;
        } else {
            Speed_Max.vx = DEFAULT_CONSTRAINT_V_MAX;
            Speed_Max.vy = DEFAULT_CONSTRAINT_V_MAX;
        }
    } else {
        Speed_Max.vx = DEFAULT_CONSTRAINT_V_MAX;
        Speed_Max.vy = DEFAULT_CONSTRAINT_V_MAX;
    }
}

void set_Constraint_vt_max(float vt_max) {
    if (vt_max != 0) {
        if (vt_max <= DEFAULT_CONSTRAINT_VT_MAX) {
            Speed_Max.vt = vt_max;
        } else {
            Speed_Max.vt = DEFAULT_CONSTRAINT_VT_MAX;
        }         
    } else {
        Speed_Max.vt = DEFAULT_CONSTRAINT_VT_MAX;
    }
}

void set_Constraint_acceleration_xy_max(float a_max, float at_max) {
    if (a_max != 0) {
        Accel_Max.ax = a_max;
        Accel_Max.ay = a_max;
    } else {
        Accel_Max.ax = DEFAULT_CONSTRAINT_A_MAX;
        Accel_Max.ay = DEFAULT_CONSTRAINT_A_MAX;
    }
    if (at_max != 0) {
        Accel_Max.at = at_max;
    } else {
        Accel_Max.at = DEFAULT_CONSTRAINT_AT_MAX;
    }
}


