
#include "lib_asserv.h"


Acceleration Accel_Max;
Speed Speed_Max;

Speed speed_order;
Speed speed_order_constrained;

float Accel_Max_Roue;
float Speed_Max_Roue;

float Speed_Order_1, Speed_Order_2, Speed_Order_3;

float tau_x, tau_y, tau_t = 0;

float old_vx1, old_vx2, old_vx3 = 0;
float old_vy1, old_vy2, old_vy3 = 0;
float old_vt1, old_vt2, old_vt3 = 0;

float vx_o, vy_o, vt_o = 0;

float old_vx_o, old_vy_o, old_vt_o = 0;

float v_o, old_v_o = 0;


void speed_constrainer_init(void)
{
	Accel_Max.ax    = DEFAULT_CONSTRAINT_A_MAX;
	Accel_Max.ay    = DEFAULT_CONSTRAINT_A_MAX;
	Accel_Max.at    = DEFAULT_CONSTRAINT_AT_MAX;
    Accel_Max_Roue  = DEFAULT_CONSTRAINT_A_ROUE;

	Speed_Max.vx   = DEFAULT_CONSTRAINT_V_MAX;
	Speed_Max.vy   = DEFAULT_CONSTRAINT_V_MAX;
	Speed_Max.vt   = DEFAULT_CONSTRAINT_VT_MAX;
    Speed_Max_Roue = DEFAULT_CONSTRAINT_V_ROUE_MAX;
}


void constrain_speed_order(float period) {
    float vx_o = speed_order.vx;
    float vy_o = speed_order.vy;
    float vt_o = speed_order.vt;

    vt_o = limit_float(vt_o, -Speed_Max.vt, Speed_Max.vt);
    float vt_component = -(vt_o * robot_wheel_distance);
	// vt_component  = limit_float(vt_component, -Speed_Max_Roue, Speed_Max_Roue);

    float wheel_speed_1 = vt_component + vx_o;  
    float wheel_speed_2 = vt_component - (vx_o * 0.5f) + (vy_o * (sqrtf(3.0f) / 2.0f)); 
    float wheel_speed_3 = vt_component - (vx_o * 0.5f) - (vy_o * (sqrtf(3.0f) / 2.0f)); 

    //limitation on wheel speed and wheel acceleration
    float speed_coef = maximum3(fabsf(wheel_speed_1), fabsf(wheel_speed_2), fabsf(wheel_speed_3));
    if (speed_coef > Speed_Max_Roue) {
        wheel_speed_1 = wheel_speed_1 * Speed_Max_Roue / speed_coef;
        wheel_speed_2 = wheel_speed_2 * Speed_Max_Roue / speed_coef;
        wheel_speed_3 = wheel_speed_3 * Speed_Max_Roue / speed_coef;
    }

    // --- Limitation d’accélération proportionnelle
    float delta_1 = wheel_speed_1 - Speed_Order_1;
    float delta_2 = wheel_speed_2 - Speed_Order_2;
    float delta_3 = wheel_speed_3 - Speed_Order_3;

    float max_delta = maximum3(fabsf(delta_1), fabsf(delta_2), fabsf(delta_3));
    float delta_vr_max = Accel_Max_Roue * period;

    if (max_delta > delta_vr_max) {
        float scale = delta_vr_max / max_delta;
        delta_1 *= scale;
        delta_2 *= scale;
        delta_3 *= scale;
    }

    Speed_Order_1 += delta_1;
    Speed_Order_2 += delta_2;
    Speed_Order_3 += delta_3;  
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


