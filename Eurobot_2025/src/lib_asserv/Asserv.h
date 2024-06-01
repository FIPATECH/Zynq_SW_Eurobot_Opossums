
// Position absolue du robot (x, y, et theta)
typedef struct {
    float x; // en metre
    float y; // en metre
    float t; // en radian
} Position;

// Vitesse et vitesse angulaire du robot
typedef struct {
    float v; // en m/s
    float vt; // en rad/s
    float v_roue;
} Speed;

// acceleration du robot (dv/dt,  d2theta/dt2   et   v*(dtheta/dt))
typedef struct {
    float a; // en m/s2
    float at; // en rad/s2
    float v_vt; // en rad*m/s2
} Acceleration;