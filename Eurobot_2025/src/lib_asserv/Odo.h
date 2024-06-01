extern int tics_g;
extern int tics_d;

extern Acceleration acceleration_robot;
extern Speed speed_robot;
extern Position position_robot;


/******************************    Fonctions    *******************************/

// initialiser l'odometrie
void odo_init(void);

// assigner des valeurs aux coefs (relations tic/metre et entraxe)
void odo_set_tic_by_meter(double param_tic_by_meter_droit, double param_tic_by_meter_gauche);

// assigner une valeur a l'ecart entre les roues d'odometrie
void odo_set_spacing(float param_spacing);

// maj de la position du robot
void odo_position_step(int qei_g, int qei_d);

// maj de la vitesse/acceleration  du robot
void odo_speed_step(float period);



// connaitre l'etat du robot
Position get_position(void);
Speed get_speed(void);
Acceleration get_acceleration(void);


// assigner des valeurs a la position (x, y et theta)
void set_position(Position pos);
void set_position_x(float x);
void set_position_y(float y);
void set_position_t(float t);