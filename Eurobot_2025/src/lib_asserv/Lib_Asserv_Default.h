/*############################################################################*/
/*                                    Odo                                     */
/*############################################################################*/

// {tic/m, m/tic, entre roues}
#define DEFAULT_ODO_TIC_BY_M_D 47269
#define DEFAULT_ODO_TIC_BY_M_G 47269

//#define DEFAULT_ODO_SPACING 0.23998
//#define DEFAULT_ODO_SPACING 0.245// gret7
#define DEFAULT_ODO_SPACING 0.242// han7

/*############################################################################*/
/*                                  Motion                                    */
/*############################################################################*/

// {v, vt} v = 0.9 * v max moteur, vt = v/(entre roues/2)
#define DEFAULT_CONSTRAINT_V_MAX  0.5
#define DEFAULT_CONSTRAINT_VT_MAX 2.0
#define DEFAULT_CONSTRAINT_V_ROUE 0.7

/* {a, at, v_vt} a = a max sans glissement, at = a/(entre roues/2),
 * v_vt = acc centripete (trop fort -> erreur odo) 0.1g
 *
 */
#define DEFAULT_CONSTRAINT_A_MAX 0.8 //1
#define DEFAULT_CONSTRAINT_AT_MAX 2.0
#define DEFAULT_CONSTRAINT_VVT_MAX 0.981


#define ASSERV_BLOCK_TIME_LIMIT 1   // 1s "blocké" avant de tout couper

#define DEFAULT_RAYON_COURBURE 0.2


/*############################################################################*/
/*                                  Asserv                                    */
/*############################################################################*/

#define DEFAULT_STOP_DISTANCE 0.005 // +-5mm
#define DEFAULT_STOP_ANGLE 0.01745// +-1deg  // en radian




/*############################################################################*/
/*                                   PID                                      */
/*############################################################################*/

// 2 PID lies a l'asserv en position (dist: position absolue, angle: position angulaire)
#define DEFAULT_PID_DIST_KP 2 // kp   
#define DEFAULT_PID_DIST_KI 0 //ki    
#define DEFAULT_PID_DIST_KD 0  //kd     

#define DEFAULT_PID_ANGLE_KP 2.1 // kp    
#define DEFAULT_PID_ANGLE_KI 0 //ki
#define DEFAULT_PID_ANGLE_KD 0 //kd    


// 2 PID lies a l'asserve en vitesse (vitesse lineaire et vitesse angulaire)
#define DEFAULT_PID_V_LIN_KP 120 // kp      
#define DEFAULT_PID_V_LIN_KI 10   //ki       
#define DEFAULT_PID_V_LIN_KD 10   //kd

#define DEFAULT_PID_V_ANGLE_KP 23//25 // kp      
#define DEFAULT_PID_V_ANGLE_KI 5 //ki       
#define DEFAULT_PID_V_ANGLE_KD 0 //kd 