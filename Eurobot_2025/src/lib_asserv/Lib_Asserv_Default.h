#ifndef _LIB_ASSERV_DEFAULT_H_
#define _LIB_ASSERV_DEFAULT_H_

/*############################################################################*/
/*                                    Odo                                     */
/*############################################################################*/

// {tic/m, m/tic, entre roues}
#define DEFAULT_ODO_SPACING  0.115//0.115
#define DEFAULT_SIZE_WHEEL 0.060 // 6cm
#define DEFAULT_WHEEL_RADIUS DEFAULT_SIZE_WHEEL/2 // 3cm

/*############################################################################*/
/*                                  Motion                                    */
/*############################################################################*/

// {v, vt} v = 0.9 * v max moteur, vt = v/(entre roues/2)
#define DEFAULT_CONSTRAINT_V_ROUE_MAX 0.7//1.5 //0.7
#define DEFAULT_CONSTRAINT_V_MAX  DEFAULT_CONSTRAINT_V_ROUE_MAX
#define DEFAULT_CONSTRAINT_VT_MAX 1.5//1.0

#define DEFAULT_CONSTRAINT_A_ROUE 0.5
#define DEFAULT_CONSTRAINT_A_MAX DEFAULT_CONSTRAINT_A_ROUE
#define DEFAULT_CONSTRAINT_AT_MAX 1//1.5 // 1 rad/s2

#define ASSERV_BLOCK_TIME_LIMIT 1   // 1s "blocké" avant de tout couper

/*############################################################################*/
/*                                  Asserv                                    */
/*############################################################################*/

#define DEFAULT_STOP_DISTANCE 0.005 // +-5mm
#define DEFAULT_STOP_ANGLE 0.01745// +-1deg  // en radian

/*############################################################################*/
/*                                   PID                                      */
/*############################################################################*/

#define DEFAULT_PID_DIST_KP 1//1.8 // kp   
#define DEFAULT_PID_DIST_KI 0 //ki    
#define DEFAULT_PID_DIST_KD 0//0.2  //kd     

#define DEFAULT_PID_ANGLE_KP 0.8//2.5   //2.3 // kp
#define DEFAULT_PID_ANGLE_KI 0 //ki
#define DEFAULT_PID_ANGLE_KD 0//1.5 //kd 

// 2 PID lies a l'asserve en vitesse (vitesse lineaire et vitesse angulaire)
#define DEFAULT_PID_V_LIN_KP 10000 // kp
#define DEFAULT_PID_V_LIN_KI 100   //ki
#define DEFAULT_PID_V_LIN_KD 0   //kd   

#endif // _LIB_ASSERV_DEFAULT_H_

