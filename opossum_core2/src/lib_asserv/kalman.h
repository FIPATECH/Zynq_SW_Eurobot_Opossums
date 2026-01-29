#ifndef __KALMAN_H_
#define __KALMAN_H_

#define STATE_SIZE 6  // x, y, theta
#define HISTORY_LEN 200  // pour 200 ms à 1 kHz

#define LIDAR_DELAY 90 // 100 ms

#define PROCESS_NOISE_ODOM_X 0.02f
#define PROCESS_NOISE_ODOM_Y 0.02f
#define PROCESS_NOISE_ODOM_THETA 0.03f

#define PROCESS_NOISE_ODOM_VX 0.02f
#define PROCESS_NOISE_ODOM_VY 0.02f
#define PROCESS_NOISE_ODOM_VTHETA 0.05f

#define PROCESS_NOISE_LIDAR_X 1.3f
#define PROCESS_NOISE_LIDAR_Y 1.3f
#define PROCESS_NOISE_LIDAR_THETA 1.5f

#define S_INV_EPS 1e-6f


typedef struct {
    float x[STATE_SIZE] __attribute__((aligned(16)));                 // état X[x, y, theta, vx, vy, vtheta]
    float P[STATE_SIZE][STATE_SIZE] __attribute__((aligned(16)));     // covariance
} KalmanState;

extern KalmanState kalman_current_state;

/**
 * Initialise l’état du filtre de Kalman.
 * 
 * @param state L’état à initialiser.
 */
void kalman_init(KalmanState* state);

/**
 * Applique la prédiction du modèle EKF à partir des vitesses dans le repère robot.
 * 
 * @param state L’état courant à prédire.
 * @param speed Vitesse du robot dans le repère robot.
 * @param dt Pas de temps (s).
 */
void kalman_predict(KalmanState* state, Speed* speed, float dt);

/**
 * Applique la correction EKF à partir d’une mesure z = [x, y, theta] dans le repère monde.
 * 
 * @param state L’état à corriger (potentiellement un état passé issu du FIFO).
 * @param z Mesure du LiDAR : position et angle absolus.
 * 
 * @note plusieurs returns possibles :
 *      - 1 : erreur de mesure (NaN)
 *      - 2 : erreur d’état (NaN)
 *      - 3 : matrice S singulière (non inversible)
 *      - 4 : Clamp de sécurité post update
 *      - 5 : Clamp de sécurité post update
 */
void kalman_update(KalmanState* state, float z[STATE_SIZE]);

#endif // __KALMAN_H_