#include "../main.h"
#include "lib_asserv.h"

KalmanFIFO kalman_fifo;

void kalman_fifo_init(KalmanFIFO* fifo) {
    memset(fifo, 0, sizeof(KalmanFIFO));
    fifo->head = 0;
}

void kalman_fifo_push(KalmanFIFO* fifo, KalmanState* state, Speed* speed_robot) {
    // Stocke l'état à l'emplacement courant
    memcpy(&fifo->buffer[fifo->head], state, sizeof(KalmanState));

    // Stocke la vitesse du robot à l'emplacement courant
    memcpy(&fifo->speed_robot[fifo->head], speed_robot, sizeof(Speed));
    
    // Incrémente la tête de la FIFO en la ramenant dans les bornes
    fifo->head = (fifo->head + 1) % KALMAN_FIFO_LEN;
}

int kalman_fifo_get_delay(KalmanFIFO* fifo, int delay_ms, float dt_ms) {
    int index = fifo->head - 100 - 1;
    if (index < 0) index += KALMAN_FIFO_LEN;
    return index;
}

void kalman_fifo_repropagate(KalmanFIFO* fifo, int delay_index, float dt_s) {
    int i = delay_index;
    int next_i;

    // On repropague en boucle circulaire jusqu'à la tête-1
    while (i != ((fifo->head - 1 + KALMAN_FIFO_LEN) % KALMAN_FIFO_LEN)) {
        next_i = (i + 1) % KALMAN_FIFO_LEN;

        // On prédit l'état suivant à partir de l'état courant i et de la vitesse à i
        kalman_predict(&fifo->buffer[next_i], &fifo->buffer[i], &fifo->speed_robot[i], dt_s);

        i = next_i;
    }
}

void kalman_init_with_lidar(KalmanFIFO* fifo, Position* lidar_pos) {
    KalmanState init_state;

    // Initialiser la position
    init_state.x[0] = lidar_pos->x;
    init_state.x[1] = lidar_pos->y;
    init_state.x[2] = principal_angle(lidar_pos->t);

    // Initialiser les vitesses à 0 (ou valeurs par défaut)
    init_state.x[3] = 0.0f; // vx
    init_state.x[4] = 0.0f; // vy
    init_state.x[5] = 0.0f; // vtheta

    kalman_current_state = init_state;

    // Initialiser la matrice de covariance P (confiance initiale)
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            init_state.P[i][j] = 0.0f;
        }
        init_state.P[i][i] = 0.01f;  // petite incertitude initiale
    }

    // Initialiser la FIFO : on remplit tout avec cet état initial
    for (int i = 0; i < KALMAN_FIFO_LEN; i++) {
        fifo->buffer[i] = init_state;
        fifo->speed_robot[i].vx = 0.0f;
        fifo->speed_robot[i].vy = 0.0f;
        fifo->speed_robot[i].vt = 0.0f;
    }
    fifo->head = 0;
}