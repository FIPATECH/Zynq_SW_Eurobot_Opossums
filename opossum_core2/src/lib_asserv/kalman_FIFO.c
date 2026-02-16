#include "../main.h"
#include "lib_asserv.h"

KalmanFIFO kalman_fifo;

void kalman_fifo_init(KalmanFIFO* fifo) {
    memset(fifo, 0, sizeof(KalmanFIFO));

    // Initialiser la tête de la FIFO
    fifo->head = 0;
    // Initialiser le compteur d'éléments
    fifo->count = 0;

    // Initialiser la mémoire de la FIFO
    KalmanState default_state;
    kalman_init(&default_state);

    for (int i = 0; i < KALMAN_FIFO_LEN; i++) {
        fifo->buffer[i] = default_state;

        fifo->speed_robot[i].vx = 0.0f;
        fifo->speed_robot[i].vy = 0.0f;
        fifo->speed_robot[i].vt = 0.0f;

        fifo->observations[i].has_lidar = 0;
        fifo->observations[i].z_lidar[0] = 0.0f;
        fifo->observations[i].z_lidar[1] = 0.0f;
        fifo->observations[i].z_lidar[2] = 0.0f;

        fifo->observations[i].has_camera = 0;
        fifo->observations[i].z_camera[0] = 0.0f;   
        fifo->observations[i].z_camera[1] = 0.0f;
        fifo->observations[i].z_camera[2] = 0.0f;
    }
}

void kalman_fifo_push(KalmanFIFO* fifo, KalmanState* state, Speed* speed_robot) {
    // Stocke l'état à l'emplacement courant
    memcpy(&fifo->buffer[fifo->head], state, sizeof(KalmanState));

    // Stocke la vitesse du robot à l'emplacement courant
    memcpy(&fifo->speed_robot[fifo->head], speed_robot, sizeof(Speed));
    
    // Incrémente la tête de la FIFO en la ramenant dans les bornes
    fifo->head = (fifo->head + 1) % KALMAN_FIFO_LEN;

    // Incrémente le compteur d'éléments    
    if (fifo->count < KALMAN_FIFO_LEN) {
        fifo->count++;
    }
}

int kalman_fifo_get_delay(KalmanFIFO* fifo, int delay_ms, float dt_ms) {
    int samples_back = (int)(delay_ms / dt_ms); 
    if (fifo->count < samples_back) {
        // printf("WARNING: FIFO not enough samples\n");
        return -1; // Erreur : pas assez d'échantillons dans la FIFO
    }

    int index = fifo->head - samples_back - 1;
    if (index < 0) index += KALMAN_FIFO_LEN;

    return index;
}

void kalman_fifo_repropagate(KalmanFIFO* fifo, int delay_index, float dt_s, float R_lidar[3], float R_camera[3]) {
    int i = delay_index;
    int next_i;

    // On repropague en boucle circulaire jusqu'à la tête-1
    while (i != ((fifo->head - 1 + KALMAN_FIFO_LEN) % KALMAN_FIFO_LEN)) {
        next_i = (i + 1) % KALMAN_FIFO_LEN;

        // utilisation direct de l’état précédent, sans copie complète
        KalmanState* current = &fifo->buffer[i];
        KalmanState* next = &fifo->buffer[next_i];

        Speed* speed = &fifo->speed_robot[i];

         // Calcul prédictif sur l’état "next" basé sur "current"
        memcpy(next, current, sizeof(KalmanState)); // Copie une fois

        // 1. Prédiction odométrique classique
        kalman_predict(next, speed, dt_s);

        // 2. CORRECTION : Si une mesure Lidar avait eu lieu à 'next_i', on la réapplique !
        if (fifo->observations[next_i].has_lidar) {
            kalman_update(next, fifo->observations[next_i].z_lidar, R_lidar);
        }

        // 3. CORRECTION : Si une mesure Caméra avait eu lieu à 'next_i', on la réapplique !
        if (fifo->observations[next_i].has_camera) {
            kalman_update(next, fifo->observations[next_i].z_camera, R_camera);
        }

        i = next_i;
    }
    
    // Après propagation, mettre à jour l’état courant
    kalman_current_state = fifo->buffer[(fifo->head - 1 + KALMAN_FIFO_LEN) % KALMAN_FIFO_LEN];
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