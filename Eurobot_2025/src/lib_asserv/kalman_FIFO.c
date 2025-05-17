#include "../main.h"
#include "lib_asserv.h"

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

KalmanState* kalman_fifo_get_delay(KalmanFIFO* fifo, int delay_ms, float dt_ms) {
    int steps_back = (int)((float)delay_ms / dt_ms + 0.5f); // arrondi
    if (steps_back >= KALMAN_FIFO_LEN) {
        steps_back = KALMAN_FIFO_LEN - 1;
    }

    int index = fifo->head - steps_back - 1;
    if (index < 0) {
        index += KALMAN_FIFO_LEN;
    }

    return &fifo->buffer[index];
}

void kalman_fifo_repropagate(KalmanFIFO* fifo, int corrected_index, float dt) {
    // Calcul de l'index dans le buffer de l'état corrigé
    int base = (fifo->head - 1 - corrected_index + KALMAN_FIFO_LEN) % KALMAN_FIFO_LEN;

    // Rejouer les prédictions à partir de cet index
    for (int i = 1; i <= corrected_index; i++) {
        int prev_idx = (base + i - 1) % KALMAN_FIFO_LEN;
        int curr_idx = (base + i) % KALMAN_FIFO_LEN;

        // Copier l'état précédent comme base de prédiction
        fifo->buffer[curr_idx] = fifo->buffer[prev_idx];

        // Appliquer la prédiction avec la vitesse historique
        kalman_predict(&fifo->buffer[curr_idx], &fifo->speed_robot[prev_idx], dt);
    }
}