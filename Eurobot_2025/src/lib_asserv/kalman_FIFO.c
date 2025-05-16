#include "../main.h"
#include "lib_asserv.h"

void kalman_fifo_init(KalmanFIFO* fifo) {
    memset(fifo, 0, sizeof(KalmanFIFO));
    fifo->head = 0;
}

void kalman_fifo_push(KalmanFIFO* fifo, KalmanState* state) {
    // Stocke l'état à l'emplacement courant
    memcpy(&fifo->buffer[fifo->head], state, sizeof(KalmanState));
    
    // Incrémente la tête de la FIFO en la ramenant dans les bornes
    fifo->head = (fifo->head + 1) % KALMAN_FIFO_LEN;
}

KalmanState* kalman_fifo_get_delay(KalmanFIFO* fifo, int delay_ms, float dt_ms) {
    int steps_back = (int)((float)delay_ms / dt_ms + 0.5f); // arrondi
    if (steps_back >= KALMAN_FIFO_LEN) {
        steps_back = KALMAN_FIFO_LEN - 1;  // Clamp
    }

    int index = fifo->head - steps_back - 1;
    if (index < 0) {
        index += KALMAN_FIFO_LEN;
    }

    return &fifo->buffer[index];
}

void kalman_fifo_repropagate(KalmanFIFO* fifo, int corrected_idx, float dt) {
    int i = (corrected_idx + 1) % KALMAN_FIFO_LEN;

    // Tant qu'on n'a pas atteint l'état le plus récent (head)
    while (i != fifo->head) {
        // Copier l'état précédent corrigé dans l'état courant avant prédiction
        memcpy(&fifo->buffer[i], &fifo->buffer[(i - 1 + KALMAN_FIFO_LEN) % KALMAN_FIFO_LEN], sizeof(KalmanState));

        // Prédire l'état courant à partir de l'état précédent corrigé
        kalman_predict(&fifo->buffer[i], &fifo->speed_robot[i], dt);

        i = (i + 1) % KALMAN_FIFO_LEN;
    }
}