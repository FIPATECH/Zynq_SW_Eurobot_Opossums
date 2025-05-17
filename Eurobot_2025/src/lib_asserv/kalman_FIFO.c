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
    int steps_back = (int)((float)delay_ms / dt_ms + 0.5f); // arrondi
    if (steps_back >= KALMAN_FIFO_LEN) return -1;

    int index = fifo->head - steps_back - 1;
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
        kalman_predict(&fifo->buffer[next_i], &fifo->speed_robot[i], dt_s);

        i = next_i;
    }
}