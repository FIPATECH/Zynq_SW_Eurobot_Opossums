#ifndef __KALMAN_FIFO_H_
#define __KALMAN_FIFO_H_

#include "kalman.h"
#include "Asserv_type.h"

#define KALMAN_FIFO_LEN 200

typedef struct {
    KalmanState buffer[KALMAN_FIFO_LEN];
    Speed speed_robot[KALMAN_FIFO_LEN];
    int head;
} KalmanFIFO;

extern KalmanFIFO kalman_fifo;

/**
 * Initialise la FIFO (position tête à 0, mémoire à zéro).
 */
void kalman_fifo_init(KalmanFIFO* fifo);

/**
 * Ajoute un état à la FIFO.
 * 
 * @param fifo La structure FIFO.
 * @param state L'état à ajouter.
 */
void kalman_fifo_push(KalmanFIFO* fifo, KalmanState* state, Speed* speed_robot);

/**
 * Récupère un état dans le passé à un délai donné (en ms).
 * 
 * @param fifo La structure FIFO.
 * @param delay_ms Le délai souhaité (ex : 100 ms).
 * @param dt_ms Période de l’odométrie en ms (ex : 1.0f).
 * 
 * @return L'état à l'index correspondant au délai, ou NULL si le délai est trop long.
 */
int kalman_fifo_get_delay(KalmanFIFO* fifo, int delay_ms, float dt_ms);


/**
 * Repropagation des états dans la FIFO à partir d’un état corrigé.
 *
 * @param fifo La structure FIFO.
 * @param delay_index L’index de l’état corrigé dans la FIFO.
 * @param dt_s Le pas de temps (s).
 */
void kalman_fifo_repropagate(KalmanFIFO* fifo, int delay_index, float dt_s);

/**
 * @brief Initialise la kalman avec les valeurs du lidar
 * 
 * @param fifo fifo de kalman
 * @param pos lidar_pos
 */
void kalman_init_with_lidar(KalmanFIFO* fifo, Position* lidar_pos);

#endif // __KALMAN_FIFO_H_