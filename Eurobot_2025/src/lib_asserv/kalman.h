#ifndef __KALMAN_H_
#define __KALMAN_H_

#define NOISE_XY_ODO 1e-3
#define NOISE_T_ODO 1e-2

#define NOISE_XY_LIDAR 50//20 //0.8
#define NOISE_T_LIDAR 50//20 //0.8

void kalman_init(Position* pos);
void kalman_predict(void);
void kalman_update(Position* pos, const Position* pos_predict, const Position lidar_meas);

#endif // __KALMAN_H_