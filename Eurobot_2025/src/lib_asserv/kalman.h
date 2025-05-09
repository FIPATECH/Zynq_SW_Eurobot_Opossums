#ifndef __KALMAN_H_
#define __KALMAN_H_

#define NOISE_XY_ODO 1e-3
#define NOISE_T_ODO 1e-2

#define NOISE_XY_LIDAR 0.6
#define NOISE_T_LIDAR 0.6

void kalman_init(Position* pos);
void kalman_predict(Position* pos, float dx, float dy, float dtheta);
void kalman_update(Position* pos, const Position* pos_predict, const Position lidar_meas);

#endif // __KALMAN_H_