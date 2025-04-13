#ifndef __KALMAN_H_
#define __KALMAN_H_

#define NOISE_XY_ODO 1e-2
#define NOISE_T_ODO 1e-2

#define NOISE_XY_LIDAR 1e-2
#define NOISE_T_LIDAR 1e-2

void kalman_init(Position* pos);
void kalman_predict(Position* pos, float dx, float dy, float dtheta);
void kalman_update(Position* pos, float z_x, float z_y, float z_theta);

#endif // __KALMAN_H_