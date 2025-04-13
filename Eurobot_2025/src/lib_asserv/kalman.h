#ifndef __KALMAN_H_
#define __KALMAN_H_

#define NOISE_XY_ODO 1e-2
#define NOISE_T_ODO 1e-2

#define NOISE_XY_LIDAR 1e-2
#define NOISE_T_LIDAR 1e-2

typedef struct {
    float x;
    float y;
    float theta;
} Vector3;

void kalman_init(Vector3* x);
void kalman_predict(Vector3* x, float dx, float dy, float dtheta);
void kalman_update(Vector3* x, float z_x, float z_y, float z_theta);

#endif // __KALMAN_H_