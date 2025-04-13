#include "lib_asserv.h"

float P[3][3]; // Covariance matrix

float Q[3][3] = {
    {NOISE_XY_ODO, 0, 0},
    {0, NOISE_XY_ODO, 0},
    {0, 0, NOISE_T_ODO}
}; // odometry noise covariance matrix

float R[3][3] = {
    {NOISE_XY_LIDAR, 0, 0},
    {0, NOISE_XY_LIDAR, 0},
    {0, 0, NOISE_T_LIDAR}
}; // lidar noise covariance matrix

void kalman_init(Vector3* x) {
    x->x = 0;
    x->y = 0;
    x->theta = 0;

    memset(P, 0, sizeof(P));
    P[0][0] = P[1][1] = 0.1f;
    P[2][2] = 0.01f;
}

void kalman_predict(Vector3* x, float dx, float dy, float dtheta) { // called each odo step : 1kHz
    x->x += dx;
    x->y += dy;
    x->theta += dtheta;

    // Mise à jour de la covariance : P = P + Q
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            P[i][j] += Q[i][j];
}

void kalman_update(Vector3* x, float z_x, float z_y, float z_theta) { // called each lidar step : 10Hz
    float z[3] = { z_x, z_y, z_theta };
    float y[3] = {
        z[0] - x->x,
        z[1] - x->y,
        principal_angle(z[2] - x->theta)
    };

    // Calcul S = P + R
    float S[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            S[i][j] = P[i][j] + R[i][j];

    // Calcul de l’inverse de S (comme S est diagonale ici, c'est simple)
    float S_inv[3][3] = {
        {1.0f / S[0][0], 0, 0},
        {0, 1.0f / S[1][1], 0},
        {0, 0, 1.0f / S[2][2]}
    };

    // Gain de Kalman K = P * S⁻¹
    float K[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            K[i][j] = P[i][j] * S_inv[j][j];  // diag simplifiée

    // Mise à jour état : x = x + K*y
    x->x += K[0][0]*y[0] + K[0][1]*y[1] + K[0][2]*y[2];
    x->y += K[1][0]*y[0] + K[1][1]*y[1] + K[1][2]*y[2];
    x->theta += K[2][0]*y[0] + K[2][1]*y[1] + K[2][2]*y[2];
    x->theta = principal_angle(x->theta);

    // Mise à jour covariance : P = (I - K) * P
    float I_K[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            I_K[i][j] = (i == j ? 1.0f : 0.0f) - K[i][j];

    float new_P[3][3] = {0};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                new_P[i][j] += I_K[i][k] * P[k][j];

    memcpy(P, new_P, sizeof(P));
}