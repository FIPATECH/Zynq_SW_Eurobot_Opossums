#include "../main.h"
#include "lib_asserv.h"

Position position_robot_odom; // Position estimée par le filtre de Kalman

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

void kalman_init(Position* pos) {
    pos->x = 0;
    pos->y = 0;
    pos->t = 0;

    memset(P, 0, sizeof(P));
    P[0][0] = P[1][1] = 0.1f;
    P[2][2] = 0.01f;
}

void kalman_predict(Position* pos, float dx, float dy, float dtheta) { // called each odo step : 1kHz
    pos->x += dx;
    pos->y += dy;
    pos->t += dtheta;

    // Mise à jour de la covariance : P = P + Q
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            P[i][j] += Q[i][j];
}

void kalman_update(Position* pos, const Position* pos_predict, const Position lidar_meas) {
    // Innovation : différence entre la mesure et la prédiction
    float y[3];
    y[0] = lidar_meas.x - pos_predict->x;
    y[1] = lidar_meas.y - pos_predict->y;

    // gestion d'angle modulo 2pi :
    float dt = lidar_meas.t - pos_predict->t;
    while (dt > PI)  dt -= 2.0f * PI;
    while (dt < -PI) dt += 2.0f * PI;
    y[2] = dt;

    // Innovation covariance : S = P + R
    float S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] = P[i][j] + R[i][j]; // H = I donc HPHᵀ = P

    // Calcul inverse de S (matrice 3x3 symétrique → on suppose R diagonale pour simplifier)
    float S_inv[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        if (S[i][i] != 0.0f)
            S_inv[i][i] = 1.0f / S[i][i];  // Inversion diagonale uniquement
    }

    // Gain de Kalman : K = P * S⁻¹
    float K[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            K[i][j] = P[i][j] * S_inv[j][j];  // simplifié car S_inv est diagonale

    // Mise à jour de l'état : x = x_pred + K * y
    pos->x = pos_predict->x + K[0][0] * y[0] + K[0][1] * y[1] + K[0][2] * y[2];
    pos->y = pos_predict->y + K[1][0] * y[0] + K[1][1] * y[1] + K[1][2] * y[2];
    pos->t = pos_predict->t + K[2][0] * y[0] + K[2][1] * y[1] + K[2][2] * y[2];

    // Normalise l'angle
    while (pos->t > PI)  pos->t -= 2.0f * PI;
    while (pos->t < -PI) pos->t += 2.0f * PI;

    // Mise à jour de la covariance : P = (I - K) * P
    float I_minus_K[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            I_minus_K[i][j] = (i == j ? 1.0f : 0.0f) - K[i][j];

    float P_new[3][3] = {0};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                P_new[i][j] += I_minus_K[i][k] * P[k][j];

    // Copie dans P
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            P[i][j] = P_new[i][j];
}