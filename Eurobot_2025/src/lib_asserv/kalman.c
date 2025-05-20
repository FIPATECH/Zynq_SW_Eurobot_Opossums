#include "../main.h"
#include "lib_asserv.h"


KalmanState kalman_current_state;

// Bruit de processus Q (tunable) pour l'odométrie
float Q[STATE_SIZE][STATE_SIZE] = {
    {PROCESS_NOISE_ODOM_X * PROCESS_NOISE_ODOM_X, 0, 0, 0, 0, 0},
    {0, PROCESS_NOISE_ODOM_Y * PROCESS_NOISE_ODOM_X, 0, 0, 0, 0},
    {0, 0, PROCESS_NOISE_ODOM_THETA * PROCESS_NOISE_ODOM_THETA, 0, 0, 0},
    {0, 0, 0, PROCESS_NOISE_ODOM_VX, 0, 0},
    {0, 0, 0, 0, PROCESS_NOISE_ODOM_VY, 0},
    {0, 0, 0, 0, 0, PROCESS_NOISE_ODOM_VTHETA}
};

float R_diag[3] = {PROCESS_NOISE_LIDAR_X, PROCESS_NOISE_LIDAR_Y, PROCESS_NOISE_LIDAR_THETA}; // Bruit de mesure lidar


void kalman_init(KalmanState* state) {
    memset(state->x, 0, sizeof(state->x));
    memset(state->P, 0, sizeof(state->P));
    for (int i = 0; i < STATE_SIZE; i++)
        state->P[i][i] = 0.01f;  // initial uncertainty
}

void kalman_predict(KalmanState* state, Speed* speed, float dt) {
    float theta = state->x[2];

    // Transformation des vitesses robot → monde
    float v_dx = speed->vx * cosf(theta) - speed->vy * sinf(theta);
    float v_dy = speed->vx * sinf(theta) + speed->vy * cosf(theta);
    float v_ang = speed->vt;

    // Mise à jour de la position estimée
    state->x[0] += v_dx * dt;
    state->x[1] += v_dy * dt;
    state->x[2] = principal_angle(state->x[2] + v_ang * dt);

    // Mise à jour des vitesses (optionnel, utile pour debug ou asserv)
    state->x[3] = v_dx;  // vitesse monde en x
    state->x[4] = v_dy;  // vitesse monde en y
    state->x[5] = v_ang; // vitesse angulaire

    // Jacobienne de la fonction de transition
    float v_lin = sqrtf(v_dx * v_dx + v_dy * v_dy); // vitesse linéaire
    float F[STATE_SIZE][STATE_SIZE] = {
        {1, 0, -v_lin * sinf(theta) * dt, 0, 0, 0},
        {0, 1,  v_lin * cosf(theta) * dt, 0, 0, 0},
        {0, 0, 1,                          0, 0, 0},
        {0, 0, 0,                          1, 0, 0},
        {0, 0, 0,                          0, 1, 0},
        {0, 0, 0,                          0, 0, 1}
    };

    // Mise à jour de la covariance : P = F * P * F^T + Q
    float P_temp[STATE_SIZE][STATE_SIZE] = {0};
    float P_new[STATE_SIZE][STATE_SIZE] = {0};

    for (int i = 0; i < STATE_SIZE; i++)
        for (int j = 0; j < STATE_SIZE; j++)
            for (int k = 0; k < STATE_SIZE; k++)
                P_temp[i][j] += F[i][k] * state->P[k][j];

    for (int i = 0; i < STATE_SIZE; i++)
        for (int j = 0; j < STATE_SIZE; j++)
            for (int k = 0; k < STATE_SIZE; k++)
                P_new[i][j] += P_temp[i][k] * F[j][k];

    for (int i = 0; i < STATE_SIZE; i++)
        for (int j = 0; j < STATE_SIZE; j++)
            state->P[i][j] = P_new[i][j] + Q[i][j];
}

void kalman_update(KalmanState* state, float z[STATE_SIZE]) {
    // Vérification de l'entrée
    for (int i = 0; i < 3; i++) {
        if (isnan(z[i]) ) {
            printf("KALMANERROR 1\n"); // Invalid Lidar measurement
            return;
        }else if (isnan(state->x[i])) {
            printf("KALMANERROR 2\n"); // Invalid Kalman state
            return;
        }
    }

    // Innovation y = z - h(x)
    float y[3];
    y[0] = z[0] - state->x[0];  // x
    y[1] = z[1] - state->x[1];  // y
    y[2] = principal_angle(z[2] - state->x[2]);  // theta

    // S = H * P * H^T + R
    // Ici H est 3x6 : on extrait juste les 3 premières lignes de l'identité
    float S[3][3] = {0};
    // const float epsilon = 1e-6f;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S[i][j] = state->P[i][j];
            if (i == j) S[i][j] += R_diag[i];
        }
        // S[i][i] += epsilon;  // éviter la singularité et permet de garder la matrice S inversible
    }

    // Inverse de S (3x3)
    float det = 
        S[0][0] * (S[1][1]*S[2][2] - S[1][2]*S[2][1]) -
        S[0][1] * (S[1][0]*S[2][2] - S[1][2]*S[2][0]) +
        S[0][2] * (S[1][0]*S[2][1] - S[1][1]*S[2][0]);

    if (fabs(det) < 1e-8f || isnan(det) || isinf(det)) {
        printf("KALMANERROR 3\n"); // Singular matrix
        return;
    }

    float inv_det = 1.0f / det;
    float S_inv[3][3];

    S_inv[0][0] =  (S[1][1]*S[2][2] - S[1][2]*S[2][1]) * inv_det;
    S_inv[0][1] = -(S[0][1]*S[2][2] - S[0][2]*S[2][1]) * inv_det;
    S_inv[0][2] =  (S[0][1]*S[1][2] - S[0][2]*S[1][1]) * inv_det;

    S_inv[1][0] = -(S[1][0]*S[2][2] - S[1][2]*S[2][0]) * inv_det;
    S_inv[1][1] =  (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * inv_det;
    S_inv[1][2] = -(S[0][0]*S[1][2] - S[0][2]*S[1][0]) * inv_det;

    S_inv[2][0] =  (S[1][0]*S[2][1] - S[1][1]*S[2][0]) * inv_det;
    S_inv[2][1] = -(S[0][0]*S[2][1] - S[0][1]*S[2][0]) * inv_det;
    S_inv[2][2] =  (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * inv_det;

    // K = P * H^T * S^-1 ; H^T est une matrice 6x3, ici on prend les 3 premières colonnes de l'identité
    float K[6][3] = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                K[i][j] += state->P[i][k] * S_inv[k][j];
            }
        }
    }

    // Mise à jour de l'état x = x + K * y
    for (int i = 0; i < 6; i++) {
        float delta = 0.0f;
        for (int j = 0; j < 3; j++) {
            delta += K[i][j] * y[j];
        }
        if (i == 2){  // angle
            state->x[i] = principal_angle(state->x[i] + delta);
        }else{
            state->x[i] += delta;
        }
        if (isnan(state->x[i]) || isinf(state->x[i]) || fabsf(state->x[i]) > 1e6f) {
            printf("KALMANERROR 4\n"); // Invalid state after update
            return;
        }
    }

    // Mise à jour de la covariance : P = (I - K * H) * P
    float KH[6][6] = {0};  // K * H est 6x6 ici
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            if (j < 3) {
                for (int k = 0; k < 3; k++) {
                    KH[i][j] += K[i][k] * (j == k ? 1.0f : 0.0f);
                }
            }
        }
    }

    float I_KH[6][6];
    for (int i = 0; i < 6; i++){
        for (int j = 0; j < 6; j++){
            I_KH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
        }
    }

    float P_new[6][6] = {0};
    for (int i = 0; i < 6; i++){
        for (int j = 0; j < 6; j++){
            for (int k = 0; k < 6; k++){
                P_new[i][j] += I_KH[i][k] * state->P[k][j];
            }
        }
    }

    // Vérification de la nouvelle covariance
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            if (isnan(P_new[i][j]) || isinf(P_new[i][j])) {
                printf("KALMANERROR 5\n"); // Invalid covariance after update
                return;
            }
        }

    memcpy(state->P, P_new, sizeof(P_new));
}

