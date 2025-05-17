#include "../main.h"
#include "lib_asserv.h"


KalmanState kalman_current_state;

// Bruit de processus Q (tunable)
float Q[STATE_SIZE][STATE_SIZE] = {
    {PROCESS_NOISE_X, 0, 0},
    {0, PROCESS_NOISE_Y, 0},
    {0, 0, PROCESS_NOISE_THETA}
};


void kalman_init(KalmanState* state) {
    memset(state->x, 0, sizeof(state->x));
    memset(state->P, 0, sizeof(state->P));
    for (int i = 0; i < STATE_SIZE; i++)
        state->P[i][i] = 0.01f;  // initial uncertainty
}

void kalman_predict(KalmanState* state, Speed* speed, float dt) {
    float theta = state->x[2];

    // Transformation de la vitesse robot → monde
    float v_dx = speed->vx * cosf(theta) - speed->vy * sinf(theta);
    float v_dy = speed->vx * sinf(theta) + speed->vy * cosf(theta);
    float v_lin = sqrtf(v_dx * v_dx + v_dy * v_dy);
    float v_ang = speed->vt;


    float dx = v_dx * dt;
    float dy = v_dy * dt;
    float dtheta = v_ang * dt;

    // Mise à jour état
    state->x[0] += dx;
    state->x[1] += dy;
    state->x[2] = principal_angle(state->x[2] + dtheta);
    // Mise à jour des vitesses dans le repère monde dans l'état
    state->x[3] = v_dx;      // vx monde
    state->x[4] = v_dy;      // vy monde
    state->x[5] = v_ang;     // vitesse angulaire

    // Jacobienne de la fonction de transition
    float F[STATE_SIZE][STATE_SIZE] = {
        {1, 0, -v_lin * sinf(theta) * dt},
        {0, 1,  v_lin * cosf(theta) * dt},
        {0, 0, 1}
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

void kalman_update(KalmanState* state, float z[STATE_SIZE], float R_diag[STATE_SIZE]) {
    // z = [x, y, theta] mesure LiDAR
    // state->x = [x, y, theta] état prédit

    // Innovation y = z - h(x)
    float y[STATE_SIZE];
    y[0] = z[0] - state->x[0];
    y[1] = z[1] - state->x[1];
    y[2] = principal_angle(z[2] - state->x[2]);

    // Jacobienne H est identité 3x3 car mesure directe de l'état
    // H = I

    // S = H * P * H^T + R = P + R (R est matrice diagonale)
    float S[STATE_SIZE][STATE_SIZE];
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            S[i][j] = state->P[i][j];
            if (i == j) S[i][j] += R_diag[i];
        }
    }

    // Calcul de l'inverse de S (3x3) - on fait l'inverse exacte d'une matrice 3x3 symétrique positive définie
    // Méthode directe pour matrice 3x3 inversible

    float det = 
        S[0][0] * (S[1][1]*S[2][2] - S[1][2]*S[2][1]) -
        S[0][1] * (S[1][0]*S[2][2] - S[1][2]*S[2][0]) +
        S[0][2] * (S[1][0]*S[2][1] - S[1][1]*S[2][0]);

    if (fabs(det) < 1e-6f) {
        // Matrice quasi singulière, on skip la mise à jour
        return;
    }

    float inv_det = 1.0f / det;
    float S_inv[STATE_SIZE][STATE_SIZE];

    S_inv[0][0] =  (S[1][1]*S[2][2] - S[1][2]*S[2][1]) * inv_det;
    S_inv[0][1] = -(S[0][1]*S[2][2] - S[0][2]*S[2][1]) * inv_det;
    S_inv[0][2] =  (S[0][1]*S[1][2] - S[0][2]*S[1][1]) * inv_det;

    S_inv[1][0] = -(S[1][0]*S[2][2] - S[1][2]*S[2][0]) * inv_det;
    S_inv[1][1] =  (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * inv_det;
    S_inv[1][2] = -(S[0][0]*S[1][2] - S[0][2]*S[1][0]) * inv_det;

    S_inv[2][0] =  (S[1][0]*S[2][1] - S[1][1]*S[2][0]) * inv_det;
    S_inv[2][1] = -(S[0][0]*S[2][1] - S[0][1]*S[2][0]) * inv_det;
    S_inv[2][2] =  (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * inv_det;

    // Calcul de K = P * H^T * S^-1 = P * S^-1 (car H=I)
    float K[STATE_SIZE][STATE_SIZE] = {0};
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            for (int k = 0; k < STATE_SIZE; k++) {
                K[i][j] += state->P[i][k] * S_inv[k][j];
            }
        }
    }

    // Mise à jour de l'état x = x + K * y
    for (int i = 0; i < STATE_SIZE; i++) {
        float delta = 0.0f;
        for (int j = 0; j < STATE_SIZE; j++) {
            delta += K[i][j] * y[j];
        }
        if (i == 2) // angle
            state->x[i] = principal_angle(state->x[i] + delta);
        else
            state->x[i] += delta;
    }

    // Mise à jour de la covariance P = (I - K * H) * P = (I - K) * P (car H=I)
    float I_K[STATE_SIZE][STATE_SIZE];
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            I_K[i][j] = (i == j ? 1.0f : 0.0f) - K[i][j];
        }
    }

    float P_new[STATE_SIZE][STATE_SIZE] = {0};
    for (int i = 0; i < STATE_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            for (int k = 0; k < STATE_SIZE; k++) {
                P_new[i][j] += I_K[i][k] * state->P[k][j];
            }
        }
    }

    memcpy(state->P, P_new, sizeof(P_new));
}

