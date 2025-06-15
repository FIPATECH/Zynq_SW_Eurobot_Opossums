#include "../main.h"
#include "lib_asserv.h"


KalmanState kalman_current_state;

// Bruit de processus Q (tunable) pour l'odométrie
float Q[STATE_SIZE][STATE_SIZE] = {
    {PROCESS_NOISE_ODOM_X * PROCESS_NOISE_ODOM_X, 0, 0, 0, 0, 0},
    {0, PROCESS_NOISE_ODOM_Y * PROCESS_NOISE_ODOM_Y, 0, 0, 0, 0},
    {0, 0, PROCESS_NOISE_ODOM_THETA * PROCESS_NOISE_ODOM_THETA, 0, 0, 0},
    {0, 0, 0, PROCESS_NOISE_ODOM_VX * PROCESS_NOISE_ODOM_VX, 0, 0},
    {0, 0, 0, 0, PROCESS_NOISE_ODOM_VY * PROCESS_NOISE_ODOM_VY, 0},
    {0, 0, 0, 0, 0, PROCESS_NOISE_ODOM_VTHETA * PROCESS_NOISE_ODOM_VTHETA}
};

float R_diag[3] = {PROCESS_NOISE_LIDAR_X * PROCESS_NOISE_LIDAR_X,
                    PROCESS_NOISE_LIDAR_Y * PROCESS_NOISE_LIDAR_Y, 
                    PROCESS_NOISE_LIDAR_THETA * PROCESS_NOISE_LIDAR_THETA}; // Bruit de mesure lidar


void kalman_init(KalmanState* state) {
    memset(state->x, 0, sizeof(state->x));
    memset(state->P, 0, sizeof(state->P));

    state->P[0][0] = 1.0f;                     
    state->P[1][1] = 1.0f;
    state->P[2][2] = 1.0f;                     // theta: ±10°
    state->P[3][3] = 1.0f;                     // vx: ±0.2 m/s
    state->P[4][4] = 1.0f;                     // vy: ±0.2 m/s
    state->P[5][5] = 1.0f;                     // vtheta: ±30°/s
}

void kalman_predict(KalmanState* state, Speed* speed, float dt) {
    float theta = principal_angle(state->x[2]);
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);

    // Transformation des vitesses robot → monde
    float v_dx = speed->vx * cos_theta - speed->vy * sin_theta;
    float v_dy = speed->vx * sin_theta + speed->vy * cos_theta;
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
    float F[STATE_SIZE][STATE_SIZE] = {
        {1, 0, (-speed->vx * sin_theta - speed->vy * cos_theta) * dt, cos_theta * dt, -sin_theta * dt, 0},
        {0, 1, ( speed->vx * cos_theta - speed->vy * sin_theta) * dt, sin_theta * dt,  cos_theta * dt, 0},
        {0, 0, 1, 0, 0, dt},
        {0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 1}
    };

    // Mise à jour de la covariance : P = F * P * F^T + Q
    float P_temp[STATE_SIZE][STATE_SIZE] = {0};

    for (int i = 0; i < STATE_SIZE; i++)
        for (int k = 0; k < STATE_SIZE; k++)
            if (F[i][k] != 0.0f)
                for (int j = 0; j < STATE_SIZE; j++)
                    P_temp[i][j] += F[i][k] * state->P[k][j];

    for (int i = 0; i < STATE_SIZE; i++)
        for (int j = 0; j < STATE_SIZE; j++) {
            state->P[i][j] = Q[i][j];
            for (int k = 0; k < STATE_SIZE; k++)
                if (P_temp[i][k] != 0.0f)
                    state->P[i][j] += P_temp[i][k] * F[j][k];
        }
}

void kalman_update(KalmanState* state, float z[3]) {
    for (int i = 0; i < 3; i++) {
        if (isnan(z[i]) || isnan(state->x[i]))
            return;
    }

    float y[3] = { z[0]-state->x[0], z[1]-state->x[1], principal_angle(z[2]-state->x[2]) };

    float S[3][3] = {
        {state->P[0][0]+R_diag[0], state->P[0][1], state->P[0][2]},
        {state->P[1][0], state->P[1][1]+R_diag[1], state->P[1][2]},
        {state->P[2][0], state->P[2][1], state->P[2][2]+R_diag[2]}
    };

    float det = S[0][0]*(S[1][1]*S[2][2]-S[1][2]*S[2][1]) -
                S[0][1]*(S[1][0]*S[2][2]-S[1][2]*S[2][0]) +
                S[0][2]*(S[1][0]*S[2][1]-S[1][1]*S[2][0]);
    if (fabsf(det) < 1e-6f) return;
    float invDet = 1.0f / det;

    float S_inv[3][3] = {
        {(S[1][1]*S[2][2]-S[1][2]*S[2][1])*invDet, -(S[0][1]*S[2][2]-S[0][2]*S[2][1])*invDet, (S[0][1]*S[1][2]-S[0][2]*S[1][1])*invDet},
        {-(S[1][0]*S[2][2]-S[1][2]*S[2][0])*invDet, (S[0][0]*S[2][2]-S[0][2]*S[2][0])*invDet, -(S[0][0]*S[1][2]-S[0][2]*S[1][0])*invDet},
        {(S[1][0]*S[2][1]-S[1][1]*S[2][0])*invDet, -(S[0][0]*S[2][1]-S[0][1]*S[2][0])*invDet, (S[0][0]*S[1][1]-S[0][1]*S[1][0])*invDet}
    };

    float K[6][3] = {0};
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                K[i][j] += state->P[i][k] * S_inv[k][j];

    for (int i = 0; i < 6; i++) {
        float delta = K[i][0]*y[0] + K[i][1]*y[1] + K[i][2]*y[2];
        state->x[i] = (i==2) ? principal_angle(state->x[i]+delta) : state->x[i]+delta;
    }

    float P_new[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            float val = state->P[i][j];
            for (int k = 0; k < 3; k++)
                val -= K[i][k] * state->P[k][j];
            P_new[i][j] = val;
        }
    memcpy(state->P, P_new, sizeof(P_new));
}

