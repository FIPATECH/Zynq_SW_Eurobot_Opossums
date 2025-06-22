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

    float cos_theta_dt = cosf(theta + speed->vt * dt);
    float sin_theta_dt = sinf(theta + speed->vt * dt);

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
        {1, 0, -v_dy * dt, cos_theta * dt, -sin_theta * dt, 0},
        {0, 1, v_dx * dt, sin_theta * dt,  cos_theta * dt, 0},
        {0, 0, 1, 0, 0, dt},
        {0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 1}
    };

    float P_temp[6][6];
    
    // i=0
    P_temp[0][0] = state->P[0][0] + (-v_dy*dt)*state->P[2][0] + (cos_theta_dt)*state->P[3][0] + (-sin_theta_dt)*state->P[4][0];
    P_temp[0][1] = state->P[0][1] + (-v_dy*dt)*state->P[2][1] + (cos_theta_dt)*state->P[3][1] + (-sin_theta_dt)*state->P[4][1];
    P_temp[0][2] = state->P[0][2] + (-v_dy*dt)*state->P[2][2] + (cos_theta_dt)*state->P[3][2] + (-sin_theta_dt)*state->P[4][2];
    P_temp[0][3] = state->P[0][3] + (-v_dy*dt)*state->P[2][3] + (cos_theta_dt)*state->P[3][3] + (-sin_theta_dt)*state->P[4][3];
    P_temp[0][4] = state->P[0][4] + (-v_dy*dt)*state->P[2][4] + (cos_theta_dt)*state->P[3][4] + (-sin_theta_dt)*state->P[4][4];
    P_temp[0][5] = state->P[0][5] + (-v_dy*dt)*state->P[2][5] + (cos_theta_dt)*state->P[3][5] + (-sin_theta_dt)*state->P[4][5];
    
    // i=1
    P_temp[1][0] = state->P[1][0] + (v_dx*dt)*state->P[2][0] + (sin_theta_dt)*state->P[3][0] + (cos_theta_dt)*state->P[4][0];
    P_temp[1][1] = state->P[1][1] + (v_dx*dt)*state->P[2][1] + (sin_theta_dt)*state->P[3][1] + (cos_theta_dt)*state->P[4][1];
    P_temp[1][2] = state->P[1][2] + (v_dx*dt)*state->P[2][2] + (sin_theta_dt)*state->P[3][2] + (cos_theta_dt)*state->P[4][2];
    P_temp[1][3] = state->P[1][3] + (v_dx*dt)*state->P[2][3] + (sin_theta_dt)*state->P[3][3] + (cos_theta_dt)*state->P[4][3];
    P_temp[1][4] = state->P[1][4] + (v_dx*dt)*state->P[2][4] + (sin_theta_dt)*state->P[3][4] + (cos_theta_dt)*state->P[4][4];
    P_temp[1][5] = state->P[1][5] + (v_dx*dt)*state->P[2][5] + (sin_theta_dt)*state->P[3][5] + (cos_theta_dt)*state->P[4][5];

    // i=2
    P_temp[2][0] = state->P[2][0] + dt*state->P[5][0];
    P_temp[2][1] = state->P[2][1] + dt*state->P[5][1];
    P_temp[2][2] = state->P[2][2] + dt*state->P[5][2];
    P_temp[2][3] = state->P[2][3] + dt*state->P[5][3];
    P_temp[2][4] = state->P[2][4] + dt*state->P[5][4];
    P_temp[2][5] = state->P[2][5] + dt*state->P[5][5];

    // i=3
    for (int j = 0; j < 6; j++) {
        P_temp[3][j] = state->P[3][j]; // F[3][3]=1
    }

    // i=4
    for (int j = 0; j < 6; j++) {
        P_temp[4][j] = state->P[4][j]; // F[4][4]=1
    }

    // i=5
    for (int j = 0; j < 6; j++) {
        P_temp[5][j] = state->P[5][j]; // F[5][5]=1
    }

    // Maintenant multiplication finale : P = P_temp * F^T + Q
    // F^T est la transposée de F,
    // donc F^T[j][k] = F[k][j]

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            // Initialisation avec Q[i][j]
            float val = Q[i][j];

            // somme sur k
            val += P_temp[i][0] * F[j][0];
            val += P_temp[i][1] * F[j][1];
            val += P_temp[i][2] * F[j][2];
            val += P_temp[i][3] * F[j][3];
            val += P_temp[i][4] * F[j][4];
            val += P_temp[i][5] * F[j][5];

            state->P[i][j] = val;
        }
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

