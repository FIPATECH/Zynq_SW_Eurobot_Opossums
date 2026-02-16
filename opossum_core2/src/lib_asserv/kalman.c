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

// float R_diag[3] = {PROCESS_NOISE_LIDAR_X * PROCESS_NOISE_LIDAR_X,
//                     PROCESS_NOISE_LIDAR_Y * PROCESS_NOISE_LIDAR_Y, 
//                     PROCESS_NOISE_LIDAR_THETA * PROCESS_NOISE_LIDAR_THETA}; // Bruit de mesure lidar


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
    // 1. State Prediction (Algebraic integration)
    float theta = principal_angle(state->x[2]);
    float cos_t = cosf(theta);
    float sin_t = sinf(theta);

    float v_dx = speed->vx * cos_t - speed->vy * sin_t;
    float v_dy = speed->vx * sin_t + speed->vy * cos_t;
    float v_ang = speed->vt;

    state->x[0] += v_dx * dt;
    state->x[1] += v_dy * dt;
    state->x[2] = principal_angle(state->x[2] + v_ang * dt);
    state->x[3] = v_dx;
    state->x[4] = v_dy;
    state->x[5] = v_ang;

    // 2. Covariance Prediction: P = F*P*F' + Q
    // Since F is [I, dt*I; 0, I], we compute only the upper triangle 
    // and mirror it, saving roughly 50% of the math.
    float dt2 = dt * dt;

    // Temporary capture of necessary P values to avoid "polluting" calculations mid-way
    // Focusing on the Position block (0,1,2) and Velocity block (3,4,5)
    
    // Diagonal Elements
    state->P[0][0] += dt * (state->P[0][3] + state->P[3][0]) + dt2 * state->P[3][3] + Q[0][0];
    state->P[1][1] += dt * (state->P[1][4] + state->P[4][1]) + dt2 * state->P[4][4] + Q[1][1];
    state->P[2][2] += dt * (state->P[2][5] + state->P[5][2]) + dt2 * state->P[5][5] + Q[2][2];

    // Off-diagonal Position/Position
    state->P[0][1] += dt * (state->P[0][4] + state->P[3][1]) + dt2 * state->P[3][4];
    state->P[0][2] += dt * (state->P[0][5] + state->P[3][2]) + dt2 * state->P[3][5];
    state->P[1][2] += dt * (state->P[1][5] + state->P[4][2]) + dt2 * state->P[4][5];

    // Position/Velocity Covariances (Top-Right 3x3)
    state->P[0][3] += dt * state->P[3][3]; state->P[0][4] += dt * state->P[3][4]; state->P[0][5] += dt * state->P[3][5];
    state->P[1][3] += dt * state->P[4][3]; state->P[1][4] += dt * state->P[4][4]; state->P[1][5] += dt * state->P[4][5];
    state->P[2][3] += dt * state->P[5][3]; state->P[2][4] += dt * state->P[5][4]; state->P[2][5] += dt * state->P[5][5];

    // Velocity block (Bottom-Right 3x3) - Only add Q
    state->P[3][3] += Q[3][3]; state->P[3][4] += Q[3][4]; state->P[3][5] += Q[3][5];
    state->P[4][4] += Q[4][4]; state->P[4][5] += Q[4][5];
    state->P[5][5] += Q[5][5];

    // Mirror to Lower Triangle (Ensure perfect symmetry)
    state->P[1][0] = state->P[0][1];
    state->P[2][0] = state->P[0][2]; state->P[2][1] = state->P[1][2];
    state->P[3][0] = state->P[0][3]; state->P[3][1] = state->P[1][3]; state->P[3][2] = state->P[2][3];
    state->P[4][0] = state->P[0][4]; state->P[4][1] = state->P[1][4]; state->P[4][2] = state->P[2][4]; state->P[4][3] = state->P[3][4];
    state->P[5][0] = state->P[0][5]; state->P[5][1] = state->P[1][5]; state->P[5][2] = state->P[2][5]; state->P[5][3] = state->P[3][5]; state->P[5][4] = state->P[4][5];
}

// Seuil de la loi du Chi-Carré pour 3 degrés de liberté (x, y, theta) à 99% de confiance
#define CHI2_THRESHOLD_99 11.34f

uint8_t kalman_update(KalmanState* state, float z[3], float R_diag[3], uint8_t bypass_outlier_rejection) {
    // Vérifs NaN
    for (int i = 0; i < 3; ++i) {
        if (isnan(z[i]) || isnan(state->x[i])) return 2; // erreur d’état ou de mesure
    }

    // H = [ I3  0 ] (mesure = x,y,theta)
    // innovation y = z - H x
    float y0 = z[0] - state->x[0];
    float y1 = z[1] - state->x[1];
    float y2 = principal_angle(z[2] - state->x[2]);

    // S = H P H^T + R = top-left 3x3 of P + R_diag
    float S[3][3];
    S[0][0] = state->P[0][0] + R_diag[0];
    S[0][1] = state->P[0][1];
    S[0][2] = state->P[0][2];

    S[1][0] = state->P[1][0];
    S[1][1] = state->P[1][1] + R_diag[1];
    S[1][2] = state->P[1][2];

    S[2][0] = state->P[2][0];
    S[2][1] = state->P[2][1];
    S[2][2] = state->P[2][2] + R_diag[2];

    // Inversion 3x3 de S avec régularisation numérique si besoin
    // calcul du déterminant
    float det = S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1])
              - S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0])
              + S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);

    // si mal conditionné, ajouter eps sur la diagonale puis recalculer (simple fallback)
    if (fabsf(det) < S_INV_EPS) {
        S[0][0] += S_INV_EPS;
        S[1][1] += S_INV_EPS;
        S[2][2] += S_INV_EPS;
        det = S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1])
            - S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0])
            + S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);
        if (fabsf(det) < S_INV_EPS) return 3; // toujours mal conditionné : abandonner update
    }

    float invDet = 1.0f / det;
    float S_inv[3][3];

    S_inv[0][0] =  (S[1][1]*S[2][2] - S[1][2]*S[2][1]) * invDet;
    S_inv[0][1] = -(S[0][1]*S[2][2] - S[0][2]*S[2][1]) * invDet;
    S_inv[0][2] =  (S[0][1]*S[1][2] - S[0][2]*S[1][1]) * invDet;

    S_inv[1][0] = -(S[1][0]*S[2][2] - S[1][2]*S[2][0]) * invDet;
    S_inv[1][1] =  (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * invDet;
    S_inv[1][2] = -(S[0][0]*S[1][2] - S[0][2]*S[1][0]) * invDet;

    S_inv[2][0] =  (S[1][0]*S[2][1] - S[1][1]*S[2][0]) * invDet;
    S_inv[2][1] = -(S[0][0]*S[2][1] - S[0][1]*S[2][0]) * invDet;
    S_inv[2][2] =  (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * invDet;


    // distance de Mahalanobis au carré pour rejection d'outliers
    float mahalanobis_sq = y0 * (y0*S_inv[0][0] + y1*S_inv[1][0] + y2*S_inv[2][0]) +
                           y1 * (y0*S_inv[0][1] + y1*S_inv[1][1] + y2*S_inv[2][1]) +
                           y2 * (y0*S_inv[0][2] + y1*S_inv[1][2] + y2*S_inv[2][2]);

    // Rejection d'outliers basée sur la distance de Mahalanobis : y^T S^-1 y > seuil chi2
    if (!bypass_outlier_rejection && mahalanobis_sq > CHI2_THRESHOLD_99) {
        return 1; // abandonner update
    }

    // Gain K = P * H^T * S_inv
    // H^T = [ I3; 0 ] => K rows 0..5, cols 0..2:
    float K[6][3];
    // pour i=0..5, K[i] = [ P[i][0], P[i][1], P[i][2] ] * S_inv
    for (int i = 0; i < 6; ++i) {
        // produit 1x3 = 1x3 * 3x3
        for (int j = 0; j < 3; ++j) {
            K[i][j] = state->P[i][0] * S_inv[0][j]
                    + state->P[i][1] * S_inv[1][j]
                    + state->P[i][2] * S_inv[2][j];
        }
    }

    // Mise à jour de l'état : x = x + K * y
    float dy[3] = { y0, y1, y2 };
    for (int i = 0; i < 6; ++i) {
        float delta = K[i][0]*dy[0] + K[i][1]*dy[1] + K[i][2]*dy[2];
        if (i == 2)
            state->x[i] = principal_angle(state->x[i] + delta);
        else
            state->x[i] += delta;
    }

    // Mise à jour de la covariance P via la forme Joseph :
    // P = (I - K H) P (I - K H)^T + K R K^T
    // Avec H = [I3 0], (I - K H) est une 6x6 = I6 ; les premiers 3 cols of KH are K[:,0..2] in rows 0..5
    // On calcule explicitement pour stabilité.

    // Precompute (I-KH) matrix works as:
    // (I-KH)[i][j] = -K[i][j] for j=0..2, else 1 if i==j (for j>=3), else 0
    // But better compute P_new using expanded Joseph form to avoid building large intermediates.

    float P_new[6][6];
    // Compute A = (I - K*H)
    // Then P_new = A * P * A^T + K * R * K^T
    // Because H picks top-left, A is simple. We'll compute directly with unrolled sums to reduce temporaries.

    // First compute A * P  (A is 6x6)
    // For row i, col j:
    // (A*P)[i][j] = P[i][j] - sum_{m=0..2} K[i][m] * P[m][j]
    float AP[6][6];
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float val = state->P[i][j];
            val -= K[i][0] * state->P[0][j];
            val -= K[i][1] * state->P[1][j];
            val -= K[i][2] * state->P[2][j];
            AP[i][j] = val;
        }
    }

    // Then P_new = AP * A^T + K * R * K^T
    // Note A^T: (AP * A^T)[i][j] = AP[i][j] - sum_{m=0..2} AP[i][m]*K[j][m]
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float val = AP[i][j];
            val -= AP[i][0] * K[j][0];
            val -= AP[i][1] * K[j][1];
            val -= AP[i][2] * K[j][2];
            P_new[i][j] = val;
        }
    }

    // Add K * R * K^T  (R is diagonal R_diag)
    // (K R K^T)[i][j] = sum_m K[i][m] * R_diag[m] * K[j][m]
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float add = K[i][0] * R_diag[0] * K[j][0]
                      + K[i][1] * R_diag[1] * K[j][1]
                      + K[i][2] * R_diag[2] * K[j][2];
            P_new[i][j] += add;
        }
    }

    // Symétriser P_new et copier dans state->P
    for (int i = 0; i < 6; ++i) {
        for (int j = i; j < 6; ++j) {
            float s = 0.5f * (P_new[i][j] + P_new[j][i]);
            state->P[i][j] = s;
            state->P[j][i] = s;
        }
    }

    return 0; // update réussi
}